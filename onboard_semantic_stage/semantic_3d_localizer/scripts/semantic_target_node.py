#!/usr/bin/env python3
"""Safe-by-default YOLO-World + aligned-depth semantic 3-D localization."""
import json
import math
import os
import sys
import threading

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transforms
import tf2_ros
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, String
from ultralytics import YOLOWorld

sys.path.insert(0, os.path.dirname(__file__))
from semantic_geometry import camera_to_world, pinhole_point, standoff_goal


def image_to_numpy(msg):
    """Convert RealSense ROS image encodings without the broken cv_bridge."""
    encoding = msg.encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        return row[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy(), encoding
    if encoding in ("mono8", "8uc1"):
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        return row[:, :msg.width].copy(), encoding
    if encoding in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        row = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)
        return row[:, :msg.width].copy(), encoding
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        row = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 4)
        return row[:, :msg.width].copy(), encoding
    raise ValueError("unsupported image encoding: %s" % msg.encoding)


def numpy_to_bgr8(array, header):
    array = np.ascontiguousarray(array, dtype=np.uint8)
    msg = Image()
    msg.header = header
    msg.height, msg.width = array.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = array.tobytes()
    return msg


def robust_bbox_depth(depth_m, bbox, min_depth, max_depth, crop_fraction):
    """Median depth in the central bbox region, with MAD outlier rejection."""
    height, width = depth_m.shape[:2]
    x1, y1, x2, y2 = [float(value) for value in bbox]
    margin_x = crop_fraction * max(0.0, x2 - x1)
    margin_y = crop_fraction * max(0.0, y2 - y1)
    ix1 = max(0, min(width - 1, int(round(x1 + margin_x))))
    ix2 = max(ix1 + 1, min(width, int(round(x2 - margin_x))))
    iy1 = max(0, min(height - 1, int(round(y1 + margin_y))))
    iy2 = max(iy1 + 1, min(height, int(round(y2 - margin_y))))
    roi = depth_m[iy1:iy2, ix1:ix2]
    valid = roi[np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)]
    if valid.size < 9:
        raise ValueError("not enough valid depth samples (%d)" % valid.size)
    median = float(np.median(valid))
    mad = float(np.median(np.abs(valid - median)))
    if mad > 1e-6:
        inliers = valid[np.abs(valid - median) <= 3.5 * 1.4826 * mad]
        if inliers.size >= 9:
            median = float(np.median(inliers))
            valid = inliers
    return median, int(valid.size), mad


class SemanticTargetNode:
    def __init__(self):
        gp = rospy.get_param
        self.execution_enabled = bool(gp("~execution_enabled", False))
        self.publish_planner_goal = bool(gp("~publish_planner_goal", False))
        self.world_frame = gp("~world_frame", "map")
        self.camera_extrinsic_frame = gp("~camera_extrinsic_frame", "camera_infra1_optical_frame")
        self.target_class = gp("~target_class", "person")
        self.classes = gp("~classes", [self.target_class])
        self.confidence = float(gp("~confidence", 0.25))
        self.device = gp("~device", 0)
        self.standoff = float(gp("~standoff", 1.0))
        self.max_rate = float(gp("~max_inference_rate", 2.0))
        self.depth_scale = float(gp("~depth_scale", 0.001))
        self.min_depth = float(gp("~min_depth", 0.15))
        self.max_depth = float(gp("~max_depth", 8.0))
        self.depth_crop_fraction = float(gp("~depth_crop_fraction", 0.25))
        self.calibration_source = gp("~calibration_source", "unspecified")
        self.debug_bbox = gp("~debug_bbox", [])
        flat_extrinsic = gp("~body_T_camera")
        if len(flat_extrinsic) != 16:
            raise ValueError("~body_T_camera must contain a 4x4 row-major matrix")
        matrix = np.asarray(flat_extrinsic, dtype=float).reshape(4, 4)
        self.rotation_body_camera = tuple(tuple(row) for row in matrix[:3, :3])
        self.translation_body_camera = tuple(matrix[:3, 3])

        self.last_inference = rospy.Time(0)
        self.busy = False
        self.lock = threading.Lock()
        self.camera_info = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        weights = os.path.expanduser(gp("~weights", "~/models/yolo_world/yolov8s-worldv2.pt"))
        self.model = YOLOWorld(weights)
        self.model.set_classes(self.classes)

        self.box_pub = rospy.Publisher("~bbox", PolygonStamped, queue_size=1)
        self.json_pub = rospy.Publisher("~detection_json", String, queue_size=1)
        self.target_pub = rospy.Publisher("~target_world", PointStamped, queue_size=1)
        self.candidate_pub = rospy.Publisher("~goal_candidate", PoseStamped, queue_size=1)
        self.yaw_candidate_pub = rospy.Publisher("~yaw_candidate", Float64, queue_size=1)
        self.annotated_pub = rospy.Publisher("~annotated_image", Image, queue_size=1)
        self.goal_pub = None
        if self.execution_enabled and self.publish_planner_goal:
            self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)

        rospy.Subscriber(gp("~camera_info_topic"), CameraInfo, self.on_camera_info, queue_size=1)
        image_sub = message_filters.Subscriber(gp("~image_topic"), Image)
        depth_sub = message_filters.Subscriber(gp("~depth_topic"), Image)
        odom_sub = message_filters.Subscriber(gp("~odom_topic"), Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, odom_sub], queue_size=int(gp("~sync_queue", 15)),
            slop=float(gp("~sync_slop", 0.15)), allow_headerless=False)
        self.sync.registerCallback(self.on_sync)
        rospy.loginfo("semantic target ready: aligned depth, frame=%s, execution=%s, goal=%s",
                      self.world_frame, self.execution_enabled, self.publish_planner_goal)

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_sync(self, image_msg, depth_msg, odom_msg):
        if self.camera_info is None:
            return
        now = rospy.Time.now()
        with self.lock:
            if self.busy or (now - self.last_inference).to_sec() < 1.0 / max(self.max_rate, 0.1):
                return
            self.busy = True
            self.last_inference = now
        try:
            self.process(image_msg, depth_msg, odom_msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "semantic observation rejected: %s", exc)
        finally:
            with self.lock:
                self.busy = False

    def select_detection(self, image):
        if isinstance(self.debug_bbox, (list, tuple)) and len(self.debug_bbox) == 4:
            return 1.0, self.target_class, [float(x) for x in self.debug_bbox], image.copy(), "debug_bbox"
        result = self.model.predict(image, conf=self.confidence, device=self.device, verbose=False)[0]
        candidates = []
        for xyxy, confidence, class_index in zip(result.boxes.xyxy.cpu().tolist(),
                                                  result.boxes.conf.cpu().tolist(),
                                                  result.boxes.cls.cpu().tolist()):
            label = result.names[int(class_index)]
            if label == self.target_class:
                candidates.append((float(confidence), label, xyxy))
        if not candidates:
            return None
        confidence, label, bbox = max(candidates, key=lambda item: item[0])
        return confidence, label, bbox, result.plot(), "yolo_world"

    def process(self, image_msg, depth_msg, odom_msg):
        image_raw, image_encoding = image_to_numpy(image_msg)
        if image_encoding == "rgb8":
            image = cv2.cvtColor(image_raw, cv2.COLOR_RGB2BGR)
        elif image_raw.ndim == 2:
            image = cv2.cvtColor(image_raw, cv2.COLOR_GRAY2BGR)
        else:
            image = image_raw
        selected = self.select_detection(image)
        if selected is None:
            return
        confidence, label, bbox, annotated, detection_source = selected

        depth_raw, depth_encoding = image_to_numpy(depth_msg)
        if depth_encoding in ("16uc1", "mono16"):
            depth_m = depth_raw.astype(np.float32) * self.depth_scale
        elif depth_encoding == "32fc1":
            depth_m = depth_raw.astype(np.float32)
        else:
            raise ValueError("depth image must be 16UC1 or 32FC1")
        if depth_m.shape[:2] != image.shape[:2]:
            raise ValueError("aligned depth and color dimensions differ")
        depth, valid_count, depth_mad = robust_bbox_depth(
            depth_m, bbox, self.min_depth, self.max_depth, self.depth_crop_fraction)

        info = self.camera_info
        fx, fy, cx, cy = float(info.K[0]), float(info.K[4]), float(info.K[2]), float(info.K[5])
        u, v = 0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3])
        point_xyz = pinhole_point(u, v, depth, fx, fy, cx, cy)
        point_camera = PointStamped()
        point_camera.header = image_msg.header
        point_camera.point.x, point_camera.point.y, point_camera.point.z = point_xyz
        if point_camera.header.frame_id != self.camera_extrinsic_frame:
            point_camera = self.tf_buffer.transform(point_camera, self.camera_extrinsic_frame,
                                                    rospy.Duration(0.15))

        pose = odom_msg.pose.pose
        target_xyz = camera_to_world(
            (point_camera.point.x, point_camera.point.y, point_camera.point.z),
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            self.rotation_body_camera, self.translation_body_camera)
        target = PointStamped()
        target.header.stamp = image_msg.header.stamp
        target.header.frame_id = odom_msg.header.frame_id or self.world_frame
        target.point.x, target.point.y, target.point.z = target_xyz

        candidate = None
        yaw = None
        vehicle_xyz = (pose.position.x, pose.position.y, pose.position.z)
        try:
            goal_xyz = standoff_goal(vehicle_xyz, target_xyz, self.standoff)
            candidate = PoseStamped()
            candidate.header = target.header
            candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z = goal_xyz
            candidate.pose.orientation.w = 1.0
            yaw = math.atan2(target.point.y - goal_xyz[1], target.point.x - goal_xyz[0])
        except ValueError:
            goal_xyz = None

        box = PolygonStamped()
        box.header = image_msg.header
        x1, y1, x2, y2 = bbox
        box.polygon.points = [Point32(x=x1, y=y1), Point32(x=x2, y=y1),
                              Point32(x=x2, y=y2), Point32(x=x1, y=y2)]
        self.box_pub.publish(box)
        self.target_pub.publish(target)
        if candidate is not None:
            self.candidate_pub.publish(candidate)
            self.yaw_candidate_pub.publish(Float64(data=yaw))

        payload = {"class": label, "confidence": confidence, "source": detection_source,
                   "bbox_xyxy": [float(x) for x in bbox], "depth_m": depth,
                   "depth_valid_samples": valid_count, "depth_mad_m": depth_mad,
                   "camera_frame": image_msg.header.frame_id,
                   "extrinsic_camera_frame": self.camera_extrinsic_frame,
                   "world_frame": target.header.frame_id, "target_world": list(target_xyz),
                   "goal_candidate": goal_xyz, "calibration_source": self.calibration_source,
                   "execution_gate_open": bool(self.goal_pub)}
        self.json_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
        cv2.putText(annotated, "%s %.2f z=%.2fm" % (label, confidence, depth),
                    (max(0, int(x1)), max(20, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 2, cv2.LINE_AA)
        self.annotated_pub.publish(numpy_to_bgr8(annotated, image_msg.header))
        if self.goal_pub is not None and candidate is not None:
            self.goal_pub.publish(candidate)
        rospy.loginfo("semantic target %s conf=%.3f depth=%.3f source=%s gate=%s",
                      label, confidence, depth, detection_source, bool(self.goal_pub))


if __name__ == "__main__":
    rospy.init_node("semantic_target_node")
    SemanticTargetNode()
    rospy.spin()
