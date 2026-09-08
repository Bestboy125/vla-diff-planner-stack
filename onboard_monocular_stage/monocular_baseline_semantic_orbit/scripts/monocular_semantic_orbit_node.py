#!/usr/bin/env python3
"""Safety-gated two-position D435-left localization and orbit state machine."""
import json
import math
import os
import threading
import time
import sys

import actionlib
import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from atomic_skill_executor.msg import ExecuteAtomicSkillAction, ExecuteAtomicSkillGoal
from camera_coordinate import Observation, RelativeTargetEstimator, TargetTemplate
from camera_coordinate.inputs import undistort_bgr
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from shared_yolo_world_detector.srv import DetectTarget
from std_msgs.msg import Bool, Header, String

# catkin_install_python executes this source through a wrapper in devel/lib.
# Put the real source directory on sys.path so the adjacent pure geometry
# module is importable in both source-tree tests and roslaunch execution.
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from monocular_orbit_geometry import (
    MonocularOrbitError, baseline_in_first_camera, baseline_waypoint,
    bbox_iou_xywh_xyxy, build_orbit_step, camera_pose_world,
    target_world_from_camera, validate_body_t_camera, validate_request,
)


def image_to_bgr(message):
    encoding = message.encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image
    if encoding in ("mono8", "8uc1"):
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        return cv2.cvtColor(rows[:, :message.width].copy(), cv2.COLOR_GRAY2BGR)
    raise MonocularOrbitError("unsupported D435-left image encoding %s" % message.encoding)


def bgr_to_message(image, header):
    image = np.ascontiguousarray(image, dtype=np.uint8)
    result = Image()
    result.header = header
    result.height, result.width = image.shape[:2]
    result.encoding = "bgr8"
    result.is_bigendian = 0
    result.step = result.width * 3
    result.data = image.tobytes()
    return result


def odom_position(message):
    point = message.pose.pose.position
    return np.asarray([point.x, point.y, point.z], dtype=np.float64)


def odom_quaternion(message):
    q = message.pose.pose.orientation
    return (q.x, q.y, q.z, q.w)


def linear_speed(message):
    velocity = message.twist.twist.linear
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)


class Capture(object):
    def __init__(self, image, intrinsics, bbox, detection_confidence,
                 camera_world, rotation_world_camera, odom, stamp):
        self.image = image
        self.intrinsics = intrinsics
        self.bbox = tuple(bbox)
        self.detection_confidence = float(detection_confidence)
        self.camera_world = camera_world
        self.rotation_world_camera = rotation_world_camera
        self.odom = odom
        self.stamp = stamp


class MonocularSemanticOrbitNode(object):
    def __init__(self):
        gp = rospy.get_param
        self.execution_enabled = bool(gp("~execution_enabled", False))
        self.world_frame = str(gp("~world_frame", "world"))
        self.body_frame = str(gp("~body_frame", "base_link"))
        self.camera_frame = str(gp("~camera_frame", "vla_usb_camera_optical_frame"))
        self.allow_empty_odom_child_frame = bool(gp("~allow_empty_odom_child_frame", True))
        self.altitude_reference_z = float(gp("~altitude_reference_z_m", 0.0))
        self.min_altitude = float(gp("~min_operating_altitude", 0.30))
        self.max_altitude = float(gp("~max_operating_altitude", 2.0))
        self.odom_timeout = float(gp("~odom_timeout", 0.30))
        self.capture_timeout = float(gp("~capture_timeout", 20.0))
        self.max_sensor_skew = float(gp("~max_sensor_skew", 0.08))
        self.settle_time = float(gp("~baseline_settle_time", 0.8))
        self.velocity_tolerance = float(gp("~capture_velocity_tolerance", 0.18))
        self.min_baseline = float(gp("~min_measured_baseline_m", 0.45))
        self.max_baseline = float(gp("~max_measured_baseline_m", 1.10))
        self.min_identity_confidence = float(gp("~min_identity_confidence", 0.15))
        self.min_estimate_confidence = float(gp("~min_estimate_confidence", 0.12))
        self.max_relative_range_std = float(gp("~max_relative_range_std", 0.35))
        self.max_approach_leg = float(gp("~max_approach_leg", 2.0))
        self.max_approach_stages = int(gp("~max_approach_stages", 4))
        self.approach_timeout = float(gp("~approach_timeout", 60.0))
        self.orbit_timeout = float(gp("~orbit_timeout", 120.0))
        self.detection_confidence = float(gp("~confidence", 0.20))
        self.device = gp("~device", 0)
        self.debug_bbox = gp("~debug_bbox", [])
        self.detector_backend = str(gp("~detector_backend", "local")).lower()
        self.detector_service = str(
            gp("~detector_service", "/shared_yolo_world_detector/detect")
        )
        self.body_t_camera_param = str(gp("~body_T_camera_param", "")).strip()
        self.extrinsic_frame_param = str(
            gp("~camera_extrinsic_frame_param", "")
        ).strip()
        if self.detector_backend not in ("local", "shared_service"):
            raise RuntimeError("~detector_backend must be local or shared_service")
        if bool(self.body_t_camera_param) != bool(self.extrinsic_frame_param):
            raise RuntimeError(
                "body_T_camera_param and camera_extrinsic_frame_param must be set together"
            )
        self.fixed_body_t_camera = None
        if self.body_t_camera_param:
            calibrated_frame = str(gp(self.extrinsic_frame_param))
            if calibrated_frame != self.camera_frame:
                raise RuntimeError(
                    "fixed camera extrinsic frame %s does not match image frame %s" %
                    (calibrated_frame, self.camera_frame)
                )
            self.fixed_body_t_camera = validate_body_t_camera(
                gp(self.body_t_camera_param)
            )

        self.lock = threading.RLock()
        self.model_lock = threading.Lock()
        self.busy = False
        self.active = None
        self.phase = None
        self.phase_started = rospy.Time(0)
        self.observation_not_before = rospy.Time(0)
        self.capture_a = None
        self.baseline_goal = None
        self.settle_started = None
        self.target_world = None
        self.approach_stage = 0
        self.latest_odom = None
        self.latest_odom_received = rospy.Time(0)
        self.latest_fcu = None
        self.camera_info = None

        self.target_label = "chair"
        self.model = None
        self.detect_proxy = None
        if self.detector_backend == "local":
            from ultralytics import YOLOWorld
            weights = os.path.expanduser(
                str(gp("~weights", "~/models/yolo_world/yolov8s-worldv2.pt"))
            )
            self.model = YOLOWorld(weights)
            self.model.set_classes([self.target_label])
        else:
            rospy.wait_for_service(self.detector_service, timeout=60.0)
            self.detect_proxy = rospy.ServiceProxy(
                self.detector_service, DetectTarget, persistent=True
            )
        self.estimator = RelativeTargetEstimator(
            min_matches=int(gp("~min_matches", 4)),
            max_reprojection_error_px=float(gp("~max_reprojection_error_px", 4.0)),
            max_target_range_m=float(gp("~max_target_range_m", 50.0)),
        )

        action_name = str(gp("~atomic_action_name", "/atomic_skill_executor/execute"))
        self.client = actionlib.SimpleActionClient(action_name, ExecuteAtomicSkillAction)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self.estimate_pub = rospy.Publisher("~estimate_json", String, queue_size=3, latch=True)
        self.target_pub = rospy.Publisher("~target_world", PointStamped, queue_size=1, latch=True)
        self.capture_a_pub = rospy.Publisher("~capture_a_annotated", Image, queue_size=1, latch=True)
        self.capture_b_pub = rospy.Publisher("~capture_b_annotated", Image, queue_size=1, latch=True)
        self.stereo_inference_pub = rospy.Publisher(
            "/semantic_raw_stereo_node/inference_enabled", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("~request", String, self.on_request, queue_size=1)
        rospy.Subscriber("~cancel", String, self.on_cancel, queue_size=1)
        rospy.Subscriber("~camera_info", CameraInfo, self.on_camera_info, queue_size=1)
        rospy.Subscriber("~fcu_state", State, self.on_fcu, queue_size=1)
        image_sub = message_filters.Subscriber("~image", Image)
        odom_sub = message_filters.Subscriber("~odom", Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, odom_sub], queue_size=int(gp("~sync_queue", 30)),
            slop=float(gp("~sync_slop", 0.04)), allow_headerless=False)
        self.sync.registerCallback(self.on_sync)
        rospy.Timer(rospy.Duration(0.1), self.on_timer)
        self.stereo_inference_pub.publish(Bool(data=True))
        self.publish_status("READY", "two-position D435-left semantic orbit initialized")
        rospy.loginfo("D435-left semantic orbit ready; execution_enabled=%s", self.execution_enabled)

    def publish_status(self, state, detail, extra=None, request_override=None):
        request = request_override if request_override is not None else self.active
        payload = {
            "state": state, "detail": detail,
            "execution_enabled": self.execution_enabled,
            "task_id": request.get("task_id") if request else None,
            "target_label": request.get("target_label") if request else None,
            "time_unix_ms": int(time.time() * 1000),
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def on_camera_info(self, message):
        with self.lock:
            self.camera_info = message

    def on_fcu(self, message):
        with self.lock:
            self.latest_fcu = (bool(message.connected), bool(message.armed), rospy.Time.now())

    def update_odom(self, message):
        with self.lock:
            self.latest_odom = message
            self.latest_odom_received = rospy.Time.now()

    def require_flight_ready(self, odom=None):
        message = odom if odom is not None else self.latest_odom
        if message is None:
            raise MonocularOrbitError("odometry is unavailable")
        if odom is None:
            age = (rospy.Time.now() - self.latest_odom_received).to_sec()
        else:
            age = (rospy.Time.now() - message.header.stamp).to_sec()
        if age < -0.05 or age > self.odom_timeout:
            raise MonocularOrbitError("odometry is stale")
        if message.header.frame_id != self.world_frame:
            raise MonocularOrbitError("odometry frame does not match world frame")
        child = message.child_frame_id.strip()
        if child and child != self.body_frame:
            raise MonocularOrbitError("odometry child frame does not match body frame")
        if not child and not self.allow_empty_odom_child_frame:
            raise MonocularOrbitError("odometry child frame is empty")
        if self.latest_fcu is None:
            raise MonocularOrbitError("FCU state is unavailable")
        connected, armed, stamp = self.latest_fcu
        if (rospy.Time.now() - stamp).to_sec() > 1.0 or not connected or not armed:
            raise MonocularOrbitError("vehicle must already be connected and armed")
        position = odom_position(message)
        agl = float(position[2]) - self.altitude_reference_z
        if not self.min_altitude <= agl <= self.max_altitude:
            raise MonocularOrbitError("vehicle altitude %.3f m AGL is outside safety bounds" % agl)
        return message

    def on_request(self, message):
        request = None
        try:
            request = validate_request(json.loads(message.data))
            with self.lock:
                if not self.execution_enabled:
                    raise MonocularOrbitError("execution is disabled by the onboard safety gate")
                if self.active is not None:
                    raise MonocularOrbitError("another D435-left semantic orbit task is active")
                self.require_flight_ready()
                if self.camera_info is None:
                    raise MonocularOrbitError("camera CameraInfo is unavailable")
                if not self.client.wait_for_server(rospy.Duration(0.5)):
                    raise MonocularOrbitError("atomic skill action server is unavailable")
                with self.model_lock:
                    if self.model is not None:
                        self.model.set_classes([request["target_label"]])
                    self.target_label = request["target_label"]
                self.active = request
                self.phase = "CAPTURE_A"
                self.phase_started = rospy.Time.now()
                self.observation_not_before = self.phase_started
                self.capture_a = None
                self.baseline_goal = None
                self.settle_started = None
                self.target_world = None
                self.approach_stage = 0
                if request["manage_stereo_inference"]:
                    self.stereo_inference_pub.publish(Bool(data=False))
                self.publish_status("CAPTURE_A", "waiting for first stationary target observation")
        except (ValueError, MonocularOrbitError) as exc:
            self.publish_status("REJECTED", str(exc), request_override=request)

    def select_detection(self, image):
        if isinstance(self.debug_bbox, (list, tuple)) and len(self.debug_bbox) == 4:
            return 1.0, tuple(float(value) for value in self.debug_bbox)
        if self.detect_proxy is not None:
            header = Header(stamp=rospy.Time.now(), frame_id=self.camera_frame)
            response = self.detect_proxy(
                bgr_to_message(image, header), self.target_label, self.detection_confidence
            )
            if not response.detected:
                return None
            if len(response.bbox_xyxy) != 4:
                raise MonocularOrbitError("shared detector returned an invalid bounding box")
            return float(response.confidence), tuple(float(value) for value in response.bbox_xyxy)
        result = self.model.predict(
            image, conf=self.detection_confidence, device=self.device, verbose=False
        )[0]
        candidates = []
        for box, score, class_index in zip(result.boxes.xyxy.cpu().tolist(),
                                            result.boxes.conf.cpu().tolist(),
                                            result.boxes.cls.cpu().tolist()):
            if result.names[int(class_index)] == self.target_label:
                candidates.append((float(score), tuple(float(value) for value in box)))
        return max(candidates, key=lambda item: item[0]) if candidates else None

    def camera_extrinsic(self, stamp):
        if self.fixed_body_t_camera is not None:
            transform = self.fixed_body_t_camera
            return (tuple(float(value) for value in transform[:3, 3]),
                    transform[:3, :3].copy())
        try:
            transform = self.tf_buffer.lookup_transform(
                self.body_frame, self.camera_frame, stamp, rospy.Duration(0.2))
        except Exception as exc:
            raise MonocularOrbitError("body-camera TF unavailable at exposure: %s" % exc)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return ((translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w))

    def make_capture(self, image_message, odom_message):
        self.require_flight_ready(odom_message)
        skew = abs((image_message.header.stamp - odom_message.header.stamp).to_sec())
        if skew > self.max_sensor_skew:
            raise MonocularOrbitError("image/odometry skew %.3fs exceeds %.3fs" %
                                      (skew, self.max_sensor_skew))
        if linear_speed(odom_message) > self.velocity_tolerance:
            raise MonocularOrbitError("vehicle is moving too fast for a sharp capture")
        info = self.camera_info
        if info is None or info.width != image_message.width or info.height != image_message.height:
            raise MonocularOrbitError("CameraInfo dimensions do not match image")
        if info.header.frame_id and info.header.frame_id != self.camera_frame:
            raise MonocularOrbitError("CameraInfo frame does not match calibrated camera frame")
        image = image_to_bgr(image_message)
        intrinsics = np.asarray(info.K, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.D, dtype=np.float64)
        image, rectified_k = undistort_bgr(image, intrinsics, distortion, intrinsics)
        with self.model_lock:
            detection = self.select_detection(image)
        if detection is None:
            raise MonocularOrbitError("YOLO-World found no %s" % self.target_label)
        confidence, bbox = detection
        translation, rotation = self.camera_extrinsic(image_message.header.stamp)
        camera_world, rotation_world_camera = camera_pose_world(
            odom_position(odom_message), odom_quaternion(odom_message), translation, rotation)
        return Capture(image, rectified_k, bbox, confidence, camera_world,
                       rotation_world_camera, odom_message, image_message.header.stamp)

    def publish_annotated(self, capture, publisher, text):
        annotated = capture.image.copy()
        x1, y1, x2, y2 = [int(round(value)) for value in capture.bbox]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(annotated, text, (max(0, x1), max(20, y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
        header = Header(stamp=capture.stamp, frame_id=self.camera_frame)
        publisher.publish(bgr_to_message(annotated, header))

    def on_sync(self, image_message, odom_message):
        self.update_odom(odom_message)
        with self.lock:
            if (self.active is None or self.phase not in ("CAPTURE_A", "CAPTURE_B")
                    or self.busy or image_message.header.stamp < self.observation_not_before):
                return
            self.busy = True
            phase = self.phase
        try:
            capture = self.make_capture(image_message, odom_message)
            with self.lock:
                if self.active is None or self.phase != phase:
                    return
                if phase == "CAPTURE_A":
                    self.accept_capture_a(capture)
                else:
                    self.accept_capture_b(capture)
        except MonocularOrbitError as exc:
            rospy.logwarn_throttle(2.0, "D435-left capture rejected: %s", exc)
        except Exception as exc:
            self.fail("D435-left estimator failed: %s" % exc)
        finally:
            with self.lock:
                self.busy = False

    def accept_capture_a(self, capture):
        request = self.active
        self.capture_a = capture
        self.publish_annotated(capture, self.capture_a_pub,
                               "%s A %.2f" % (request["target_label"], capture.detection_confidence))
        pose = capture.odom.pose.pose
        waypoint = baseline_waypoint(
            odom_position(capture.odom),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            request["baseline_distance_m"], request["baseline_direction"])
        self.baseline_goal = waypoint
        goal = ExecuteAtomicSkillGoal()
        goal.skill = "GOTO_WORLD"
        goal.center.x, goal.center.y, goal.center.z = waypoint
        goal.center_frame = "world"
        goal.timeout = self.approach_timeout
        self.phase = "MOVING_BASELINE"
        self.phase_started = rospy.Time.now()
        self.client.send_goal(goal, done_cb=self.on_baseline_done,
                              feedback_cb=self.on_action_feedback)
        self.publish_status(
            "MOVING_BASELINE", "first photo captured; moving to second viewpoint",
            {"capture_a_unix_ns": capture.stamp.to_nsec(),
             "baseline_direction": request["baseline_direction"],
             "planned_baseline_m": request["baseline_distance_m"],
             "second_viewpoint_world": list(waypoint)})

    def on_baseline_done(self, state, result):
        with self.lock:
            if self.active is None or self.phase != "MOVING_BASELINE":
                return
            if not (state == GoalStatus.SUCCEEDED and result and result.success):
                self.fail(result.message if result else "baseline movement returned no result")
                return
            self.phase = "SETTLING_B"
            self.phase_started = rospy.Time.now()
            self.settle_started = None
            self.publish_status("SETTLING_B", "second viewpoint reached; waiting for a stationary exposure")

    def accept_capture_b(self, capture_b):
        request = self.active
        capture_a = self.capture_a
        if capture_a is None:
            raise MonocularOrbitError("first capture is unavailable")
        measured_baseline = baseline_in_first_camera(
            capture_a.camera_world, capture_a.rotation_world_camera, capture_b.camera_world)
        baseline_norm = float(np.linalg.norm(measured_baseline))
        planned = request["baseline_distance_m"]
        # The command defines the second-viewpoint motion, but geometry always
        # uses the measured camera-centre displacement.  Reject a badly missed
        # move rather than silently estimating with an unintended baseline.
        required_min = max(self.min_baseline, 0.75 * planned)
        required_max = min(self.max_baseline, 1.25 * planned)
        if not required_min <= baseline_norm <= required_max:
            raise MonocularOrbitError(
                "measured baseline %.3f m is outside [%.3f, %.3f] m" %
                (baseline_norm, required_min, required_max))
        observation_a = Observation(capture_a.image, capture_a.rotation_world_camera,
                                    capture_a.intrinsics, capture_a.stamp.to_nsec())
        observation_b = Observation(capture_b.image, capture_b.rotation_world_camera,
                                    capture_b.intrinsics, capture_b.stamp.to_nsec())
        template = TargetTemplate(capture_a.image, capture_a.bbox)
        result = self.estimator.estimate(template, observation_a, observation_b,
                                         measured_baseline)
        if result.estimate_method == "bbox_center":
            raise MonocularOrbitError("bbox-center fallback is forbidden for flight execution")
        if result.identity_confidence < self.min_identity_confidence:
            raise MonocularOrbitError("target identity confidence is too low")
        if result.confidence < self.min_estimate_confidence:
            raise MonocularOrbitError("two-view geometry confidence is too low")
        target_range = float(np.linalg.norm(result.target_camera))
        relative_range_std = result.range_std_m / max(target_range, 1e-6)
        if relative_range_std > self.max_relative_range_std:
            raise MonocularOrbitError("relative range uncertainty is too high")
        overlap = bbox_iou_xywh_xyxy(result.bbox_in_b, capture_b.bbox)
        if overlap < 0.10:
            raise MonocularOrbitError("matched target does not overlap second YOLO detection")
        target_world = target_world_from_camera(
            result.target_camera, capture_b.camera_world, capture_b.rotation_world_camera)
        self.target_world = target_world
        target_message = PointStamped()
        target_message.header.stamp = capture_b.stamp
        target_message.header.frame_id = self.world_frame
        target_message.point.x, target_message.point.y, target_message.point.z = target_world
        self.target_pub.publish(target_message)
        self.publish_annotated(capture_b, self.capture_b_pub,
                               "%s B range=%.2fm" % (request["target_label"], target_range))
        estimate = {
            "task_id": request["task_id"], "target_label": request["target_label"],
            "capture_a_unix_ns": capture_a.stamp.to_nsec(),
            "capture_b_unix_ns": capture_b.stamp.to_nsec(),
            "planned_baseline_m": planned, "measured_baseline_m": baseline_norm,
            "baseline_a_camera_m": measured_baseline.tolist(),
            "target_camera_b_m": result.target_camera.tolist(),
            "target_world_m": list(target_world), "inliers": int(result.inliers),
            "reprojection_error_px": float(result.reprojection_error_px),
            "range_std_m": float(result.range_std_m),
            "relative_range_std": float(relative_range_std),
            "identity_confidence": float(result.identity_confidence),
            "confidence": float(result.confidence), "estimate_method": result.estimate_method,
            "second_detection_iou": float(overlap),
        }
        self.estimate_pub.publish(String(data=json.dumps(estimate, separators=(",", ":"))))
        self.publish_status("TARGET_ESTIMATED", "two-position target estimate passed all gates", estimate)
        if request["localize_only"]:
            self.publish_status(
                "LOCALIZED", "two-position coarse target is ready for a parent mission", estimate
            )
            self.reset()
            return
        self.start_next_navigation_step()

    def start_next_navigation_step(self):
        request = self.active
        current = odom_position(self.require_flight_ready())
        spec = build_orbit_step(request, self.target_world, current, self.max_approach_leg)
        if spec["approach_required"]:
            if self.approach_stage >= self.max_approach_stages:
                raise MonocularOrbitError("circle entry remains too far after maximum approach stages")
            self.approach_stage += 1
            goal = ExecuteAtomicSkillGoal()
            goal.skill = "GOTO_WORLD"
            goal.center.x, goal.center.y, goal.center.z = spec["approach_waypoint_world"]
            goal.center_frame = "world"
            goal.timeout = self.approach_timeout
            self.phase = "APPROACHING"
            self.phase_started = rospy.Time.now()
            self.client.send_goal(goal, done_cb=self.on_approach_done,
                                  feedback_cb=self.on_action_feedback)
            self.publish_status(
                "APPROACHING", "moving toward D435-left estimated orbit center",
                {"stage": self.approach_stage, "max_stages": self.max_approach_stages,
                 "target_world": list(self.target_world),
                 "approach_waypoint_world": list(spec["approach_waypoint_world"]),
                 "remaining_to_entry_m": spec["remaining_to_entry_m"]})
            return
        goal = ExecuteAtomicSkillGoal()
        goal.skill = "ORBIT"
        goal.center.x, goal.center.y, goal.center.z = spec["center"]
        goal.center_frame = "world"
        goal.radius = request["radius_m"]
        goal.orbit_angle = spec["orbit_angle_rad"]
        goal.direction = spec["direction"]
        goal.yaw_mode = request["yaw_mode"]
        goal.timeout = self.orbit_timeout
        self.phase = "ORBITING"
        self.phase_started = rospy.Time.now()
        self.client.send_goal(goal, done_cb=self.on_orbit_done,
                              feedback_cb=self.on_action_feedback)
        self.publish_status("ORBITING", "starting atomic orbit around D435-left target",
                            {"target_world": list(self.target_world),
                             "circle_entry_world": list(spec["entry_world"])})

    def on_approach_done(self, state, result):
        with self.lock:
            if self.active is None or self.phase != "APPROACHING":
                return
            if not (state == GoalStatus.SUCCEEDED and result and result.success):
                self.fail(result.message if result else "approach returned no result")
                return
            try:
                self.start_next_navigation_step()
            except MonocularOrbitError as exc:
                self.fail(str(exc))

    def on_orbit_done(self, state, result):
        with self.lock:
            if self.active is None or self.phase != "ORBITING":
                return
            success = bool(state == GoalStatus.SUCCEEDED and result and result.success)
            detail = result.message if result else "orbit returned no result"
            self.publish_status("SUCCEEDED" if success else "FAILED", detail)
            self.reset()

    def on_action_feedback(self, feedback):
        with self.lock:
            if self.active is None:
                return
            self.publish_status(self.phase or "ACTIVE", "atomic skill progress",
                                {"progress": float(feedback.progress),
                                 "waypoint_index": int(feedback.waypoint_index),
                                 "waypoint_count": int(feedback.waypoint_count)})

    def on_timer(self, _event):
        with self.lock:
            if self.active is None:
                return
            now = rospy.Time.now()
            if self.phase in ("CAPTURE_A", "CAPTURE_B"):
                if (now - self.phase_started).to_sec() > self.capture_timeout:
                    self.fail("target capture timed out")
                return
            if self.phase != "SETTLING_B":
                return
            try:
                odom = self.require_flight_ready()
                error = float(np.linalg.norm(odom_position(odom) - np.asarray(self.baseline_goal)))
                stationary = error <= 0.25 and linear_speed(odom) <= self.velocity_tolerance
                if not stationary:
                    self.settle_started = None
                    return
                if self.settle_started is None:
                    self.settle_started = now
                    return
                if (now - self.settle_started).to_sec() >= self.settle_time:
                    self.phase = "CAPTURE_B"
                    self.phase_started = now
                    self.observation_not_before = now
                    self.publish_status("CAPTURE_B", "capturing second stationary target observation")
            except MonocularOrbitError as exc:
                self.fail(str(exc))

    def on_cancel(self, _message):
        with self.lock:
            if self.active is None:
                return
            self.client.cancel_goal()
            self.publish_status("CANCELLED", "operator HOLD cancelled D435-left semantic orbit")
            self.reset()

    def fail(self, detail):
        self.client.cancel_goal()
        self.publish_status("FAILED", detail)
        self.reset()

    def reset(self):
        if self.active is None or self.active.get("manage_stereo_inference", True):
            self.stereo_inference_pub.publish(Bool(data=True))
        self.active = None
        self.phase = None
        self.capture_a = None
        self.baseline_goal = None
        self.settle_started = None
        self.target_world = None
        self.approach_stage = 0


if __name__ == "__main__":
    rospy.init_node("monocular_semantic_orbit")
    MonocularSemanticOrbitNode()
    rospy.spin()
