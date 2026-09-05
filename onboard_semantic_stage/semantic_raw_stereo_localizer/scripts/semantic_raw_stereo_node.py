#!/usr/bin/env python3
"""YOLO-World + raw D435 infrared stereo target localization, safe by default."""
import json
import os
import re
import sys
import threading
from collections import deque
from copy import deepcopy

import cv2
import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from ultralytics import YOLOWorld

sys.path.insert(0, os.path.dirname(__file__))
from raw_stereo_geometry import (bbox_center_point, body_to_world, camera_to_body,
                                 select_bbox_feature_cluster,
                                 standoff_goal, triangulate_rectified,
                                 validate_projection_matrices)


def image_to_numpy(message):
    encoding = message.encoding.lower()
    if encoding in ("mono8", "8uc1"):
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        return row[:, :message.width].copy()
    if encoding in ("bgr8", "rgb8"):
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = row[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image
    raise ValueError("raw stereo images must be mono8, bgr8 or rgb8, got %s" % message.encoding)


def numpy_to_bgr8(image, header):
    image = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height, message.width = image.shape[:2]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = message.width * 3
    message.data = image.tobytes()
    return message


class OrbStereoMatcher(object):
    def __init__(self, max_features, ratio, bbox_shrink):
        self.orb = cv2.ORB_create(nfeatures=int(max_features), fastThreshold=8,
                                  edgeThreshold=19, patchSize=31)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.ratio = float(ratio)
        self.bbox_shrink = float(bbox_shrink)

    def match(self, left, right, bbox):
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if left.ndim == 3 else left
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if right.ndim == 3 else right
        height, width = left_gray.shape
        x1, y1, x2, y2 = [float(value) for value in bbox]
        margin_x = self.bbox_shrink * max(0.0, x2 - x1)
        margin_y = self.bbox_shrink * max(0.0, y2 - y1)
        ix1 = max(0, min(width - 1, int(round(x1 + margin_x))))
        ix2 = max(ix1 + 1, min(width, int(round(x2 - margin_x))))
        iy1 = max(0, min(height - 1, int(round(y1 + margin_y))))
        iy2 = max(iy1 + 1, min(height, int(round(y2 - margin_y))))
        mask = np.zeros_like(left_gray, dtype=np.uint8)
        mask[iy1:iy2, ix1:ix2] = 255
        keypoints_left, descriptors_left = self.orb.detectAndCompute(left_gray, mask)
        keypoints_right, descriptors_right = self.orb.detectAndCompute(right_gray, None)
        if descriptors_left is None or descriptors_right is None:
            raise ValueError("ORB found no descriptors in one stereo view")
        pairs_forward = self.matcher.knnMatch(descriptors_left, descriptors_right, k=2)
        pairs_reverse = self.matcher.knnMatch(descriptors_right, descriptors_left, k=2)
        reverse_best = {}
        for pair in pairs_reverse:
            if len(pair) != 2:
                continue
            best, second = pair
            if best.distance < self.ratio * second.distance:
                reverse_best[best.queryIdx] = best.trainIdx
        accepted = []
        for pair in pairs_forward:
            if len(pair) != 2:
                continue
            best, second = pair
            if (best.distance < self.ratio * second.distance and
                    reverse_best.get(best.trainIdx) == best.queryIdx):
                accepted.append(best)
        if not accepted:
            raise ValueError("no stereo feature match passed mutual ratio checks")
        accepted.sort(key=lambda match: match.distance)
        points_left = np.asarray([keypoints_left[m.queryIdx].pt for m in accepted], dtype=np.float64)
        points_right = np.asarray([keypoints_right[m.trainIdx].pt for m in accepted], dtype=np.float64)
        return points_left, points_right


class DiskLightGlueStereoMatcher(object):
    """Learned two-view correspondence without vendoring restricted SuperPoint code."""
    def __init__(self, device, max_features, confidence, bbox_shrink,
                 lk_refine, lk_window, lk_max_fb_error):
        try:
            import torch
            from kornia.feature import DISK, LightGlue
        except ImportError as exc:
            raise RuntimeError(
                "lightglue backend requires kornia==0.7.2 and kornia_rs") from exc
        self.torch = torch
        if str(device).isdigit():
            device = "cuda:%s" % device
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.extractor = DISK.from_pretrained("depth", device=self.device).eval()
        self.matcher = LightGlue(features="disk").eval().to(self.device)
        self.max_features = int(max_features)
        self.confidence = float(confidence)
        self.bbox_shrink = float(bbox_shrink)
        self.lk_refine = bool(lk_refine)
        self.lk_window = int(lk_window)
        self.lk_max_fb_error = float(lk_max_fb_error)

    def _refine_subpixel(self, left, right, points_left, points_right):
        if not self.lk_refine or not len(points_left):
            return points_left, points_right
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if left.ndim == 3 else left
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if right.ndim == 3 else right
        left_cv = points_left.astype(np.float32).reshape(-1, 1, 2)
        right_initial = points_right.astype(np.float32).reshape(-1, 1, 2)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        window = (self.lk_window, self.lk_window)
        right_refined, status_forward, _error_forward = cv2.calcOpticalFlowPyrLK(
            left_gray, right_gray, left_cv, right_initial.copy(), winSize=window,
            maxLevel=2, criteria=criteria, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
        left_reprojected, status_backward, _error_backward = cv2.calcOpticalFlowPyrLK(
            right_gray, left_gray, right_refined, left_cv.copy(), winSize=window,
            maxLevel=2, criteria=criteria, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
        forward_ok = status_forward.reshape(-1).astype(bool)
        backward_ok = status_backward.reshape(-1).astype(bool)
        fb_error = np.linalg.norm(left_reprojected.reshape(-1, 2) - points_left, axis=1)
        keep = forward_ok & backward_ok & np.isfinite(fb_error)
        keep &= fb_error <= self.lk_max_fb_error
        refined = right_refined.reshape(-1, 2)
        # The D435 inputs are rectified; retain only the refined horizontal coordinate.
        refined[:, 1] = points_left[:, 1]
        return points_left[keep], refined[keep].astype(np.float64)

    def _extract(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        tensor = self.torch.from_numpy(np.ascontiguousarray(gray)).to(
            device=self.device, dtype=self.torch.float32)
        tensor = tensor[None, None].repeat(1, 3, 1, 1) / 255.0
        with self.torch.inference_mode():
            return self.extractor(
                tensor, n=self.max_features, pad_if_not_divisible=True)[0]

    def match(self, left, right, bbox):
        left_features = self._extract(left)
        right_features = self._extract(right)
        height, width = left.shape[:2]

        def data(features):
            return {
                "keypoints": features.keypoints[None],
                "descriptors": features.descriptors[None],
                "image_size": self.torch.tensor(
                    [[width, height]], device=self.device, dtype=self.torch.float32),
            }

        with self.torch.inference_mode():
            result = self.matcher({
                "image0": data(left_features), "image1": data(right_features)})
        match_index = result["matches0"][0]
        match_score = result["matching_scores0"][0]
        valid = (match_index >= 0) & (match_score >= self.confidence)
        left_indices = self.torch.where(valid)[0]
        right_indices = match_index[valid]
        points_left = left_features.keypoints[left_indices].detach().cpu().numpy()
        points_right = right_features.keypoints[right_indices].detach().cpu().numpy()

        x1, y1, x2, y2 = [float(value) for value in bbox]
        margin_x = self.bbox_shrink * max(0.0, x2 - x1)
        margin_y = self.bbox_shrink * max(0.0, y2 - y1)
        keep = ((points_left[:, 0] >= x1 + margin_x) &
                (points_left[:, 0] <= x2 - margin_x) &
                (points_left[:, 1] >= y1 + margin_y) &
                (points_left[:, 1] <= y2 - margin_y))
        points_left, points_right = points_left[keep], points_right[keep]
        if len(points_left) < 4:
            raise ValueError("DISK/LightGlue found fewer than four target-box matches")
        points_left, points_right = self._refine_subpixel(
            left, right, points_left, points_right)
        if len(points_left) < 4:
            raise ValueError("LK forward/backward refinement retained fewer than four matches")
        if len(points_left) >= 8:
            _fundamental, mask = cv2.findFundamentalMat(
                points_left, points_right, cv2.FM_RANSAC, 1.5, 0.995)
            if mask is not None:
                keep = mask.reshape(-1).astype(bool)
                points_left, points_right = points_left[keep], points_right[keep]
        return points_left.astype(np.float64), points_right.astype(np.float64)


class SgbmStereoDepth(object):
    """Independent dense disparity from the two rectified infrared images."""
    def __init__(self, num_disparities, block_size, uniqueness_ratio,
                 speckle_window_size, speckle_range, bbox_shrink,
                 min_valid_pixels):
        num_disparities = int(num_disparities)
        if num_disparities <= 0 or num_disparities % 16:
            raise ValueError("~sgbm_num_disparities must be a positive multiple of 16")
        block_size = int(block_size)
        if block_size < 3 or not block_size % 2:
            raise ValueError("~sgbm_block_size must be odd and at least 3")
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disparities, blockSize=block_size,
            P1=8 * block_size * block_size, P2=32 * block_size * block_size,
            disp12MaxDiff=1, preFilterCap=31,
            uniquenessRatio=int(uniqueness_ratio),
            speckleWindowSize=int(speckle_window_size),
            speckleRange=int(speckle_range), mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.bbox_shrink = float(bbox_shrink)
        self.min_valid_pixels = int(min_valid_pixels)

    def estimate(self, left, right, bbox, left_projection, right_projection,
                 min_disparity, min_depth, max_depth, mad_scale, max_depth_mad):
        left_p, _right_p, baseline = validate_projection_matrices(
            left_projection, right_projection)
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if left.ndim == 3 else left
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if right.ndim == 3 else right
        disparity = self.matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        height, width = disparity.shape
        x1, y1, x2, y2 = [float(value) for value in bbox]
        margin_x = self.bbox_shrink * max(0.0, x2 - x1)
        margin_y = self.bbox_shrink * max(0.0, y2 - y1)
        ix1 = max(0, min(width - 1, int(round(x1 + margin_x))))
        ix2 = max(ix1 + 1, min(width, int(round(x2 - margin_x))))
        iy1 = max(0, min(height - 1, int(round(y1 + margin_y))))
        iy2 = max(iy1 + 1, min(height, int(round(y2 - margin_y))))
        roi_disparity = disparity[iy1:iy2, ix1:ix2]
        valid = np.isfinite(roi_disparity) & (roi_disparity >= float(min_disparity))
        depth = np.full_like(roi_disparity, np.nan, dtype=np.float32)
        depth[valid] = left_p[0, 0] * abs(float(baseline)) / roi_disparity[valid]
        valid &= (depth >= float(min_depth)) & (depth <= float(max_depth))
        values = depth[valid].astype(np.float64)
        if len(values) < self.min_valid_pixels:
            raise ValueError("SGBM valid target pixels %d < %d" %
                             (len(values), self.min_valid_pixels))
        depth_median = float(np.median(values))
        depth_mad = float(np.median(np.abs(values - depth_median)))
        if depth_mad > float(max_depth_mad):
            raise ValueError("SGBM depth MAD %.3fm exceeds %.3fm" %
                             (depth_mad, max_depth_mad))
        if depth_mad > 1e-6:
            sigma = 1.4826 * depth_mad
            values = values[np.abs(values - depth_median) <= float(mad_scale) * sigma]
            if len(values) < self.min_valid_pixels:
                raise ValueError("SGBM robust target pixels %d < %d" %
                                 (len(values), self.min_valid_pixels))
            depth_median = float(np.median(values))
        return {
            "depth_m": depth_median,
            "depth_mad_m": depth_mad,
            "baseline_m": abs(float(baseline)),
            "dense_valid_pixels": int(len(values)),
            "median_disparity_px": float(left_p[0, 0] * abs(float(baseline)) / depth_median),
        }


class SemanticRawStereoNode(object):
    def __init__(self):
        gp = rospy.get_param
        self.execution_enabled = bool(gp("~execution_enabled", False))
        self.publish_planner_goal = bool(gp("~publish_planner_goal", False))
        self.auto_publish_stable_goal = bool(gp("~auto_publish_stable_goal", False))
        self.planner_goal_topic = gp("~planner_goal_topic", "/goal")
        self.world_frame = gp("~world_frame", "world")
        self.require_world_frame = bool(gp("~require_world_frame", True))
        self.target_class = gp("~target_class", "person")
        self.classes = gp("~classes", [self.target_class])
        self.confidence = float(gp("~confidence", 0.20))
        self.device = gp("~device", 0)
        self.debug_bbox = gp("~debug_bbox", [])
        self.depth_backend = gp("~depth_backend", "sgbm").lower()
        if self.depth_backend not in ("sgbm", "orb", "lightglue"):
            raise ValueError("~depth_backend must be sgbm, orb or lightglue")
        self.min_matches = int(gp("~min_matches", 8))
        self.max_sensor_skew = float(gp("~max_sensor_skew", 0.10))
        self.max_rate = float(gp("~max_inference_rate", 2.0))
        self.min_depth = float(gp("~min_depth", 0.35))
        self.max_depth = float(gp("~max_depth", 6.0))
        self.min_disparity = float(gp("~min_disparity_px", 1.0))
        self.max_epipolar_error = float(gp("~max_epipolar_error_px", 1.5))
        self.depth_mad_scale = float(gp("~depth_mad_scale", 3.5))
        self.max_depth_mad = float(gp("~max_depth_mad", 0.50))
        self.sparse_depth_cluster_gap = float(gp("~sparse_depth_cluster_gap", 0.25))
        self.standoff = float(gp("~standoff", 1.0))
        self.keep_body_altitude = bool(gp("~keep_body_altitude", True))
        self.min_stable_observations = max(1, int(gp("~min_stable_observations", 4)))
        self.max_target_jitter = float(gp("~max_target_jitter", 0.35))
        self.stable_timeout = float(gp("~stable_timeout", 1.5))
        self.calibration_source = gp("~calibration_source", "unspecified")

        transform = np.asarray(gp("~body_T_left_camera"), dtype=np.float64).reshape(4, 4)
        if (not np.isfinite(transform).all() or
                not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)):
            raise ValueError("~body_T_left_camera must be a finite homogeneous transform")
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=2e-3):
            raise ValueError("~body_T_left_camera rotation must be orthonormal")
        self.body_t_left_camera = transform

        self.stereo_matcher = OrbStereoMatcher(
            gp("~max_features", 2500), gp("~match_ratio", 0.75),
            gp("~bbox_mask_shrink", 0.08))
        self.lightglue_matcher = None
        if self.depth_backend == "lightglue":
            self.lightglue_matcher = DiskLightGlueStereoMatcher(
                gp("~lightglue_device", self.device),
                gp("~lightglue_max_features", 1024),
                gp("~lightglue_confidence", 0.10),
                gp("~bbox_mask_shrink", 0.08),
                gp("~lightglue_lk_refine", False),
                gp("~lightglue_lk_window", 15),
                gp("~lightglue_lk_max_fb_error", 0.75))
        self.sgbm = SgbmStereoDepth(
            gp("~sgbm_num_disparities", 64), gp("~sgbm_block_size", 5),
            gp("~sgbm_uniqueness_ratio", 8), gp("~sgbm_speckle_window_size", 60),
            gp("~sgbm_speckle_range", 2), gp("~bbox_depth_shrink", 0.25),
            gp("~min_dense_pixels", 150))
        weights = os.path.expanduser(gp("~weights", "~/models/yolo_world/yolov8s-worldv2.pt"))
        self.model_lock = threading.Lock()
        self.model = YOLOWorld(weights)
        self.model.set_classes(self.classes)

        self.left_info = None
        self.right_info = None
        self.lock = threading.Lock()
        self.busy = False
        self.last_inference = rospy.Time(0)
        self.target_history = deque(maxlen=self.min_stable_observations)
        self.last_target_observation = rospy.Time(0)
        self.stable_candidate = None
        self.stable_candidate_stamp = rospy.Time(0)
        self.last_auto_goal = None

        self.camera_pub = rospy.Publisher("~target_left_camera", PointStamped, queue_size=1)
        self.body_pub = rospy.Publisher("~target_body", PointStamped, queue_size=1)
        self.world_pub = rospy.Publisher("~target_world", PointStamped, queue_size=1)
        self.stable_world_pub = rospy.Publisher(
            "~stable_target_world", PointStamped, queue_size=1
        )
        self.candidate_pub = rospy.Publisher("~goal_candidate", PoseStamped, queue_size=1)
        self.stable_pub = rospy.Publisher("~stable_goal_candidate", PoseStamped, queue_size=1)
        self.json_pub = rospy.Publisher("~estimate_json", String, queue_size=1)
        self.annotated_pub = rospy.Publisher("~annotated_left", Image, queue_size=1)
        self.target_class_status_pub = rospy.Publisher(
            "~target_class_status", String, queue_size=1, latch=True
        )
        self.goal_pub = None
        if self.execution_enabled and self.publish_planner_goal:
            self.goal_pub = rospy.Publisher(self.planner_goal_topic, PoseStamped, queue_size=1)
        self.send_goal_service = rospy.Service("~send_goal", Trigger, self.on_send_goal)

        rospy.Subscriber(gp("~left_info_topic"), CameraInfo, self.on_left_info, queue_size=1)
        rospy.Subscriber(gp("~right_info_topic"), CameraInfo, self.on_right_info, queue_size=1)
        rospy.Subscriber(
            gp("~target_class_command_topic", "~target_class_command"),
            String,
            self.on_target_class_command,
            queue_size=1,
        )
        left_sub = message_filters.Subscriber(gp("~left_topic"), Image)
        right_sub = message_filters.Subscriber(gp("~right_topic"), Image)
        odom_sub = message_filters.Subscriber(gp("~odom_topic"), Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub, odom_sub], queue_size=int(gp("~sync_queue", 20)),
            slop=float(gp("~sync_slop", 0.05)), allow_headerless=False)
        self.sync.registerCallback(self.on_sync)
        rospy.loginfo("raw stereo semantic localizer ready: no depth topic, class=%s, gate=%s",
                      self.target_class, bool(self.goal_pub))

    def on_left_info(self, message):
        self.left_info = message

    def on_right_info(self, message):
        self.right_info = message

    def on_target_class_command(self, message):
        target_class = message.data.strip().lower()
        if re.fullmatch(r"[a-z][a-z-]{0,31}", target_class) is None:
            rospy.logwarn("rejected target class; expected one English word")
            return
        with self.model_lock:
            self.model.set_classes([target_class])
            self.target_class = target_class
            self.classes = [target_class]
            with self.lock:
                self.target_history.clear()
                self.last_target_observation = rospy.Time(0)
                self.stable_candidate = None
                self.stable_candidate_stamp = rospy.Time(0)
                self.last_auto_goal = None
        self.target_class_status_pub.publish(String(data=target_class))
        rospy.loginfo("YOLO-World target class changed to %s; stability history reset", target_class)

    def on_send_goal(self, _request):
        if self.goal_pub is None:
            return TriggerResponse(False, "execution gates are closed")
        with self.lock:
            candidate = deepcopy(self.stable_candidate)
            stamp = self.stable_candidate_stamp
        if candidate is None:
            return TriggerResponse(False, "no stable raw-stereo goal is available")
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.0 or age > self.stable_timeout:
            return TriggerResponse(False, "stable raw-stereo goal is stale (age=%.3fs)" % age)
        candidate.header.stamp = rospy.Time.now()
        self.goal_pub.publish(candidate)
        return TriggerResponse(True, "published one raw-stereo goal to %s" % self.planner_goal_topic)

    def on_sync(self, left_message, right_message, odom_message):
        if self.left_info is None or self.right_info is None:
            return
        now = rospy.Time.now()
        with self.lock:
            if self.busy or (now - self.last_inference).to_sec() < 1.0 / max(self.max_rate, 0.1):
                return
            self.busy = True
            self.last_inference = now
        try:
            with self.model_lock:
                self.process(left_message, right_message, odom_message)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "raw stereo observation rejected: %s", exc)
        finally:
            with self.lock:
                self.busy = False

    def select_detection(self, image):
        if isinstance(self.debug_bbox, (list, tuple)) and len(self.debug_bbox) == 4:
            return 1.0, [float(value) for value in self.debug_bbox], "debug_bbox"
        result = self.model.predict(image, conf=self.confidence, device=self.device, verbose=False)[0]
        candidates = []
        for box, score, class_index in zip(result.boxes.xyxy.cpu().tolist(),
                                            result.boxes.conf.cpu().tolist(),
                                            result.boxes.cls.cpu().tolist()):
            label = result.names[int(class_index)]
            if label == self.target_class:
                candidates.append((float(score), box))
        if not candidates:
            return None
        score, box = max(candidates, key=lambda item: item[0])
        return score, [float(value) for value in box], "yolo_world_infra1"

    def process(self, left_message, right_message, odom_message):
        stamps = [left_message.header.stamp.to_sec(), right_message.header.stamp.to_sec(),
                  odom_message.header.stamp.to_sec()]
        sensor_skew = max(stamps) - min(stamps)
        if sensor_skew > self.max_sensor_skew:
            raise ValueError("sensor timestamp skew %.3fs exceeds %.3fs" %
                             (sensor_skew, self.max_sensor_skew))
        if self.require_world_frame and odom_message.header.frame_id != self.world_frame:
            raise ValueError("odometry frame %s does not match %s" %
                             (odom_message.header.frame_id, self.world_frame))

        left_raw = image_to_numpy(left_message)
        right_raw = image_to_numpy(right_message)
        if left_raw.shape[:2] != right_raw.shape[:2]:
            raise ValueError("left and right stereo image dimensions differ")
        if (self.left_info.width != left_message.width or
                self.left_info.height != left_message.height or
                self.right_info.width != right_message.width or
                self.right_info.height != right_message.height):
            raise ValueError("CameraInfo dimensions do not match the stereo images")
        validate_projection_matrices(self.left_info.P, self.right_info.P)
        left_bgr = cv2.cvtColor(left_raw, cv2.COLOR_GRAY2BGR) if left_raw.ndim == 2 else left_raw
        selected = self.select_detection(left_bgr)
        if selected is None:
            rospy.loginfo_throttle(5.0, "YOLO-World found no %s in left infrared image",
                                   self.target_class)
            return
        confidence, bbox, source = selected
        descriptor_matches = None
        cluster_matches = None
        inliers = None
        if self.depth_backend == "sgbm":
            result = self.sgbm.estimate(
                left_raw, right_raw, bbox, self.left_info.P, self.right_info.P,
                self.min_disparity, self.min_depth, self.max_depth,
                self.depth_mad_scale, self.max_depth_mad)
        else:
            matcher = (self.lightglue_matcher if self.depth_backend == "lightglue"
                       else self.stereo_matcher)
            points_left, points_right = matcher.match(left_raw, right_raw, bbox)
            if len(points_left) < self.min_matches:
                raise ValueError("stereo descriptor matches %d < %d" %
                                 (len(points_left), self.min_matches))
            descriptor_matches = len(points_left)
            cluster_indices = select_bbox_feature_cluster(
                points_left, bbox, min_points=self.min_matches)
            points_left = points_left[cluster_indices]
            points_right = points_right[cluster_indices]
            cluster_matches = len(points_left)
            result = triangulate_rectified(
                points_left, points_right, self.left_info.P, self.right_info.P,
                min_disparity_px=self.min_disparity,
                max_epipolar_error_px=self.max_epipolar_error,
                min_depth=self.min_depth, max_depth=self.max_depth,
                mad_scale=self.depth_mad_scale, max_depth_mad=self.max_depth_mad,
                prefer_near_cluster=True, min_cluster_points=self.min_matches,
                depth_cluster_gap=self.sparse_depth_cluster_gap)
            inliers = len(result["points_left_camera"])
            if inliers < self.min_matches:
                raise ValueError("triangulated stereo inliers %d < %d" %
                                 (inliers, self.min_matches))
        point_camera = bbox_center_point(bbox, result["depth_m"], self.left_info.P)

        pose = odom_message.pose.pose
        body_position = np.asarray([pose.position.x, pose.position.y, pose.position.z])
        body_quaternion = [pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w]
        point_body = camera_to_body(point_camera, self.body_t_left_camera)
        point_world = body_to_world(point_body, body_position, body_quaternion)

        target_camera_message = PointStamped()
        target_camera_message.header = left_message.header
        target_camera_message.point.x, target_camera_message.point.y, target_camera_message.point.z = point_camera
        target_body_message = PointStamped()
        target_body_message.header.stamp = left_message.header.stamp
        target_body_message.header.frame_id = odom_message.child_frame_id or "body"
        target_body_message.point.x, target_body_message.point.y, target_body_message.point.z = point_body
        target_world_message = PointStamped()
        target_world_message.header.stamp = left_message.header.stamp
        target_world_message.header.frame_id = odom_message.header.frame_id
        target_world_message.point.x, target_world_message.point.y, target_world_message.point.z = point_world
        self.camera_pub.publish(target_camera_message)
        self.body_pub.publish(target_body_message)
        self.world_pub.publish(target_world_message)

        goal = None
        candidate = None
        try:
            goal = standoff_goal(body_position, point_world, self.standoff,
                                 keep_body_altitude=self.keep_body_altitude)
            candidate = PoseStamped()
            candidate.header = target_world_message.header
            candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z = goal
            candidate.pose.orientation.w = 1.0
            self.candidate_pub.publish(candidate)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "raw stereo target has no safe goal candidate: %s", exc)

        now = rospy.Time.now()
        if (not self.last_target_observation.is_zero() and
                (now - self.last_target_observation).to_sec() > self.stable_timeout):
            self.target_history.clear()
            self.last_auto_goal = None
        self.last_target_observation = now
        with self.lock:
            self.stable_candidate = None
            self.stable_candidate_stamp = rospy.Time(0)
        self.target_history.append(point_world.copy())
        jitter = None
        stable = False
        if candidate is not None and len(self.target_history) == self.min_stable_observations:
            samples = np.asarray(self.target_history)
            stable_target = np.median(samples, axis=0)
            jitter = float(np.max(np.linalg.norm(samples - stable_target, axis=1)))
            if jitter <= self.max_target_jitter:
                stable_goal = standoff_goal(body_position, stable_target, self.standoff,
                                             keep_body_altitude=self.keep_body_altitude)
                stable_candidate = deepcopy(candidate)
                (stable_candidate.pose.position.x, stable_candidate.pose.position.y,
                 stable_candidate.pose.position.z) = stable_goal
                self.stable_pub.publish(stable_candidate)
                stable_target_message = PointStamped()
                stable_target_message.header = target_world_message.header
                (stable_target_message.point.x, stable_target_message.point.y,
                 stable_target_message.point.z) = stable_target
                self.stable_world_pub.publish(stable_target_message)
                with self.lock:
                    self.stable_candidate = deepcopy(stable_candidate)
                    self.stable_candidate_stamp = now
                stable = True
                if self.goal_pub is not None and self.auto_publish_stable_goal:
                    if (self.last_auto_goal is None or
                            np.linalg.norm(stable_goal - self.last_auto_goal) > self.max_target_jitter):
                        self.goal_pub.publish(stable_candidate)
                        self.last_auto_goal = stable_goal.copy()

        payload = {
            "class": self.target_class, "confidence": confidence, "source": source,
            "bbox_left_xyxy": bbox,
            "depth_source": "raw_infra_stereo_%s" % self.depth_backend,
            "uses_realsense_depth_topic": False, "depth_m": result["depth_m"],
            "depth_mad_m": result["depth_mad_m"], "baseline_m": result["baseline_m"],
            "descriptor_matches": descriptor_matches,
            "target_cluster_matches": cluster_matches, "triangulated_inliers": inliers,
            "depth_cluster_size": result.get("depth_cluster_size"),
            "dense_valid_pixels": result.get("dense_valid_pixels"),
            "median_disparity_px": result.get("median_disparity_px"),
            "median_epipolar_error_px": result.get("median_epipolar_error_px"),
            "sensor_skew_s": sensor_skew, "target_left_camera": point_camera.tolist(),
            "target_body": point_body.tolist(), "target_world": point_world.tolist(),
            "goal_candidate": None if goal is None else goal.tolist(),
            "stable_observations": len(self.target_history),
            "target_jitter_m": jitter, "stable_goal_available": stable,
            "execution_gate_open": bool(self.goal_pub),
            "planner_goal_topic": self.planner_goal_topic,
            "calibration_source": self.calibration_source,
        }
        self.json_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        annotated = left_bgr.copy()
        x1, y1, x2, y2 = [int(round(value)) for value in bbox]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 255, 0), 2)
        quality_count = result.get("dense_valid_pixels", inliers)
        cv2.putText(annotated, "%s %.2f raw-%s z=%.2fm n=%d" %
                    (self.target_class, confidence, self.depth_backend,
                     result["depth_m"], quality_count),
                    (max(0, x1), max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 0), 2, cv2.LINE_AA)
        self.annotated_pub.publish(numpy_to_bgr8(annotated, left_message.header))
        rospy.loginfo("raw stereo target %s backend=%s depth=%.3f samples=%d skew=%.3f stable=%s",
                      self.target_class, self.depth_backend, result["depth_m"],
                      quality_count, sensor_skew, stable)


if __name__ == "__main__":
    rospy.init_node("semantic_raw_stereo_node")
    SemanticRawStereoNode()
    rospy.spin()
