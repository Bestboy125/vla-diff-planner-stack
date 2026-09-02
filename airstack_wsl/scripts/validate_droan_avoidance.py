#!/usr/bin/env python3
"""Publish a collision-course global plan and verify DROAN changes its shape."""

import json
import math
import os
import time

import numpy as np
import rclpy
from airstack_msgs.msg import Odometry as AirStackOdometry
from airstack_msgs.msg import TrajectoryXYZVYaw
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray


class DroanValidator(Node):
    def __init__(self) -> None:
        super().__init__("droan_obstacle_avoidance_validator")
        self.odom = None
        self.look_ahead = None
        self.depth_stats = None
        self.disparity_frames = 0
        self.expanded_frames = 0
        self.trajectories = []
        self.trajectory_debug = {}
        self.path_publish_count = 0
        self.accept_trajectories_after = time.monotonic() + 2.0
        self.plan_altitude = float(os.environ.get("DROAN_PLAN_ALTITUDE", "nan"))
        requested_dx = float(os.environ.get("DROAN_PLAN_DX", "1.0"))
        requested_dy = float(os.environ.get("DROAN_PLAN_DY", "0.0"))
        direction_norm = math.hypot(requested_dx, requested_dy)
        if direction_norm < 1e-9:
            raise ValueError("DROAN_PLAN_DX/DY must define a non-zero direction")
        self.plan_direction = (
            requested_dx / direction_norm,
            requested_dy / direction_norm,
        )
        self.plan_length = float(os.environ.get("DROAN_PLAN_LENGTH", "3.5"))
        self.min_cross_track = float(
            os.environ.get("DROAN_MIN_CROSS_TRACK", "0.65")
        )
        self.max_vertical_deviation = float(
            os.environ.get("DROAN_MAX_VERTICAL_DEVIATION", "0.40")
        )
        self.min_pole_clearance = float(
            os.environ.get("DROAN_MIN_POLE_CLEARANCE", "0.83")
        )
        self.pole_xy = np.asarray(
            [
                float(os.environ.get("DROAN_POLE_X", "3.647")),
                float(os.environ.get("DROAN_POLE_Y", "1.637")),
            ],
            dtype=np.float64,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Path, "/robot_1/vla/global_plan", 10)
        self.create_subscription(
            Odometry, "/robot_1/odometry_conversion/odometry", self.odom_cb, 10
        )
        self.create_subscription(
            AirStackOdometry,
            "/robot_1/trajectory_controller/look_ahead",
            self.look_ahead_cb,
            10,
        )
        self.create_subscription(
            Image, "/robot_1/sensors/front_camera/image/depth", self.depth_cb, 10
        )
        self.create_subscription(
            DisparityImage,
            "/robot_1/sensors/front_camera/disparity",
            lambda _msg: setattr(self, "disparity_frames", self.disparity_frames + 1),
            10,
        )
        self.create_subscription(
            Image,
            "/robot_1/droan/foreground_expanded",
            lambda _msg: setattr(self, "expanded_frames", self.expanded_frames + 1),
            10,
        )
        self.create_subscription(
            TrajectoryXYZVYaw,
            "/robot_1/vla/optimized_trajectory",
            self.trajectory_cb,
            10,
        )
        self.create_subscription(
            MarkerArray,
            "/robot_1/droan/traj_debug",
            self.trajectory_debug_cb,
            10,
        )
        self.timer = self.create_timer(0.5, self.publish_path)

    def odom_cb(self, message: Odometry) -> None:
        self.odom = message

    def look_ahead_cb(self, message: AirStackOdometry) -> None:
        self.look_ahead = message

    def depth_cb(self, message: Image) -> None:
        if message.encoding not in ("32FC1", "32FC"):
            return
        depth = np.ndarray(
            (message.height, message.width),
            dtype=np.dtype("<f4" if not message.is_bigendian else ">f4"),
            buffer=message.data,
            strides=(message.step, 4),
        )
        valid = depth[np.isfinite(depth) & (depth > 0.05)]
        if valid.size:
            h, w = depth.shape
            center = depth[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
            center = center[np.isfinite(center) & (center > 0.05)]
            self.depth_stats = {
                "minimum_m": float(valid.min()),
                "median_m": float(np.median(valid)),
                "center_minimum_m": float(center.min()) if center.size else None,
            }

    def publish_path(self) -> None:
        if self.look_ahead is not None:
            position = self.look_ahead.pose.position
        elif self.odom is not None:
            position = self.odom.pose.pose.position
        else:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for offset in np.linspace(0.0, self.plan_length, 15):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(position.x + offset * self.plan_direction[0])
            pose.pose.position.y = float(position.y + offset * self.plan_direction[1])
            pose.pose.position.z = float(
                self.plan_altitude
                if math.isfinite(self.plan_altitude)
                else position.z
            )
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.path_publish_count += 1

    def trajectory_cb(self, message: TrajectoryXYZVYaw) -> None:
        if (
            len(message.waypoints) < 2
            or self.path_publish_count < 3
            or time.monotonic() < self.accept_trajectories_after
        ):
            return
        local_points = np.asarray(
            [[p.position.x, p.position.y, p.position.z] for p in message.waypoints],
            dtype=np.float64,
        )
        points = self.points_in_map(local_points, message.header.frame_id)
        if points is None:
            return
        lateral_span = float(np.ptp(points[:, 1]))
        relative = points - points[0]
        cross_track = (
            -relative[:, 0] * self.plan_direction[1]
            + relative[:, 1] * self.plan_direction[0]
        )
        cross_track_span = float(np.ptp(cross_track))
        max_cross_track = float(np.max(np.abs(cross_track)))
        vertical_span = float(np.ptp(points[:, 2]))
        max_vertical_deviation = float(
            np.max(np.abs(points[:, 2] - points[0, 2]))
        )
        pole_clearance = self.polyline_distance_xy(points[:, :2], self.pole_xy)
        curve_length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        chord = float(np.linalg.norm(points[-1] - points[0]))
        self.trajectories.append(
            {
                "frame": message.header.frame_id,
                "waypoint_count": len(message.waypoints),
                "first_local": local_points[0].tolist(),
                "last_local": local_points[-1].tolist(),
                "first_map": points[0].tolist(),
                "last_map": points[-1].tolist(),
                "lateral_span_m": lateral_span,
                "cross_track_span_m": cross_track_span,
                "max_cross_track_m": max_cross_track,
                "vertical_span_m": vertical_span,
                "max_vertical_deviation_m": max_vertical_deviation,
                "minimum_pole_center_clearance_m": pole_clearance,
                "curve_length_m": curve_length,
                "chord_m": chord,
                "non_straightness_m": max(0.0, curve_length - chord),
            }
        )

    def points_in_map(self, points: np.ndarray, frame_id: str):
        if frame_id == "map":
            return points
        try:
            transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
        except TransformException as exc:
            self.get_logger().warning(f"trajectory TF unavailable: {exc}")
            return None
        q = transform.transform.rotation
        q_vector = np.asarray([q.x, q.y, q.z], dtype=np.float64)
        cross = 2.0 * np.cross(np.broadcast_to(q_vector, points.shape), points)
        rotated = points + q.w * cross + np.cross(
            np.broadcast_to(q_vector, points.shape), cross
        )
        translation = transform.transform.translation
        return rotated + np.asarray(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )

    @staticmethod
    def polyline_distance_xy(points: np.ndarray, target: np.ndarray) -> float:
        starts = points[:-1]
        segments = points[1:] - starts
        lengths_sq = np.sum(segments * segments, axis=1)
        fractions = np.divide(
            np.sum((target - starts) * segments, axis=1),
            lengths_sq,
            out=np.zeros_like(lengths_sq),
            where=lengths_sq > 1e-12,
        )
        fractions = np.clip(fractions, 0.0, 1.0)
        closest = starts + fractions[:, None] * segments
        return float(np.min(np.linalg.norm(closest - target, axis=1)))

    def trajectory_debug_cb(self, message: MarkerArray) -> None:
        counts = {}
        for marker in message.markers:
            counts[marker.ns] = counts.get(marker.ns, 0) + len(marker.points)
        self.trajectory_debug = counts


def main() -> int:
    rclpy.init()
    node = DroanValidator()
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.trajectories and node.disparity_frames >= 2 and node.expanded_frames >= 1:
            break

    best = max(
        node.trajectories,
        key=lambda item: item["max_cross_track_m"],
        default=None,
    )
    # This benchmark is explicitly a same-altitude "go around the pole" task.
    # A merely non-straight or vertically climbing segment is not sufficient:
    # require a one-sided lateral detour at least as large as DROAN's expansion
    # radius and reject candidates that evade primarily by climbing/diving.
    modified = bool(
        best
        and best["max_cross_track_m"] >= node.min_cross_track
        and best["max_vertical_deviation_m"] <= node.max_vertical_deviation
        and best["minimum_pole_center_clearance_m"] >= node.min_pole_clearance
    )
    obstacle_seen = bool(
        node.depth_stats and node.depth_stats["minimum_m"] < 3.5
    )
    passed = bool(
        node.odom
        and node.look_ahead
        and node.disparity_frames >= 1
        and node.expanded_frames >= 1
        and best
        and obstacle_seen
        and modified
        and node.trajectory_debug.get("collision_points", 0) > 0
    )
    report = {
        "status": "pass" if passed else "fail",
        "nominal_plan": {
            "length_m": node.plan_length,
            "direction_xy": list(node.plan_direction),
            "altitude_m": node.plan_altitude if math.isfinite(node.plan_altitude) else None,
            "lateral_span_m": 0.0,
            "vertical_span_m": 0.0,
        },
        "depth": node.depth_stats,
        "disparity_frames": node.disparity_frames,
        "expanded_frames": node.expanded_frames,
        "optimized_segments": len(node.trajectories),
        "most_modified_segment": best,
        "obstacle_seen": obstacle_seen,
        "trajectory_modified": modified,
        "acceptance": {
            "minimum_cross_track_m": node.min_cross_track,
            "maximum_vertical_deviation_m": node.max_vertical_deviation,
            "minimum_pole_center_clearance_m": node.min_pole_clearance,
            "pole_xy_map": node.pole_xy.tolist(),
        },
        "trajectory_debug_points": node.trajectory_debug,
        "vehicle_commanded": False,
    }
    print(("DROAN_AVOIDANCE_PASS " if passed else "DROAN_AVOIDANCE_FAIL ") + json.dumps(report, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
