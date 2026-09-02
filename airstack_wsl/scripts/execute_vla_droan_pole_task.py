#!/usr/bin/env python3
"""Execute the simulation-only VLA/DROAN utility-pole benchmark safely."""

import argparse
import copy
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from airstack_msgs.msg import Odometry as AirStackOdometry
from airstack_msgs.msg import TrajectoryXYZVYaw
from airstack_msgs.srv import RobotCommand, TakeoffLandingCommand, TrajectoryMode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from validate_flight_execution import FlightValidator, NS


class PoleTaskExecutor(FlightValidator):
    def __init__(self) -> None:
        super().__init__()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_optimized = None
        self.latest_optimized_map = None
        self.latest_optimized_at = 0.0
        self.look_ahead_state = None
        self.debug_counts = {}
        self.path_pub = self.create_publisher(NavPath, f"{NS}/vla/global_plan", 10)
        self.segment_pub = self.create_publisher(
            TrajectoryXYZVYaw,
            f"{NS}/trajectory_controller/trajectory_segment_to_add",
            1,
        )
        self.create_subscription(
            TrajectoryXYZVYaw,
            f"{NS}/vla/optimized_trajectory",
            self.optimized_cb,
            10,
        )
        self.create_subscription(
            AirStackOdometry,
            f"{NS}/trajectory_controller/look_ahead",
            lambda message: setattr(self, "look_ahead_state", message),
            10,
        )
        self.create_subscription(
            MarkerArray, f"{NS}/droan/traj_debug", self.debug_cb, 10
        )

        self.direction = np.asarray([0.9124236720043529, 0.40924692151083203])
        self.direction /= np.linalg.norm(self.direction)
        self.pole_xy = np.asarray([3.647, 1.637], dtype=np.float64)
        self.plan_start = None
        self.plan_end = None
        self.altitude = 1.2
        self.min_clearance = 0.83
        self.min_actual_clearance = math.inf
        self.forwarded_segments = 0
        self.rejected_segments = 0
        self.last_rejection = "none"
        self.last_gate_metrics = None

    def landed(self) -> bool:
        """Return true at the Pegasus vehicle's simulated ground-rest height.

        The generic validator uses 0.14 m, but this Pegasus rigid body settles
        with its odometry origin roughly 0.18 m above the ground plane.  Keep
        this model-specific tolerance local to the pole benchmark so a normal
        touchdown cannot be misreported as a landing timeout.
        """
        return self.odom is not None and self.odom.pose.pose.position.z <= 0.25

    def debug_cb(self, message: MarkerArray) -> None:
        counts = {}
        for marker in message.markers:
            counts[marker.ns] = counts.get(marker.ns, 0) + len(marker.points)
        self.debug_counts = counts

    def optimized_cb(self, message: TrajectoryXYZVYaw) -> None:
        if len(message.waypoints) < 2:
            return
        local = np.asarray(
            [[p.position.x, p.position.y, p.position.z] for p in message.waypoints],
            dtype=np.float64,
        )
        mapped = self.points_in_map(local, message.header.frame_id)
        if mapped is None:
            return
        self.latest_optimized = message
        self.latest_optimized_map = mapped
        self.latest_optimized_at = time.monotonic()

    def points_in_map(self, points: np.ndarray, frame_id: str):
        if frame_id == "map":
            return points
        try:
            transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
        except TransformException:
            return None
        q = transform.transform.rotation
        q_vector = np.asarray([q.x, q.y, q.z], dtype=np.float64)
        cross = 2.0 * np.cross(np.broadcast_to(q_vector, points.shape), points)
        rotated = points + q.w * cross + np.cross(
            np.broadcast_to(q_vector, points.shape), cross
        )
        t = transform.transform.translation
        return rotated + np.asarray([t.x, t.y, t.z], dtype=np.float64)

    @staticmethod
    def polyline_distance(points: np.ndarray, target: np.ndarray) -> float:
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

    def publish_global_path(self) -> None:
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        path_start = self.plan_start
        if self.odom is not None:
            pose = self.odom.pose.pose.position
            current = np.asarray([pose.x, pose.y], dtype=np.float64)
            current_along = float((current - self.plan_start) @ self.direction)
            pole_along = float((self.pole_xy - self.plan_start) @ self.direction)
            # Once the vehicle is beyond the pole, never republish the stale
            # pre-obstacle portion of the global path. Reintroducing those
            # points makes GlobalPlan::set_global_plan undo its own trim and
            # can cause a safe planner to propose a backwards return.
            if current_along > pole_along + 0.50:
                path_start = current
        for fraction in np.linspace(0.0, 1.0, 25):
            xy = path_start + fraction * (self.plan_end - path_start)
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(xy[0])
            pose.pose.position.y = float(xy[1])
            pose.pose.position.z = self.altitude
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def gate_candidate(self):
        message = self.latest_optimized
        points = self.latest_optimized_map
        if message is None or points is None:
            return None, "no_candidate", None
        if time.monotonic() - self.latest_optimized_at > 1.5:
            return None, "stale_candidate", None
        if not np.all(np.isfinite(points)):
            return None, "non_finite", None

        pose = self.odom.pose.pose.position
        current = np.asarray([pose.x, pose.y], dtype=np.float64)
        continuity_pose = (
            self.look_ahead_state.pose.position
            if self.look_ahead_state is not None
            else pose
        )
        continuity_point = np.asarray(
            [continuity_pose.x, continuity_pose.y], dtype=np.float64
        )
        relative = points[:, :2] - self.plan_start
        cross_track = np.abs(
            -relative[:, 0] * self.direction[1]
            + relative[:, 1] * self.direction[0]
        )
        max_cross_track = float(np.max(cross_track))
        vertical_error = float(np.max(np.abs(points[:, 2] - self.altitude)))
        pole_clearance = self.polyline_distance(points[:, :2], self.pole_xy)
        start_distance = float(np.linalg.norm(points[0, :2] - continuity_point))
        progress = float((points[-1, :2] - continuity_point) @ self.direction)
        current_along = float((current - self.plan_start) @ self.direction)
        pole_along = float((self.pole_xy - self.plan_start) @ self.direction)
        collision_points = int(self.debug_counts.get("collision_points", 0))
        metrics = {
            "max_cross_track_m": max_cross_track,
            "max_altitude_error_m": vertical_error,
            "minimum_pole_center_clearance_m": pole_clearance,
            "candidate_start_distance_m": start_distance,
            "forward_progress_m": progress,
            "collision_points": collision_points,
        }

        if vertical_error > 0.25:
            return None, "altitude_gate", metrics
        if pole_clearance < self.min_clearance:
            return None, "clearance_gate", metrics
        if start_distance > 0.75:
            return None, "continuity_gate", metrics
        if progress < 0.60:
            return None, "progress_gate", metrics
        if current_along < pole_along + 0.80:
            if collision_points <= 0:
                return None, "collision_evidence_gate", metrics
            if max_cross_track < 0.65:
                return None, "detour_gate", metrics

        safe_message = copy.deepcopy(message)
        for waypoint in safe_message.waypoints:
            waypoint.velocity = min(max(float(waypoint.velocity), 0.20), 0.45)
        return safe_message, "accepted", metrics

    def prepare_and_takeoff(self) -> None:
        self.wait_for(
            lambda: self.state is not None and self.state.connected and self.odom is not None,
            20.0,
            "MAVLink and odometry",
        )
        if self.state.armed:
            raise RuntimeError("refusing to start: vehicle is already armed")
        self.wait_for(
            lambda: self.tracking_count >= 10 and self.command_count >= 10,
            10.0,
            "AirStack control stream",
        )
        self.set_trajectory_mode(TrajectoryMode.Request.ROBOT_POSE)
        self.reset_integrators()
        baseline = self.command_count
        self.wait_for(
            lambda: self.command_count >= baseline + 30
            and self.last_command is not None
            and 0.35 <= self.last_command.thrust.z <= 0.75,
            5.0,
            "pre-OFFBOARD hover stream",
        )
        self.robot_command(RobotCommand.Request.REQUEST_CONTROL)
        self.wait_for(lambda: self.state.mode == "OFFBOARD", 8.0, "OFFBOARD")
        self.robot_command(RobotCommand.Request.ARM)
        self.wait_for(lambda: self.state.armed, 8.0, "arming")
        self.reset_integrators()
        self.set_trajectory_mode(TrajectoryMode.Request.TRACK)
        self.takeoff_landing(TakeoffLandingCommand.Request.TAKEOFF)
        self.wait_for(
            lambda: self.odom.pose.pose.position.z >= 1.0,
            30.0,
            "1.0 m takeoff altitude",
        )
        settle_until = time.monotonic() + 2.0
        self.wait_for(lambda: time.monotonic() >= settle_until, 3.0, "hover settling")

    def land_and_disarm(self) -> None:
        self.set_trajectory_mode(TrajectoryMode.Request.TRACK)
        self.takeoff_landing(TakeoffLandingCommand.Request.LAND)
        self.wait_for(self.landed, 35.0, "landing")
        self.px4_mode("AUTO.LAND")
        try:
            self.wait_for(lambda: not self.state.armed, 12.0, "auto-disarm")
        except TimeoutError:
            self.robot_command(RobotCommand.Request.DISARM)
            self.wait_for(lambda: not self.state.armed, 8.0, "disarm")
        self.px4_mode("AUTO.LOITER")
        self.wait_for(lambda: self.state.mode == "AUTO.LOITER", 8.0, "AUTO.LOITER")
        self.set_trajectory_mode(TrajectoryMode.Request.ROBOT_POSE)
        self.reset_integrators()

    def execute(self) -> dict:
        self.prepare_and_takeoff()
        position = self.odom.pose.pose.position
        self.plan_start = np.asarray([position.x, position.y], dtype=np.float64)
        self.plan_end = self.plan_start + 6.0 * self.direction
        self.altitude = float(position.z)
        # Discard any candidate generated from a previous validation path or
        # from the ascent phase. Only segments produced after this fixed task
        # corridor is published are eligible for execution.
        self.latest_optimized = None
        self.latest_optimized_map = None
        self.latest_optimized_at = 0.0

        accepted = None
        accepted_metrics = None
        last_evaluated_at = 0.0
        path_publish_count = 0
        deadline = time.monotonic() + 18.0
        while time.monotonic() < deadline and accepted is None:
            self.publish_global_path()
            path_publish_count += 1
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                path_publish_count >= 3
                and self.latest_optimized_at > last_evaluated_at
            ):
                accepted, reason, metrics = self.gate_candidate()
                last_evaluated_at = self.latest_optimized_at
                self.last_gate_metrics = metrics
                if accepted is None:
                    self.last_rejection = reason
                    self.rejected_segments += 1
                    print(
                        "POLE_TASK_GATE_REJECT "
                        + json.dumps({"reason": reason, "metrics": metrics}, sort_keys=True),
                        flush=True,
                    )
                else:
                    accepted_metrics = metrics
        if accepted is None:
            raise RuntimeError(f"no safe initial DROAN segment: {self.last_rejection}")

        self.set_trajectory_mode(TrajectoryMode.Request.ADD_SEGMENT)
        self.segment_pub.publish(accepted)
        self.forwarded_segments += 1
        last_forward = time.monotonic()
        last_candidate_stamp = self.latest_optimized_at
        deadline = time.monotonic() + 55.0
        completed = False
        while time.monotonic() < deadline:
            self.publish_global_path()
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self.odom.pose.pose.position
            current = np.asarray([pose.x, pose.y], dtype=np.float64)
            actual_clearance = float(np.linalg.norm(current - self.pole_xy))
            self.min_actual_clearance = min(self.min_actual_clearance, actual_clearance)
            if actual_clearance < self.min_clearance:
                raise RuntimeError(
                    f"actual pole clearance violated: {actual_clearance:.3f} m"
                )
            if not 0.75 <= pose.z <= 1.55:
                raise RuntimeError(f"altitude safety gate violated: {pose.z:.3f} m")

            along = float((current - self.plan_start) @ self.direction)
            endpoint_distance = float(np.linalg.norm(current - self.plan_end))
            if along >= 5.0 and endpoint_distance <= 1.4:
                completed = True
                break

            if self.latest_optimized_at > last_candidate_stamp:
                candidate, reason, metrics = self.gate_candidate()
                last_candidate_stamp = self.latest_optimized_at
                if candidate is not None:
                    self.segment_pub.publish(candidate)
                    self.forwarded_segments += 1
                    last_forward = time.monotonic()
                    accepted_metrics = metrics
                    self.last_gate_metrics = metrics
                    self.last_rejection = "none"
                else:
                    self.rejected_segments += 1
                    self.last_rejection = reason
                    self.last_gate_metrics = metrics
            # A clamped 0.45 m/s, 4.2 m safe segment can legitimately take
            # roughly nine seconds to execute. Continue monitoring the actual
            # vehicle while that already-vetted segment runs; only abort when
            # both replanning and the safe-segment execution window expire.
            if time.monotonic() - last_forward > 12.0:
                raise RuntimeError(f"safe replanning timeout: {self.last_rejection}")

        if not completed:
            raise TimeoutError("pole task did not reach the post-obstacle goal")

        final_position = self.odom.pose.pose.position
        self.land_and_disarm()
        return {
            "status": "pass",
            "forwarded_segments": self.forwarded_segments,
            "rejected_segments": self.rejected_segments,
            "minimum_actual_pole_center_clearance_m": self.min_actual_clearance,
            "final_flight_xy": [final_position.x, final_position.y],
            "final_z": self.odom.pose.pose.position.z,
            "armed_final": self.state.armed,
            "mode_final": self.state.mode,
            "last_accepted_segment": accepted_metrics,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    executor = PoleTaskExecutor()
    result = None
    try:
        result = executor.execute()
        print("VLA_DROAN_POLE_EXECUTION_PASS " + json.dumps(result, sort_keys=True))
        return 0
    except Exception as exc:
        result = {
            "status": "fail",
            "error": f"{type(exc).__name__}: {exc}",
            "forwarded_segments": executor.forwarded_segments,
            "rejected_segments": executor.rejected_segments,
            "minimum_actual_pole_center_clearance_m": (
                executor.min_actual_clearance
                if math.isfinite(executor.min_actual_clearance)
                else None
            ),
            "last_rejection": executor.last_rejection,
            "last_gate_metrics": executor.last_gate_metrics,
        }
        print("VLA_DROAN_POLE_EXECUTION_FAIL " + json.dumps(result, sort_keys=True))
        return 1
    finally:
        if result is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        executor.safe_recover()
        executor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
