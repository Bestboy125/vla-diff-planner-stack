#!/usr/bin/env python3
"""Run simulation-only OpenVLA inference every K safe DROAN segments.

The policy provides a body-frame planar direction.  A bounded local corridor
is rebuilt from the current vehicle pose after every K accepted trajectory
segments; DROAN continuously optimizes that corridor between policy calls.
Only safety-gated segments are forwarded to the PX4 SITL controller.
"""

import argparse
import base64
from io import BytesIO
import json
import math
import os
from pathlib import Path
import subprocess
import time
import urllib.error
import urllib.request

import numpy as np
from PIL import Image as PILImage
import rclpy
from airstack_msgs.srv import TrajectoryMode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from capture_vla_observation import rgb_array
from execute_vla_droan_pole_task import PoleTaskExecutor


STOP_WORDS = ("stop", "hold", "停止", "悬停", "保持")
EXPECTED_SEMANTIC = ["dx_body", "dy_body", "dz_body", "d_yaw"]
EXPECTED_UNITS = ["m", "m", "m", "rad"]


def default_openvla_url() -> str:
    configured = os.environ.get("OPENVLA_URL")
    if configured:
        return configured.rstrip("/")
    try:
        route = subprocess.check_output(
            ["ip", "route", "show", "default"], text=True, timeout=2.0
        ).split()
        gateway = route[route.index("via") + 1]
    except Exception:
        gateway = "127.0.0.1"
    return f"http://{gateway}:5007"


def yaw_degrees(orientation) -> float:
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return math.degrees(yaw)


def direction_change_degrees(previous: np.ndarray, current: np.ndarray) -> float:
    cosine = float(np.clip(previous @ current, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


class ContinuousOpenVLAExecutor(PoleTaskExecutor):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.args = args
        self.openvla_url = (args.openvla_url or default_openvla_url()).rstrip("/")
        self.rgb = None
        self.rgb_received_at = 0.0
        self.rgb_sequence = 0
        self.last_inference_rgb_sequence = 0
        self.inference_trace = []
        self.inference_failures = 0
        self.segments_since_inference = 0
        self.corridor_updated_at = 0.0
        self.corridor_publish_count = 0
        self.mission_start = None
        self.mission_direction = None
        self.mission_goal = None
        self.create_subscription(
            Image,
            "/robot_1/sensors/front_camera/image/rgb",
            self.rgb_cb,
            qos_profile_sensor_data,
        )

    def rgb_cb(self, message: Image) -> None:
        self.rgb = message
        self.rgb_received_at = time.monotonic()
        self.rgb_sequence += 1

    def request_json(self, path: str, payload=None, timeout=None) -> dict:
        data = None
        headers = {}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            f"{self.openvla_url}{path}", data=data, headers=headers
        )
        try:
            with urllib.request.urlopen(
                request, timeout=timeout or self.args.inference_timeout
            ) as response:
                return json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"OpenVLA HTTP request failed for {path}: {type(exc).__name__}: {exc}"
            ) from exc

    def preflight(self) -> dict:
        self.wait_for(
            lambda: self.state is not None
            and self.state.connected
            and self.odom is not None
            and self.rgb is not None,
            20.0,
            "MAVLink, odometry and RGB",
        )
        if self.state.armed:
            raise RuntimeError("refusing to start: vehicle is already armed")
        health = self.request_json("/", timeout=3.0)
        contract = health.get("action_contract", {})
        if health.get("status") != "ok" or contract.get("shape") != [4]:
            raise RuntimeError(f"unexpected OpenVLA health contract: {health}")
        if contract.get("semantic") != EXPECTED_SEMANTIC:
            raise RuntimeError(f"unexpected OpenVLA action semantic: {contract}")
        if contract.get("units") != EXPECTED_UNITS:
            raise RuntimeError(f"unexpected OpenVLA action units: {contract}")
        return health

    def wait_for_fresh_rgb(self) -> None:
        self.wait_for(
            lambda: self.rgb is not None
            and self.rgb_sequence > self.last_inference_rgb_sequence
            and time.monotonic() - self.rgb_received_at <= self.args.max_image_age,
            self.args.fresh_image_timeout,
            "fresh RGB for OpenVLA",
        )

    def infer_and_update_corridor(self) -> dict:
        self.wait_for_fresh_rgb()
        message = self.rgb
        image_sequence = self.rgb_sequence
        image_received_at = self.rgb_received_at
        image = PILImage.fromarray(rgb_array(message))
        encoded = BytesIO()
        image.save(encoded, format="JPEG", quality=90)

        pose = self.odom.pose.pose
        proprio = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            yaw_degrees(pose.orientation),
        ]
        payload = {
            "image": base64.b64encode(encoded.getvalue()).decode("ascii"),
            "instr": self.args.instruction,
            "proprio": proprio,
        }
        started = time.monotonic()
        result = self.request_json("/predict", payload=payload)
        latency_ms = (time.monotonic() - started) * 1000.0
        if result.get("status") != "success":
            raise RuntimeError(result.get("message", "OpenVLA returned an error"))
        if result.get("action_semantic") != EXPECTED_SEMANTIC:
            raise RuntimeError("OpenVLA response action semantic changed")
        if result.get("action_units") != EXPECTED_UNITS:
            raise RuntimeError("OpenVLA response action units changed")

        action = np.asarray(result.get("action_local_delta"), dtype=np.float64)
        if action.shape != (1, 4) or not np.all(np.isfinite(action)):
            raise RuntimeError(f"OpenVLA action must be finite [1,4], got {action}")
        action = action[0]
        if (
            np.max(np.abs(action[:2])) > self.args.max_planar_action_m
            or abs(float(action[2])) > self.args.max_vertical_action_m
            or abs(float(action[3])) > self.args.max_yaw_action_rad
        ):
            raise RuntimeError(f"OpenVLA action exceeds deployment bounds: {action.tolist()}")
        planar_norm = float(np.linalg.norm(action[:2]))
        if planar_norm < self.args.min_direction_norm:
            raise RuntimeError(
                f"OpenVLA planar action is degenerate: norm={planar_norm:.8f}"
            )

        body_direction = action[:2] / planar_norm
        yaw = math.radians(proprio[3])
        rotation = np.asarray(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
            dtype=np.float64,
        )
        map_direction = rotation @ body_direction
        direction_change = None
        if self.direction is not None and self.inference_trace:
            direction_change = direction_change_degrees(self.direction, map_direction)
            if direction_change > self.args.max_direction_change_deg:
                raise RuntimeError(
                    "OpenVLA direction jump exceeds gate: "
                    f"{direction_change:.2f} > {self.args.max_direction_change_deg:.2f} deg"
                )

        current = np.asarray([pose.position.x, pose.position.y], dtype=np.float64)
        self.direction = map_direction
        self.plan_start = current
        self.plan_end = current + self.args.local_horizon_m * map_direction
        self.altitude = float(pose.position.z)
        if self.mission_start is None:
            self.mission_start = current.copy()
            self.mission_direction = map_direction.copy()
            self.mission_goal = (
                self.mission_start
                + self.args.mission_distance_m * self.mission_direction
            )

        # Reject planner output produced from the previous VLA corridor.
        self.latest_optimized = None
        self.latest_optimized_map = None
        self.latest_optimized_at = 0.0
        self.corridor_updated_at = time.monotonic()
        self.corridor_publish_count = 0
        self.last_inference_rgb_sequence = image_sequence
        self.segments_since_inference = 0
        trace = {
            "index": len(self.inference_trace),
            "forwarded_segments_before": self.forwarded_segments,
            "rgb_sequence": image_sequence,
            "image_age_before_request_ms": (started - image_received_at) * 1000.0,
            "inference_latency_ms": latency_ms,
            "proprio": proprio,
            "raw_action_local_delta": action.tolist(),
            "planar_action_norm_m": planar_norm,
            "body_direction_xy": body_direction.tolist(),
            "map_direction_xy": map_direction.tolist(),
            "direction_change_deg": direction_change,
            "corridor_start_xy": current.tolist(),
            "corridor_end_xy": self.plan_end.tolist(),
        }
        self.inference_trace.append(trace)
        print("OPENVLA_KSTEP_INFERENCE " + json.dumps(trace, sort_keys=True), flush=True)
        return trace

    def corridor_candidate_ready(self) -> bool:
        return (
            self.corridor_publish_count >= 3
            and self.latest_optimized_at > self.corridor_updated_at
            and time.monotonic() - self.corridor_updated_at >= 0.35
        )

    def task_completed(self) -> bool:
        if self.mission_goal is None or self.odom is None:
            return False
        pose = self.odom.pose.pose.position
        current = np.asarray([pose.x, pose.y], dtype=np.float64)
        progress = float((current - self.mission_start) @ self.mission_direction)
        goal_distance = float(np.linalg.norm(current - self.mission_goal))
        return (
            progress >= self.args.mission_distance_m - self.args.goal_tolerance_m
            and goal_distance <= self.args.goal_tolerance_m
        )

    def monitor_vehicle(self) -> None:
        pose = self.odom.pose.pose.position
        current = np.asarray([pose.x, pose.y], dtype=np.float64)
        actual_clearance = float(np.linalg.norm(current - self.pole_xy))
        self.min_actual_clearance = min(self.min_actual_clearance, actual_clearance)
        if actual_clearance < self.min_clearance:
            raise RuntimeError(
                f"actual pole clearance violated: {actual_clearance:.3f} m"
            )
        if not self.altitude - 0.45 <= pose.z <= self.altitude + 0.35:
            raise RuntimeError(f"altitude safety gate violated: {pose.z:.3f} m")

    def execute_continuous(self) -> dict:
        health = self.preflight()
        if any(word in self.args.instruction.lower() for word in STOP_WORDS):
            return {
                "status": "pass",
                "decision": "HOLD",
                "detail": "Stop/hold instruction terminated before arming.",
                "flight_command_forwarded": False,
                "armed_final": self.state.armed,
                "mode_final": self.state.mode,
                "openvla_health": health,
            }
        if self.args.preflight_only:
            self.infer_and_update_corridor()
            return {
                "status": "pass",
                "decision": "PREFLIGHT_ONLY",
                "flight_command_forwarded": False,
                "armed_final": self.state.armed,
                "mode_final": self.state.mode,
                "inference_every_k": self.args.inference_every_k,
                "inferences": self.inference_trace,
                "openvla_health": health,
            }

        self.prepare_and_takeoff()
        self.infer_and_update_corridor()

        accepted_metrics = None
        last_candidate_stamp = 0.0
        last_forward = time.monotonic()
        deadline = time.monotonic() + self.args.mission_timeout
        self.set_trajectory_mode(TrajectoryMode.Request.ADD_SEGMENT)

        while time.monotonic() < deadline:
            self.publish_global_path()
            self.corridor_publish_count += 1
            rclpy.spin_once(self, timeout_sec=0.1)
            self.monitor_vehicle()
            if self.task_completed():
                break

            if (
                self.forwarded_segments > 0
                and self.segments_since_inference >= self.args.inference_every_k
            ):
                self.infer_and_update_corridor()
                last_candidate_stamp = 0.0
                continue

            if (
                self.corridor_candidate_ready()
                and self.latest_optimized_at > last_candidate_stamp
            ):
                candidate, reason, metrics = self.gate_candidate()
                last_candidate_stamp = self.latest_optimized_at
                self.last_gate_metrics = metrics
                if candidate is None:
                    self.rejected_segments += 1
                    self.last_rejection = reason
                else:
                    self.segment_pub.publish(candidate)
                    self.forwarded_segments += 1
                    self.segments_since_inference += 1
                    last_forward = time.monotonic()
                    accepted_metrics = metrics
                    self.last_rejection = "none"

            if self.forwarded_segments == 0 and time.monotonic() - self.corridor_updated_at > 18.0:
                raise RuntimeError(f"no safe initial DROAN segment: {self.last_rejection}")
            if self.forwarded_segments > 0 and time.monotonic() - last_forward > 12.0:
                raise RuntimeError(f"safe replanning timeout: {self.last_rejection}")
        else:
            raise TimeoutError("continuous OpenVLA task did not reach its goal")

        final_position = self.odom.pose.pose.position
        final_flight_xy = [float(final_position.x), float(final_position.y)]
        self.land_and_disarm()
        return {
            "status": "pass",
            "decision": "MISSION_COMPLETE",
            "instruction": self.args.instruction,
            "inference_every_k": self.args.inference_every_k,
            "inference_count": len(self.inference_trace),
            "inferences": self.inference_trace,
            "forwarded_segments": self.forwarded_segments,
            "rejected_segments": self.rejected_segments,
            "minimum_actual_pole_center_clearance_m": self.min_actual_clearance,
            "mission_goal_xy": self.mission_goal.tolist(),
            "final_flight_xy": final_flight_xy,
            "final_z": float(self.odom.pose.pose.position.z),
            "armed_final": bool(self.state.armed),
            "mode_final": self.state.mode,
            "last_accepted_segment": accepted_metrics,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="Fly forward and avoid the utility pole")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--openvla-url")
    parser.add_argument("--inference-every-k", type=int, default=3)
    parser.add_argument("--local-horizon-m", type=float, default=4.5)
    parser.add_argument("--mission-distance-m", type=float, default=6.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=1.4)
    parser.add_argument("--mission-timeout", type=float, default=90.0)
    parser.add_argument("--inference-timeout", type=float, default=5.0)
    parser.add_argument("--fresh-image-timeout", type=float, default=4.0)
    parser.add_argument("--max-image-age", type=float, default=1.0)
    parser.add_argument("--min-direction-norm", type=float, default=1e-5)
    parser.add_argument("--max-direction-change-deg", type=float, default=75.0)
    parser.add_argument("--max-planar-action-m", type=float, default=1.0)
    parser.add_argument("--max-vertical-action-m", type=float, default=0.25)
    parser.add_argument("--max-yaw-action-rad", type=float, default=0.5)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    if args.inference_every_k < 1:
        parser.error("--inference-every-k must be at least 1")
    for name in ("local_horizon_m", "mission_distance_m", "goal_tolerance_m"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    executor = ContinuousOpenVLAExecutor(args)
    result = None
    exit_code = 1
    try:
        result = executor.execute_continuous()
        exit_code = 0
        print("OPENVLA_KSTEP_DROAN_PASS " + json.dumps(result, sort_keys=True), flush=True)
    except Exception as exc:
        result = {
            "status": "fail",
            "error": f"{type(exc).__name__}: {exc}",
            "instruction": args.instruction,
            "inference_every_k": args.inference_every_k,
            "inference_count": len(executor.inference_trace),
            "inferences": executor.inference_trace,
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
        print("OPENVLA_KSTEP_DROAN_FAIL " + json.dumps(result, sort_keys=True), flush=True)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if result is not None:
            args.output.write_text(
                json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        executor.safe_recover()
        executor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
