"""K-frame onboard observation pipeline for planner-only VLA previews."""

import asyncio
import base64
from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Any

from .mission import MissionManager
from .model_gateway import ModelGateway
from .onboard_bridge import OnboardBridgeClient, build_planning_preview, build_trajectory_command
from .schemas import (
    BridgeCommandName,
    BridgeCommandRequest,
    InferenceRequest,
    MissionMode,
    MissionState,
    OnboardObservation,
    PlanningPreviewRequest,
    PolicyName,
)


@dataclass(frozen=True)
class ObservationContract:
    k_frames: int
    max_age_ms: int
    max_sync_error_ms: int
    world_frame: str
    body_frame: str
    camera_frame: str
    calibration_id: str
    max_future_skew_ms: int = 300
    observation_mode: str = "calibrated"

    def __post_init__(self) -> None:
        if self.observation_mode not in {"calibrated", "image_odom"}:
            raise ValueError("VLA_OBSERVATION_MODE must be calibrated or image_odom")
        if self.k_frames < 1:
            raise ValueError("OBSERVATION_K_FRAMES must be >= 1")
        if self.max_future_skew_ms < 0:
            raise ValueError("OBSERVATION_MAX_FUTURE_SKEW_MS must be >= 0")


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_delta_to_world_target(
    position: tuple[float, float, float], yaw: float, action: list[list[float]]
) -> list[list[float]]:
    """Convert ROS FLU body delta to ENU world target using the FAST-LIO yaw."""
    dx_body, dy_body, dz_body, d_yaw = (float(item) for item in action[0])
    x, y, z = position
    return [[
        x + math.cos(yaw) * dx_body - math.sin(yaw) * dy_body,
        y + math.sin(yaw) * dx_body + math.cos(yaw) * dy_body,
        z + dz_body,
        math.atan2(math.sin(yaw + d_yaw), math.cos(yaw + d_yaw)),
    ]]


class ObservationPipeline:
    """Keeps image ingestion non-blocking and infers on every Kth fresh frame."""

    def __init__(
        self,
        contract: ObservationContract,
        mission_manager: MissionManager,
        model_gateway: ModelGateway,
        onboard_bridge: OnboardBridgeClient,
        command_ttl_ms: int,
    ) -> None:
        self.contract = contract
        self.mission_manager = mission_manager
        self.model_gateway = model_gateway
        self.onboard_bridge = onboard_bridge
        self.command_ttl_ms = command_ttl_ms
        self._lock = asyncio.Lock()
        self._latest: OnboardObservation | None = None
        self._latest_jpeg: bytes | None = None
        self._last_sequence: dict[str, int] = {}
        self._accepted_frames = 0
        self._frames_since_inference = 0
        self._inference_task: asyncio.Task | None = None
        self._preview_sequence = 0
        self._last_result: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._last_received_monotonic: float | None = None
        self._receive_times: deque[float] = deque(maxlen=60)
        self._diagnostic_busy = False
        self._last_diagnostic: dict[str, Any] | None = None

    async def ingest(self, observation: OnboardObservation) -> dict[str, Any]:
        jpeg = self._validate_observation(observation)
        async with self._lock:
            previous = self._last_sequence.get(observation.vehicle_id, -1)
            if observation.sequence <= previous:
                raise ValueError("observation sequence is duplicate or out of order")
            self._last_sequence[observation.vehicle_id] = observation.sequence
            self._latest = observation
            self._latest_jpeg = jpeg
            received_at = time.monotonic()
            self._last_received_monotonic = received_at
            self._receive_times.append(received_at)
            self._accepted_frames += 1
            self._frames_since_inference += 1
            should_infer = self._frames_since_inference >= self.contract.k_frames
            busy = self._diagnostic_busy or (self._inference_task is not None and not self._inference_task.done())
            if should_infer and not busy:
                self._frames_since_inference = 0
                self._inference_task = asyncio.create_task(self._infer_and_preview(observation))
        return {
            "status": "accepted",
            "sequence": observation.sequence,
            "selected_for_inference": should_infer and not busy,
            "k_frames": self.contract.k_frames,
        }

    async def infer_latest_no_motion(self, instruction: str, policy: PolicyName) -> dict[str, Any]:
        """Model diagnostic only. Never creates a mission or sends a bridge command."""
        mission = await self.mission_manager.current()
        if mission is not None and mission.state == MissionState.RUNNING:
            raise ValueError('diagnostic unavailable while a mission is RUNNING')
        async with self._lock:
            if self._diagnostic_busy or (self._inference_task is not None and not self._inference_task.done()):
                raise ValueError('inference is busy; retry when idle')
            if self._latest is None:
                raise ValueError('no onboard observation is available')
            observation = self._latest.model_copy(deep=True)
            self._validate_observation(observation)
            self._diagnostic_busy = True
        started = time.monotonic()
        try:
            pose = observation.odometry.pose
            q = pose.orientation
            yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
            request = InferenceRequest(
                image_base64=observation.image_base64, instruction=instruction,
                proprio=[pose.position.x, pose.position.y, pose.position.z, math.degrees(yaw)],
            )
            predict = self.model_gateway.predict_openvla if policy == PolicyName.OPENVLA else self.model_gateway.predict_pi05
            prediction = await predict(request)
            action = prediction['action_local_delta']
            if (len(action) != 1 or len(action[0]) != 4
                    or not all(math.isfinite(float(value)) for value in action[0])):
                raise ValueError('model action must be one finite 4D body delta')
            action = [[float(value) for value in action[0]]]
            result = {
                'policy': policy.value,
                'observation_sequence': observation.sequence,
                'capture_unix_ms': observation.capture_unix_ms,
                'action_local_delta': action,
                'target_world': body_delta_to_world_target(
                    (pose.position.x, pose.position.y, pose.position.z), yaw, action),
                'latency_ms': round((time.monotonic() - started) * 1000, 1),
                'source_age_at_completion_ms': int(time.time() * 1000) - observation.capture_unix_ms,
                'motion_command_sent': False,
                'flight_execution_validated': False,
                'output_mode': 'diagnostic_only',
            }
            async with self._lock:
                self._last_diagnostic = result
            return result
        finally:
            async with self._lock:
                self._diagnostic_busy = False

    def _validate_observation(self, observation: OnboardObservation) -> bytes:
        now_ms = int(time.time() * 1000)
        age_ms = now_ms - observation.capture_unix_ms
        if age_ms < -self.contract.max_future_skew_ms or age_ms > self.contract.max_age_ms:
            raise ValueError("observation timestamp is stale or clock-unsynchronized")
        sync_error = abs(observation.capture_unix_ms - observation.odometry.stamp_unix_ms)
        if sync_error > self.contract.max_sync_error_ms:
            raise ValueError("image and FAST-LIO odometry are not time-synchronized")
        if observation.odometry.frame_id != self.contract.world_frame:
            raise ValueError("FAST-LIO world frame does not match EXPECTED_WORLD_FRAME")
        if observation.odometry.child_frame_id != self.contract.body_frame:
            raise ValueError("odometry child frame does not match EXPECTED_BODY_FRAME")
        if observation.image_frame_id != self.contract.camera_frame:
            raise ValueError("image frame does not match EXPECTED_CAMERA_FRAME")
        if observation.observation_mode != self.contract.observation_mode:
            raise ValueError("observation mode does not match host configuration")
        transform = observation.body_from_camera
        if self.contract.observation_mode == "calibrated":
            if transform is None:
                raise ValueError("calibrated mode requires camera extrinsics")
            if transform.parent_frame_id != self.contract.body_frame:
                raise ValueError("camera extrinsic parent must be the body frame")
            if transform.child_frame_id != self.contract.camera_frame:
                raise ValueError("camera extrinsic child must be the optical frame")
            if not observation.calibration_validated:
                raise ValueError("camera calibration is not operator-validated")
        elif transform is not None or observation.calibration_validated:
            raise ValueError("image_odom cannot claim calibrated camera extrinsics")
        if self.contract.calibration_id == "REQUIRED":
            raise ValueError("EXPECTED_CALIBRATION_ID must be configured")
        if observation.calibration_id != self.contract.calibration_id:
            raise ValueError("camera calibration_id does not match host configuration")
        try:
            jpeg = base64.b64decode(observation.image_base64, validate=True)
        except Exception as exc:
            raise ValueError("image_base64 is not valid base64") from exc
        if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            raise ValueError("image payload is not a complete JPEG frame")
        return jpeg

    async def _infer_and_preview(self, observation: OnboardObservation) -> None:
        mission = await self.mission_manager.current()
        if mission is None or mission.state != MissionState.RUNNING:
            async with self._lock:
                self._last_error = "Kth frame received; no RUNNING mission, inference skipped"
            return
        pose = observation.odometry.pose
        q = pose.orientation
        yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
        request = InferenceRequest(
            image_base64=observation.image_base64,
            instruction=mission.instruction,
            proprio=[pose.position.x, pose.position.y, pose.position.z, math.degrees(yaw)],
        )
        try:
            if mission.policy == PolicyName.OPENVLA:
                result = await self.model_gateway.predict_openvla(request)
            else:
                result = await self.model_gateway.predict_pi05(request)
            action = result["action_local_delta"]
            action_chunk = result.get("action_chunk") or action
            target = body_delta_to_world_target(
                (pose.position.x, pose.position.y, pose.position.z), yaw, action
            )
            async with self._lock:
                sequence = self._preview_sequence
                self._preview_sequence += 1
            if mission.mode == MissionMode.LIVE:
                command = build_trajectory_command(
                    BridgeCommandRequest(
                        mission_id=mission.mission_id,
                        sequence=sequence,
                        policy=mission.policy,
                        command=BridgeCommandName.TRACK,
                        action_local_delta=action,
                        target_mission=target,
                        action_chunk=action_chunk,
                    ),
                    self.command_ttl_ms,
                )
                output_mode = "live_trajectory"
            else:
                preview = PlanningPreviewRequest(
                    mission_id=mission.mission_id,
                    sequence=sequence,
                    policy=mission.policy,
                    source_vehicle_id=observation.vehicle_id,
                    source_observation_sequence=observation.sequence,
                    source_capture_unix_ms=observation.capture_unix_ms,
                    calibration_id=observation.calibration_id,
                    world_frame_id=self.contract.world_frame,
                    body_frame_id=self.contract.body_frame,
                    camera_frame_id=self.contract.camera_frame,
                    action_local_delta=action,
                    target_mission=target,
                    action_chunk=action_chunk,
                )
                command = build_planning_preview(preview, self.command_ttl_ms)
                output_mode = "planner_preview"
            ack = await self.onboard_bridge.send(command)
            async with self._lock:
                self._last_result = {
                    "observation_sequence": observation.sequence,
                    "preview_sequence": sequence,
                    "policy": mission.policy.value,
                    "action_local_delta": action,
                    "target_world": target,
                    "output_mode": output_mode,
                    "delivery": ack,
                    "completed_unix_ms": int(time.time() * 1000),
                }
                self._last_error = None
        except Exception as exc:
            async with self._lock:
                self._last_error = "{}: {}".format(type(exc).__name__, exc)

    async def snapshot(self) -> dict[str, Any]:
        async with self._lock:
            latest = self._latest
            busy = self._inference_task is not None and not self._inference_task.done()
            now_monotonic = time.monotonic()
            receive_age_ms = (
                round((now_monotonic - self._last_received_monotonic) * 1000.0, 1)
                if self._last_received_monotonic is not None
                else None
            )
            receive_fps = 0.0
            if len(self._receive_times) >= 2:
                elapsed = self._receive_times[-1] - self._receive_times[0]
                if elapsed > 0:
                    receive_fps = round((len(self._receive_times) - 1) / elapsed, 1)
            local_state = None
            if latest:
                pose = latest.odometry.pose
                q = pose.orientation
                local_state = {
                    "position": pose.position.model_dump(),
                    "yaw_rad": quaternion_yaw(q.x, q.y, q.z, q.w),
                    "linear_velocity": latest.odometry.linear_velocity.model_dump(),
                    "angular_velocity": latest.odometry.angular_velocity.model_dump(),
                    "odom_stamp_unix_ms": latest.odometry.stamp_unix_ms,
                }
            return {
                "connected": latest is not None,
                "vehicle_id": latest.vehicle_id if latest else None,
                "image_sequence": latest.sequence if latest else None,
                "capture_unix_ms": latest.capture_unix_ms if latest else None,
                "world_frame": latest.odometry.frame_id if latest else None,
                "body_frame": latest.odometry.child_frame_id if latest else None,
                "camera_frame": latest.image_frame_id if latest else None,
                "calibration_id": latest.calibration_id if latest else None,
                "calibration_validated": latest.calibration_validated if latest else False,
                "observation_mode": self.contract.observation_mode,
                "camera_extrinsics_used": self.contract.observation_mode == "calibrated",
                "accepted_frames": self._accepted_frames,
                "receive_age_ms": receive_age_ms,
                "receive_fps": receive_fps,
                "frames_until_inference": self.contract.k_frames - self._frames_since_inference,
                "k_frames": self.contract.k_frames,
                "inference_busy": busy,
                "last_result": self._last_result,
                "diagnostic_busy": self._diagnostic_busy,
                "last_diagnostic": self._last_diagnostic,
                "last_error": self._last_error,
                "local_state": local_state,
                "planner_preview": latest.planner_preview.model_dump() if latest and latest.planner_preview else None,
            }

    async def latest_jpeg(self) -> tuple[bytes, int] | None:
        async with self._lock:
            if self._latest_jpeg is None or self._latest is None:
                return None
            return self._latest_jpeg, self._latest.sequence
