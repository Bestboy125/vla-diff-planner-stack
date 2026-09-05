"""Ground-station side of the TCP/NDJSON Diff-Planner bridge contract."""

import asyncio
import json
import time
from typing import Any

from .schemas import AtomicTaskName, BridgeCommandName, BridgeCommandRequest, PlanningPreviewRequest


ACTION_SEMANTIC = ["dx_body", "dy_body", "dz_body", "d_yaw"]
ACTION_UNITS = ["m", "m", "m", "rad"]


def build_trajectory_command(request: BridgeCommandRequest, ttl_ms: int) -> dict[str, Any]:
    if request.command == BridgeCommandName.TRACK:
        if request.action_local_delta is None or request.target_mission is None:
            raise ValueError("TRACK requires action_local_delta and target_mission")
    return {
        "schema_version": 1,
        "type": "trajectory_command",
        "mission_id": str(request.mission_id),
        "sequence": request.sequence,
        "sent_at_unix_ms": int(time.time() * 1000),
        "ttl_ms": ttl_ms,
        "policy": request.policy.value,
        "command": request.command.value,
        "frame_id": "world",
        "action_semantic": ACTION_SEMANTIC,
        "action_units": ACTION_UNITS,
        "action_local_delta": request.action_local_delta,
        "target_mission": request.target_mission,
        "action_chunk": request.action_chunk,
    }


def build_planning_preview(request: PlanningPreviewRequest, ttl_ms: int) -> dict[str, Any]:
    """Build a planner-only message. It is never interpreted as a flight command."""
    return {
        "schema_version": 2,
        "type": "planning_preview",
        "mission_id": str(request.mission_id),
        "sequence": request.sequence,
        "sent_at_unix_ms": int(time.time() * 1000),
        "ttl_ms": ttl_ms,
        "policy": request.policy.value,
        "command": "PLAN_PREVIEW",
        "frame_id": request.world_frame_id,
        "body_frame_id": request.body_frame_id,
        "camera_frame_id": request.camera_frame_id,
        "calibration_id": request.calibration_id,
        "source_observation": {
            "vehicle_id": request.source_vehicle_id,
            "sequence": request.source_observation_sequence,
            "capture_unix_ms": request.source_capture_unix_ms,
        },
        "action_semantic": ACTION_SEMANTIC,
        "action_units": ACTION_UNITS,
        "action_local_delta": request.action_local_delta,
        "target_mission": request.target_mission,
        "action_chunk": request.action_chunk,
    }


def build_operator_task(
    task_id: str,
    sequence: int,
    action: AtomicTaskName,
    magnitude: float,
    ttl_ms: int,
    world_frame: str,
    body_frame: str,
    orbit: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a one-shot, operator-selected onboard primitive."""
    unit = "rad" if action in {AtomicTaskName.YAW_LEFT, AtomicTaskName.YAW_RIGHT} else "m"
    if action in {AtomicTaskName.HOLD, AtomicTaskName.LAND}:
        unit = "none"
    result = {
        "schema_version": 3,
        "type": "operator_task",
        "task_id": task_id,
        "sequence": sequence,
        "sent_at_unix_ms": int(time.time() * 1000),
        "ttl_ms": ttl_ms,
        "command": action.value.upper(),
        "frame_id": world_frame,
        "body_frame_id": body_frame,
        "magnitude": magnitude,
        "magnitude_unit": unit,
    }
    if orbit is not None:
        result["orbit"] = orbit
    return result


def build_semantic_orbit_task(
    task_id: str,
    sequence: int,
    target_label: str,
    ttl_ms: int,
    world_frame: str,
    body_frame: str,
    direction: str,
) -> dict[str, Any]:
    """Build the authenticated request consumed by the onboard semantic-orbit state machine."""
    return {
        "schema_version": 3,
        "type": "operator_task",
        "task_id": task_id,
        "sequence": sequence,
        "sent_at_unix_ms": int(time.time() * 1000),
        "ttl_ms": ttl_ms,
        "command": "SEMANTIC_ORBIT",
        "frame_id": world_frame,
        "body_frame_id": body_frame,
        "magnitude": 1.5,
        "magnitude_unit": "m",
        "semantic_orbit": {
            "target_label": target_label.lower(),
            "radius_m": 1.5,
            "laps": 1.0,
            "direction": direction,
            "yaw_mode": "face_center",
            "keep_current_altitude": True,
        },
    }


class OnboardBridgeClient:
    def __init__(self, host: str, port: int, token: str, timeout_sec: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.token = token
        self.timeout_sec = timeout_sec

    async def send(self, command: dict[str, Any]) -> dict[str, Any]:
        if not self.token or self.token == "REQUIRED":
            raise RuntimeError("ONBOARD_BRIDGE_TOKEN must be configured before live delivery")
        wire_command = dict(command)
        wire_command["auth_token"] = self.token
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port), timeout=self.timeout_sec
            )
            writer.write(json.dumps(wire_command, separators=(",", ":")).encode("utf-8") + b"\n")
            await asyncio.wait_for(writer.drain(), timeout=self.timeout_sec)
            raw_ack = await asyncio.wait_for(reader.readline(), timeout=self.timeout_sec)
            writer.close()
            await writer.wait_closed()
        except (OSError, asyncio.TimeoutError) as exc:
            raise RuntimeError("onboard bridge unavailable: {}".format(exc)) from exc
        if not raw_ack:
            raise RuntimeError("onboard bridge closed without an acknowledgement")
        try:
            ack = json.loads(raw_ack)
        except json.JSONDecodeError as exc:
            raise RuntimeError("onboard bridge returned invalid JSON") from exc
        if ack.get("type") not in {"trajectory_ack", "planning_preview_ack", "operator_task_ack"}:
            raise RuntimeError("onboard bridge returned an unexpected message type")
        identifier_key = "task_id" if command.get("type") == "operator_task" else "mission_id"
        if ack.get(identifier_key) != command[identifier_key] or ack.get("sequence") != command["sequence"]:
            raise RuntimeError("onboard acknowledgement does not match the command")
        return ack
