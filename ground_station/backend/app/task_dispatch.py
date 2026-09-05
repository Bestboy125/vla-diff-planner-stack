"""Safety-gated dispatch for operator primitives and VLA embodied missions."""

import asyncio
import math
import time
from typing import Any
from uuid import uuid4

from fastapi import HTTPException

from .mission import MissionManager
from .onboard_bridge import OnboardBridgeClient, build_operator_task
from .schemas import (
    AtomicTaskName,
    EmbodiedTaskName,
    MissionCreate,
    MissionMode,
    OrbitDirection,
    TaskCategory,
    TaskDispatchRequest,
)


ATOMIC_LABELS = {
    AtomicTaskName.TAKEOFF: "起飞到指定相对高度",
    AtomicTaskName.LAND: "降落",
    AtomicTaskName.HOLD: "悬停保持",
    AtomicTaskName.MOVE_FORWARD: "向前移动",
    AtomicTaskName.MOVE_BACKWARD: "向后移动",
    AtomicTaskName.MOVE_LEFT: "向左移动",
    AtomicTaskName.MOVE_RIGHT: "向右移动",
    AtomicTaskName.MOVE_UP: "向上移动",
    AtomicTaskName.MOVE_DOWN: "向下移动",
    AtomicTaskName.YAW_LEFT: "向左旋转",
    AtomicTaskName.YAW_RIGHT: "向右旋转",
    AtomicTaskName.ORBIT_WORLD: "按世界坐标圆心绕飞",
}


class TaskDispatcher:
    def __init__(
        self,
        mission_manager: MissionManager,
        onboard_bridge: OnboardBridgeClient,
        *,
        control_output_enabled: bool,
        operator_control_token: str,
        live_control_confirmation: str,
        command_ttl_ms: int,
        world_frame: str,
        body_frame: str,
    ) -> None:
        self.mission_manager = mission_manager
        self.onboard_bridge = onboard_bridge
        self.control_output_enabled = control_output_enabled
        self.operator_control_token = operator_control_token
        self.live_control_confirmation = live_control_confirmation
        self.command_ttl_ms = command_ttl_ms
        self.world_frame = world_frame
        self.body_frame = body_frame
        self._lock = asyncio.Lock()
        self._sequence = 0
        self._history: list[dict[str, Any]] = []

    def catalog(self) -> dict[str, Any]:
        return {
            "atomic_tasks": [
                {"name": task.value, "label": ATOMIC_LABELS[task]}
                for task in AtomicTaskName
            ],
            "embodied_tasks": [
                {"name": EmbodiedTaskName.FREEFORM.value, "label": "自由具身指令"},
                {"name": EmbodiedTaskName.ORBIT_TARGET.value, "label": "按半径绕目标飞行"},
                {
                    "name": EmbodiedTaskName.PASS_TARGET_FORWARD.value,
                    "label": "飞过目标后继续前进",
                },
            ],
            "limits": {
                "distance_m": [0.05, 2.0],
                "takeoff_height_m": [0.8, 0.8],
                "yaw_deg": [1.0, 90.0],
                "radius_m": [0.5, 5.0],
                "laps": [0.25, 3.0],
                "extra_distance_m": [0.2, 5.0],
            },
            "control_output_enabled": self.control_output_enabled,
            "live_confirmation_phrase": self.live_control_confirmation,
        }

    async def dispatch(
        self, request: TaskDispatchRequest, operator_token: str | None
    ) -> dict[str, Any]:
        if request.mode == MissionMode.LIVE:
            self._require_live_authorization(request, operator_token)
        if request.category == TaskCategory.ATOMIC:
            result = await self._dispatch_atomic(request)
        else:
            result = await self._dispatch_embodied(request)
        async with self._lock:
            self._history.insert(0, result)
            del self._history[20:]
        return result

    async def snapshot(self) -> dict[str, Any]:
        async with self._lock:
            return {
                "control_output_enabled": self.control_output_enabled,
                "recent_tasks": [dict(item) for item in self._history[:8]],
            }

    def _require_live_authorization(
        self, request: TaskDispatchRequest, operator_token: str | None
    ) -> None:
        if not self.control_output_enabled:
            raise HTTPException(
                status_code=423,
                detail="Live task output is locked by CONTROL_OUTPUT_ENABLED=false.",
            )
        if not self.operator_control_token or self.operator_control_token == "REQUIRED":
            raise HTTPException(
                status_code=503,
                detail="OPERATOR_CONTROL_TOKEN is not configured on the host.",
            )
        if operator_token != self.operator_control_token:
            raise HTTPException(status_code=401, detail="Invalid operator control token.")
        if request.live_confirmation != self.live_control_confirmation:
            raise HTTPException(status_code=428, detail="Live-control confirmation does not match.")

    async def _dispatch_atomic(self, request: TaskDispatchRequest) -> dict[str, Any]:
        assert request.atomic_task is not None
        task_id = str(uuid4())
        magnitude = self._atomic_magnitude(request)
        if request.atomic_task == AtomicTaskName.TAKEOFF and abs(magnitude - 0.8) > 1e-6:
            raise HTTPException(status_code=422, detail="Takeoff height is fixed at 0.8 m in the VLA launch configuration.")
        async with self._lock:
            sequence = self._sequence
            self._sequence += 1
        command = build_operator_task(
            task_id=task_id,
            sequence=sequence,
            action=request.atomic_task,
            magnitude=magnitude,
            ttl_ms=self.command_ttl_ms,
            world_frame=self.world_frame,
            body_frame=self.body_frame,
            orbit=(
                {
                    "center": [
                        request.parameters.center_x_m,
                        request.parameters.center_y_m,
                        request.parameters.center_z_m,
                    ],
                    "radius_m": request.parameters.radius_m,
                    "laps": request.parameters.laps,
                    "direction": request.parameters.orbit_direction.value,
                    "yaw_mode": "face_center",
                }
                if request.atomic_task == AtomicTaskName.ORBIT_WORLD else None
            ),
        )
        if request.mode == MissionMode.DRY_RUN:
            delivery = {
                "status": "safety_locked",
                "detail": "Atomic task validated but not sent because mode=dry_run.",
            }
        else:
            try:
                delivery = await self.onboard_bridge.send(command)
            except RuntimeError as exc:
                raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {
            "task_id": task_id,
            "category": request.category.value,
            "task": request.atomic_task.value,
            "label": ATOMIC_LABELS[request.atomic_task],
            "mode": request.mode.value,
            "created_unix_ms": int(time.time() * 1000),
            "command": command,
            "delivery": delivery,
        }

    async def _dispatch_embodied(self, request: TaskDispatchRequest) -> dict[str, Any]:
        instruction = self._compose_embodied_instruction(request)
        mission = await self.mission_manager.create(
            MissionCreate(instruction=instruction, policy=request.policy, mode=request.mode)
        )
        mission = await self.mission_manager.start(mission.mission_id)
        return {
            "task_id": str(mission.mission_id),
            "category": request.category.value,
            "task": request.embodied_task.value if request.embodied_task else "freeform",
            "label": "VLA + Diff-Planner 具身任务",
            "mode": request.mode.value,
            "created_unix_ms": int(time.time() * 1000),
            "normalized_instruction": instruction,
            "mission": mission.model_dump(mode="json"),
            "delivery": {
                "status": "mission_started",
                "detail": "Waiting for the next K-frame VLA inference cycle.",
            },
        }

    @staticmethod
    def _atomic_magnitude(request: TaskDispatchRequest) -> float:
        assert request.atomic_task is not None
        if request.atomic_task == AtomicTaskName.TAKEOFF:
            return request.parameters.takeoff_height_m
        if request.atomic_task in {AtomicTaskName.YAW_LEFT, AtomicTaskName.YAW_RIGHT}:
            return math.radians(request.parameters.yaw_deg)
        if request.atomic_task in {AtomicTaskName.HOLD, AtomicTaskName.LAND}:
            return 0.0
        if request.atomic_task == AtomicTaskName.ORBIT_WORLD:
            return request.parameters.radius_m
        return request.parameters.distance_m

    @staticmethod
    def _compose_embodied_instruction(request: TaskDispatchRequest) -> str:
        assert request.embodied_task is not None
        params = request.parameters
        if request.embodied_task == EmbodiedTaskName.FREEFORM:
            return request.instruction
        if request.embodied_task == EmbodiedTaskName.ORBIT_TARGET:
            direction = (
                "顺时针"
                if params.orbit_direction == OrbitDirection.CLOCKWISE
                else "逆时针"
            )
            return (
                f"识别并接近{params.target_label}，保持约{params.radius_m:.2f}米安全半径，"
                f"以{direction}方向绕飞{params.laps:g}圈；持续避障并保持当前安全高度，完成后悬停。"
            )
        return (
            f"识别{params.target_label}并安全飞过该目标，飞过后沿机头方向继续前进"
            f"{params.extra_distance_m:.2f}米；全程避障，完成后悬停。"
        )
