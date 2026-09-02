from datetime import datetime, timezone
from enum import Enum
from typing import Literal
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator, model_validator


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


class PolicyName(str, Enum):
    OPENVLA = "openvla"
    PI05 = "pi05"


class MissionMode(str, Enum):
    DRY_RUN = "dry_run"
    LIVE = "live"


class MissionState(str, Enum):
    IDLE = "IDLE"
    ARMED = "ARMED"
    RUNNING = "RUNNING"
    HOLDING = "HOLDING"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    FAULT = "FAULT"


class TaskCategory(str, Enum):
    ATOMIC = "atomic"
    EMBODIED = "embodied"


class AtomicTaskName(str, Enum):
    TAKEOFF = "takeoff"
    LAND = "land"
    HOLD = "hold"
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    MOVE_UP = "move_up"
    MOVE_DOWN = "move_down"
    YAW_LEFT = "yaw_left"
    YAW_RIGHT = "yaw_right"


class EmbodiedTaskName(str, Enum):
    FREEFORM = "freeform"
    ORBIT_TARGET = "orbit_target"
    PASS_TARGET_FORWARD = "pass_target_forward"


class OrbitDirection(str, Enum):
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"


class MissionCreate(BaseModel):
    instruction: str = Field(min_length=2, max_length=500)
    policy: PolicyName = PolicyName.OPENVLA
    mode: MissionMode = MissionMode.DRY_RUN

    @field_validator("instruction")
    @classmethod
    def normalize_instruction(cls, value: str) -> str:
        normalized = " ".join(value.split())
        if not normalized:
            raise ValueError("instruction cannot be blank")
        return normalized


class TaskParameters(BaseModel):
    distance_m: float = Field(default=0.5, ge=0.05, le=1.0)
    takeoff_height_m: float = Field(default=1.0, ge=0.3, le=2.0)
    yaw_deg: float = Field(default=30.0, ge=1.0, le=90.0)
    target_label: str = Field(default="", max_length=100)
    radius_m: float = Field(default=1.5, ge=0.5, le=5.0)
    laps: float = Field(default=1.0, ge=0.25, le=3.0)
    orbit_direction: OrbitDirection = OrbitDirection.CLOCKWISE
    extra_distance_m: float = Field(default=2.0, ge=0.2, le=5.0)

    @field_validator("target_label")
    @classmethod
    def normalize_target_label(cls, value: str) -> str:
        return " ".join(value.split())


class TaskDispatchRequest(BaseModel):
    category: TaskCategory
    atomic_task: AtomicTaskName | None = None
    embodied_task: EmbodiedTaskName | None = None
    instruction: str = Field(default="", max_length=500)
    policy: PolicyName = PolicyName.OPENVLA
    mode: MissionMode = MissionMode.DRY_RUN
    parameters: TaskParameters = Field(default_factory=TaskParameters)
    live_confirmation: str = Field(default="", max_length=128)

    @field_validator("instruction")
    @classmethod
    def normalize_task_instruction(cls, value: str) -> str:
        return " ".join(value.split())

    @model_validator(mode="after")
    def validate_task_selection(self) -> "TaskDispatchRequest":
        if self.category == TaskCategory.ATOMIC:
            if self.atomic_task is None:
                raise ValueError("atomic_task is required for an atomic task")
            if self.embodied_task is not None:
                raise ValueError("embodied_task is not valid for an atomic task")
        else:
            if self.embodied_task is None:
                raise ValueError("embodied_task is required for an embodied task")
            if self.atomic_task is not None:
                raise ValueError("atomic_task is not valid for an embodied task")
            if self.embodied_task == EmbodiedTaskName.FREEFORM and len(self.instruction) < 2:
                raise ValueError("freeform embodied tasks require an instruction")
            if self.embodied_task in {
                EmbodiedTaskName.ORBIT_TARGET,
                EmbodiedTaskName.PASS_TARGET_FORWARD,
            } and not self.parameters.target_label:
                raise ValueError("the selected embodied task requires target_label")
        return self


class Mission(BaseModel):
    mission_id: UUID = Field(default_factory=uuid4)
    instruction: str
    policy: PolicyName
    mode: MissionMode
    state: MissionState = MissionState.ARMED
    status_message: str = "Mission created; waiting to start."
    created_at: datetime = Field(default_factory=utc_now)
    updated_at: datetime = Field(default_factory=utc_now)


class InferenceRequest(BaseModel):
    image_base64: str = Field(min_length=16, max_length=8_000_000)
    instruction: str = Field(min_length=2, max_length=500)
    proprio: list[float] = Field(min_length=4, max_length=4)

    @field_validator("proprio")
    @classmethod
    def validate_proprio(cls, values: list[float]) -> list[float]:
        import math

        if not all(math.isfinite(value) for value in values):
            raise ValueError("proprio must contain finite values")
        return values


class ServiceState(BaseModel):
    status: Literal["online", "offline", "degraded"]
    detail: str
    latency_ms: float | None = None


class BridgeCommandName(str, Enum):
    TRACK = "TRACK"
    HOLD = "HOLD"
    COMPLETE = "COMPLETE"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class Vector3(BaseModel):
    x: float
    y: float
    z: float


class Quaternion(BaseModel):
    x: float
    y: float
    z: float
    w: float

    @model_validator(mode="after")
    def validate_unit_quaternion(self) -> "Quaternion":
        import math

        values = (self.x, self.y, self.z, self.w)
        if not all(math.isfinite(item) for item in values):
            raise ValueError("quaternion must contain finite values")
        norm = math.sqrt(sum(item * item for item in values))
        if not 0.99 <= norm <= 1.01:
            raise ValueError("quaternion must be normalized")
        return self


class PoseState(BaseModel):
    position: Vector3
    orientation: Quaternion


class OdometryState(BaseModel):
    stamp_unix_ms: int = Field(gt=0)
    frame_id: str = Field(min_length=1, max_length=128)
    child_frame_id: str = Field(min_length=1, max_length=128)
    pose: PoseState
    linear_velocity: Vector3
    angular_velocity: Vector3


class CameraIntrinsics(BaseModel):
    width: int = Field(gt=0, le=8192)
    height: int = Field(gt=0, le=8192)
    distortion_model: str = Field(min_length=1, max_length=64)
    k: list[float] = Field(min_length=9, max_length=9)
    d: list[float] = Field(max_length=16)

    @field_validator("k", "d")
    @classmethod
    def finite_calibration(cls, value: list[float]) -> list[float]:
        import math

        if not all(math.isfinite(item) for item in value):
            raise ValueError("camera calibration must contain finite values")
        return value


class RigidTransform(BaseModel):
    parent_frame_id: str = Field(min_length=1, max_length=128)
    child_frame_id: str = Field(min_length=1, max_length=128)
    translation: Vector3
    rotation: Quaternion


class PlannerPreviewState(BaseModel):
    stamp_unix_ms: int = Field(gt=0)
    frame_id: str = Field(min_length=1, max_length=128)
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    yaw: float


class OnboardObservation(BaseModel):
    schema_version: Literal[1] = 1
    type: Literal["onboard_observation"] = "onboard_observation"
    vehicle_id: str = Field(min_length=1, max_length=64)
    sequence: int = Field(ge=0)
    capture_unix_ms: int = Field(gt=0)
    image_encoding: Literal["jpeg"] = "jpeg"
    image_base64: str = Field(min_length=16, max_length=8_000_000)
    image_frame_id: str = Field(min_length=1, max_length=128)
    odometry: OdometryState
    camera_intrinsics: CameraIntrinsics
    body_from_camera: RigidTransform
    calibration_id: str = Field(min_length=1, max_length=128)
    calibration_validated: bool = False
    planner_preview: PlannerPreviewState | None = None


class PlanningPreviewRequest(BaseModel):
    mission_id: UUID
    sequence: int = Field(ge=0)
    policy: PolicyName
    source_vehicle_id: str
    source_observation_sequence: int = Field(ge=0)
    source_capture_unix_ms: int = Field(gt=0)
    calibration_id: str
    world_frame_id: str
    body_frame_id: str
    camera_frame_id: str
    action_local_delta: list[list[float]]
    target_mission: list[list[float]]

    @field_validator("action_local_delta", "target_mission")
    @classmethod
    def validate_preview_vector(cls, value: list[list[float]]) -> list[list[float]]:
        import math

        if len(value) != 1 or len(value[0]) != 4:
            raise ValueError("value must have shape [1, 4]")
        if not all(math.isfinite(item) for item in value[0]):
            raise ValueError("value must contain finite numbers")
        return value


class BridgeCommandRequest(BaseModel):
    mission_id: UUID
    sequence: int = Field(ge=0)
    policy: PolicyName
    command: BridgeCommandName
    action_local_delta: list[list[float]] | None = None
    target_mission: list[list[float]] | None = None

    @field_validator("action_local_delta", "target_mission")
    @classmethod
    def validate_optional_vector(cls, value: list[list[float]] | None) -> list[list[float]] | None:
        import math

        if value is None:
            return None
        if len(value) != 1 or len(value[0]) != 4:
            raise ValueError("value must have shape [1, 4]")
        if not all(math.isfinite(item) for item in value[0]):
            raise ValueError("value must contain finite numbers")
        return value

    @model_validator(mode="after")
    def require_track_vectors(self) -> "BridgeCommandRequest":
        if self.command == BridgeCommandName.TRACK:
            if self.action_local_delta is None or self.target_mission is None:
                raise ValueError("TRACK requires action_local_delta and target_mission")
        return self
