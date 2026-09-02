import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Settings:
    host: str = os.environ.get("GROUND_STATION_HOST", "0.0.0.0")
    port: int = int(os.environ.get("GROUND_STATION_PORT", "8080"))
    openvla_url: str = os.environ.get("OPENVLA_URL", "http://127.0.0.1:5007")
    pi05_host: str = os.environ.get("PI05_HOST", "127.0.0.1")
    pi05_port: int = int(os.environ.get("PI05_PORT", "8000"))
    control_output_enabled: bool = os.environ.get("CONTROL_OUTPUT_ENABLED", "false").lower() == "true"
    host_onboard_ip: str = os.environ.get("HOST_ONBOARD_IP", "127.0.0.1")
    host_operator_ip: str = os.environ.get("HOST_OPERATOR_IP", "127.0.0.1")
    onboard_bridge_host: str = os.environ.get("ONBOARD_BRIDGE_HOST", "127.0.0.1")
    onboard_bridge_port: int = int(os.environ.get("ONBOARD_BRIDGE_PORT", "50051"))
    onboard_bridge_token: str = os.environ.get("ONBOARD_BRIDGE_TOKEN", "REQUIRED")
    onboard_command_ttl_ms: int = int(os.environ.get("ONBOARD_COMMAND_TTL_MS", "500"))
    operator_control_token: str = os.environ.get("OPERATOR_CONTROL_TOKEN", "REQUIRED")
    live_control_confirmation: str = os.environ.get(
        "LIVE_CONTROL_CONFIRMATION", "I_ACCEPT_REAL_FLIGHT_CONTROL"
    )
    onboard_observation_token: str = os.environ.get("ONBOARD_OBSERVATION_TOKEN", "REQUIRED")
    observation_k_frames: int = int(os.environ.get("OBSERVATION_K_FRAMES", "5"))
    observation_max_age_ms: int = int(os.environ.get("OBSERVATION_MAX_AGE_MS", "750"))
    observation_max_future_skew_ms: int = int(
        os.environ.get("OBSERVATION_MAX_FUTURE_SKEW_MS", "1000")
    )
    observation_max_sync_error_ms: int = int(os.environ.get("OBSERVATION_MAX_SYNC_ERROR_MS", "80"))
    expected_world_frame: str = os.environ.get("EXPECTED_WORLD_FRAME", "world")
    expected_body_frame: str = os.environ.get("EXPECTED_BODY_FRAME", "base_link")
    expected_camera_frame: str = os.environ.get("EXPECTED_CAMERA_FRAME", "camera_color_optical_frame")
    expected_calibration_id: str = os.environ.get("EXPECTED_CALIBRATION_ID", "REQUIRED")
    status_update_hz: float = float(os.environ.get("STATUS_UPDATE_HZ", "5"))
    video_stream_hz: float = float(os.environ.get("VIDEO_STREAM_HZ", "20"))
    frontend_dist: Path = Path(__file__).resolve().parents[2] / "frontend" / "dist"


settings = Settings()
