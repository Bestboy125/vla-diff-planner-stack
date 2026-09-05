import base64

import pytest
from pydantic import ValidationError

from app.observation_pipeline import ObservationContract, ObservationPipeline
from app.schemas import OnboardObservation


@pytest.fixture
def payload():
    vector = {"x": 0, "y": 0, "z": 0}
    return {
        "vehicle_id": "uav0", "sequence": 1, "capture_unix_ms": 1000000,
        "image_base64": base64.b64encode(b"\xff\xd8abcdefgh\xff\xd9").decode(),
        "image_frame_id": "usb", "observation_mode": "image_odom",
        "body_from_camera": None, "calibration_validated": False, "calibration_id": "usb-v1",
        "camera_intrinsics": {"width": 640, "height": 480, "distortion_model": "plumb_bob", "k": [1]*9, "d": []},
        "odometry": {"stamp_unix_ms": 1000000, "frame_id": "world", "child_frame_id": "base_link",
                     "pose": {"position": vector, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
                     "linear_velocity": vector, "angular_velocity": vector},
    }


def pipeline(mode="image_odom"):
    return ObservationPipeline(ObservationContract(5, 750, 80, "world", "base_link", "usb", "usb-v1",
                                                    observation_mode=mode), None, None, None, 500)


def test_image_odom_accepted_without_tf(payload, monkeypatch):
    monkeypatch.setattr("app.observation_pipeline.time.time", lambda: 1000)
    assert pipeline()._validate_observation(OnboardObservation(**payload)).startswith(b"\xff\xd8")


def test_both_sides_must_opt_in(payload, monkeypatch):
    monkeypatch.setattr("app.observation_pipeline.time.time", lambda: 1000)
    with pytest.raises(ValueError, match="mode does not match"):
        pipeline("calibrated")._validate_observation(OnboardObservation(**payload))


@pytest.mark.parametrize("mutation", ["calibrated", "claim", "fake_transform", "missing_odom", "invalid_mode"])
def test_invalid_schema_rejected(payload, mutation):
    if mutation == "calibrated":
        payload["observation_mode"] = "calibrated"
    elif mutation == "claim":
        payload["calibration_validated"] = True
    elif mutation == "fake_transform":
        payload["body_from_camera"] = {"parent_frame_id": "base_link", "child_frame_id": "usb",
                                      "translation": {"x": 0, "y": 0, "z": 0},
                                      "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}}
    elif mutation == "missing_odom":
        del payload["odometry"]
    else:
        payload["observation_mode"] = "anything"
    with pytest.raises(ValidationError):
        OnboardObservation(**payload)


@pytest.mark.parametrize("mutation,error", [("stale", "timestamp"), ("pair", "not time-synchronized"),
                                            ("world", "world frame"), ("body", "child frame"),
                                            ("camera", "image frame"), ("id", "calibration_id")])
def test_existing_guards_preserved(payload, monkeypatch, mutation, error):
    monkeypatch.setattr("app.observation_pipeline.time.time", lambda: 1000)
    if mutation == "stale":
        payload["capture_unix_ms"] -= 751
    elif mutation == "pair":
        payload["odometry"]["stamp_unix_ms"] -= 81
    elif mutation == "world":
        payload["odometry"]["frame_id"] = "wrong"
    elif mutation == "body":
        payload["odometry"]["child_frame_id"] = "wrong"
    elif mutation == "camera":
        payload["image_frame_id"] = "wrong"
    else:
        payload["calibration_id"] = "wrong"
    with pytest.raises(ValueError, match=error):
        pipeline()._validate_observation(OnboardObservation(**payload))
