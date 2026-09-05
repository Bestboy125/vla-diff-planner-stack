import asyncio
import base64
from io import BytesIO
import json
import threading
import time
from uuid import uuid4

from fastapi.testclient import TestClient
import numpy as np
from PIL import Image
from websockets.sync.server import serve

from app.main import app, mission_manager, model_gateway, task_dispatcher
from app.model_gateway import ModelGateway, _packb, _unpackb
from app.onboard_bridge import OnboardBridgeClient
from app.mission import MissionManager
from app.observation_pipeline import ObservationContract, ObservationPipeline, body_delta_to_world_target
from app.schemas import InferenceRequest, MissionCreate, OnboardObservation


client = TestClient(app)


def reset_manager() -> None:
    mission_manager._mission = None
    mission_manager.control_output_enabled = False


def test_health_is_available_when_models_are_offline() -> None:
    reset_manager()
    response = client.get("/api/health")
    assert response.status_code == 200
    payload = response.json()
    assert payload["backend"] == "online"
    assert payload["safety_lock"] is True
    assert set(payload["models"]) == {"openvla", "pi05"}


def test_dry_run_mission_lifecycle() -> None:
    reset_manager()
    created = client.post(
        "/api/missions",
        json={"instruction": "Fly forward and keep clear of obstacles", "policy": "openvla", "mode": "dry_run"},
    )
    assert created.status_code == 201
    mission = created.json()["mission"]
    mission_id = mission["mission_id"]
    assert mission["state"] == "ARMED"

    started = client.post(f"/api/missions/{mission_id}/start")
    assert started.status_code == 200
    assert started.json()["mission"]["state"] == "RUNNING"

    held = client.post(f"/api/missions/{mission_id}/hold")
    assert held.status_code == 200
    assert held.json()["mission"]["state"] == "HOLDING"

    stopped = client.post(f"/api/missions/{mission_id}/stop")
    assert stopped.status_code == 200
    assert stopped.json()["mission"]["state"] == "ABORTED"


def test_live_mission_is_safety_locked() -> None:
    reset_manager()
    created = client.post(
        "/api/missions",
        json={"instruction": "Approach the marked target", "policy": "openvla", "mode": "live"},
    )
    mission_id = created.json()["mission"]["mission_id"]
    response = client.post(f"/api/missions/{mission_id}/start")
    assert response.status_code == 423


def test_atomic_task_dry_run_is_validated_without_onboard_delivery(monkeypatch) -> None:
    async def forbidden_send(_command):
        raise AssertionError("dry-run atomic tasks must not contact the onboard bridge")

    monkeypatch.setattr(task_dispatcher.onboard_bridge, "send", forbidden_send)
    response = client.post(
        "/api/tasks/dispatch",
        json={
            "category": "atomic",
            "atomic_task": "move_forward",
            "mode": "dry_run",
            "parameters": {"distance_m": 0.4},
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["delivery"]["status"] == "safety_locked"
    assert payload["command"]["type"] == "operator_task"
    assert payload["command"]["command"] == "MOVE_FORWARD"
    assert payload["command"]["magnitude"] == 0.4


def test_atomic_live_task_is_rejected_by_host_safety_lock() -> None:
    response = client.post(
        "/api/tasks/dispatch",
        json={
            "category": "atomic",
            "atomic_task": "yaw_left",
            "mode": "live",
            "live_confirmation": "I_ACCEPT_REAL_FLIGHT_CONTROL",
            "parameters": {"yaw_deg": 20},
        },
        headers={"X-Operator-Token": "not-configured"},
    )
    assert response.status_code == 423


def test_world_orbit_atomic_task_builds_structured_locked_command(monkeypatch) -> None:
    async def forbidden_send(_command):
        raise AssertionError("dry-run orbit must not contact the onboard bridge")
    monkeypatch.setattr(task_dispatcher.onboard_bridge, "send", forbidden_send)
    response = client.post("/api/tasks/dispatch", json={
        "category": "atomic", "atomic_task": "orbit_world", "mode": "dry_run",
        "parameters": {"center_x_m": 2.0, "center_y_m": -1.0, "center_z_m": 1.2,
                       "radius_m": 1.5, "laps": 0.5, "orbit_direction": "counterclockwise"},
    })
    assert response.status_code == 200
    payload = response.json()
    assert payload["delivery"]["status"] == "safety_locked"
    assert payload["command"]["command"] == "ORBIT_WORLD"
    assert payload["command"]["orbit"] == {
        "center": [2.0, -1.0, 1.2], "radius_m": 1.5, "laps": 0.5,
        "direction": "counterclockwise", "yaw_mode": "face_center",
    }


def test_orbit_template_creates_running_dry_run_vla_mission() -> None:
    reset_manager()
    response = client.post(
        "/api/tasks/dispatch",
        json={
            "category": "embodied",
            "embodied_task": "orbit_target",
            "policy": "openvla",
            "mode": "dry_run",
            "parameters": {
                "target_label": "电线杆",
                "radius_m": 1.5,
                "laps": 1,
                "orbit_direction": "clockwise",
            },
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["mission"]["state"] == "RUNNING"
    assert "电线杆" in payload["normalized_instruction"]
    assert "1.50米" in payload["normalized_instruction"]


def test_proprio_requires_four_values() -> None:
    response = client.post(
        "/api/inference/openvla",
        json={"image_base64": "a" * 32, "instruction": "Fly forward", "proprio": [0, 0, 0]},
    )
    assert response.status_code == 422


def test_openpi_msgpack_numpy_roundtrip() -> None:
    original = {
        "observation/state": np.asarray([1, 2, 3, 4], dtype=np.float32),
        "observation/image": np.arange(18, dtype=np.uint8).reshape(2, 3, 3),
        "prompt": "fly forward",
    }
    restored = _unpackb(_packb(original))
    np.testing.assert_array_equal(restored["observation/state"], original["observation/state"])
    np.testing.assert_array_equal(restored["observation/image"], original["observation/image"])
    assert restored["prompt"] == original["prompt"]


def test_pi05_route_uses_common_action_contract(monkeypatch) -> None:
    async def fake_predict(_request):
        return {
            "action_local_delta": [[0.1, 0.0, 0.0, 0.01]],
            "target_mission": [[0.1, 0.0, 0.0, 0.01]],
            "action_chunk": [[0.1, 0.0, 0.0, 0.01]],
            "action_semantic": ["dx_body", "dy_body", "dz_body", "d_yaw"],
            "action_units": ["m", "m", "m", "rad"],
            "server_timing": {"infer_ms": 100.0},
        }

    monkeypatch.setattr(model_gateway, "predict_pi05", fake_predict)
    response = client.post(
        "/api/inference/pi05",
        json={"image_base64": "a" * 32, "instruction": "Fly forward", "proprio": [0, 0, 0, 0]},
    )
    assert response.status_code == 200
    assert response.json()["action_local_delta"] == [[0.1, 0.0, 0.0, 0.01]]


def test_pi05_openpi_websocket_protocol() -> None:
    received = {}

    def handler(socket) -> None:
        socket.send(_packb({"policy": "fake-pi05"}))
        received.update(_unpackb(socket.recv()))
        actions = np.tile(np.asarray([[0.1, 0.0, -0.02, 0.01]], dtype=np.float32), (10, 1))
        socket.send(_packb({"actions": actions, "server_timing": {"infer_ms": 12.5}}))

    image_buffer = BytesIO()
    Image.new("RGB", (8, 8), color=(10, 20, 30)).save(image_buffer, format="PNG")
    request = InferenceRequest(
        image_base64=base64.b64encode(image_buffer.getvalue()).decode("ascii"),
        instruction="Fly forward",
        proprio=[1.0, 2.0, 3.0, 90.0],
    )

    with serve(handler, "127.0.0.1", 0) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        port = server.socket.getsockname()[1]
        gateway = ModelGateway("http://127.0.0.1:1", "127.0.0.1", port)
        result = asyncio.run(gateway.predict_pi05(request))
        server.shutdown()
        thread.join(timeout=2)

    assert received["prompt"] == "Fly forward"
    assert received["observation/state"].shape == (4,)
    assert received["observation/image"].shape == (8, 8, 3)
    assert len(result["action_chunk"]) == 10
    assert result["action_local_delta"] == [[0.10000000149011612, 0.0, -0.019999999552965164, 0.009999999776482582]]
    np.testing.assert_allclose(result["target_mission"][0], [1.0, 2.1, 2.98, np.pi / 2 + 0.01])


def test_bridge_track_command_matches_onboard_contract_without_delivery() -> None:
    reset_manager()
    created = client.post(
        "/api/missions",
        json={"instruction": "Move forward", "policy": "openvla", "mode": "dry_run"},
    )
    mission_id = created.json()["mission"]["mission_id"]
    assert client.post(f"/api/missions/{mission_id}/start").status_code == 200
    response = client.post(
        "/api/bridge/commands",
        json={
            "mission_id": mission_id,
            "sequence": 7,
            "policy": "openvla",
            "command": "TRACK",
            "action_local_delta": [[0.2, 0.0, 0.0, 0.01]],
            "target_mission": [[1.2, 2.0, 1.0, 0.51]],
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["delivery"]["status"] == "safety_locked"
    assert payload["command"]["type"] == "trajectory_command"
    assert payload["command"]["action_units"] == ["m", "m", "m", "rad"]
    assert payload["command"]["target_mission"] == [[1.2, 2.0, 1.0, 0.51]]
    assert "auth_token" not in payload["command"]


def test_bridge_rejects_track_without_model_output() -> None:
    response = client.post(
        "/api/bridge/commands",
        json={
            "mission_id": str(uuid4()),
            "sequence": 1,
            "policy": "pi05",
            "command": "TRACK",
        },
    )
    assert response.status_code == 422


def test_bridge_rejects_command_for_unknown_mission() -> None:
    reset_manager()
    response = client.post(
        "/api/bridge/commands",
        json={
            "mission_id": str(uuid4()),
            "sequence": 1,
            "policy": "pi05",
            "command": "HOLD",
        },
    )
    assert response.status_code == 404


def test_onboard_tcp_client_matches_ack_to_command() -> None:
    async def scenario() -> None:
        received = {}

        async def handler(reader, writer) -> None:
            received.update(json.loads(await reader.readline()))
            ack = {
                "schema_version": 1,
                "type": "trajectory_ack",
                "mission_id": received["mission_id"],
                "sequence": received["sequence"],
                "status": "preview",
                "reason": "test",
            }
            writer.write(json.dumps(ack).encode("utf-8") + b"\n")
            await writer.drain()
            writer.close()
            await writer.wait_closed()

        server = await asyncio.start_server(handler, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        bridge = OnboardBridgeClient("127.0.0.1", port, "unit-test-token")
        command = {
            "schema_version": 1,
            "type": "trajectory_command",
            "mission_id": str(uuid4()),
            "sequence": 9,
        }
        ack = await bridge.send(command)
        server.close()
        await server.wait_closed()
        assert ack["status"] == "preview"
        assert received["auth_token"] == "unit-test-token"

    asyncio.run(scenario())


def test_flu_body_delta_rotates_into_enu_world() -> None:
    target = body_delta_to_world_target((1.0, 2.0, 3.0), np.pi / 2, [[0.4, 0.2, -0.1, 0.05]])
    np.testing.assert_allclose(target[0], [0.8, 2.4, 2.9, np.pi / 2 + 0.05], atol=1e-8)


def test_preview_launch_has_no_flight_control_topic() -> None:
    root = __import__("pathlib").Path(__file__).resolve().parents[2]
    launch = (
        root.parent
        / "Diff-Planner"
        / "src"
        / "integration"
        / "vla_diff_bridge"
        / "launch"
        / "vla_fastlio_diff_preview_stack.launch"
    ).read_text(encoding="utf-8")
    assert "/vla/optimized_trajectory_preview" in launch
    assert "/setpoints_cmd" not in launch
    assert "mavros" not in launch.lower()
    assert "px4ctrl" not in launch.lower()


def test_k_frame_pipeline_selects_only_every_kth_frame_and_sends_preview() -> None:
    class FakeGateway:
        calls = 0

        async def predict_openvla(self, _request):
            self.calls += 1
            return {
                "action_local_delta": [[0.2, 0.0, 0.0, 0.1]],
                "action_chunk": [
                    [0.2, 0.0, 0.0, 0.1],
                    [0.2, 0.0, 0.0, 0.0],
                    [0.2, 0.0, 0.0, 0.0],
                ],
            }

    class FakeBridge:
        sent = []

        async def send(self, command):
            self.sent.append(command)
            return {
                "type": "planning_preview_ack",
                "mission_id": command["mission_id"],
                "sequence": command["sequence"],
                "status": "preview_published",
            }

    async def scenario() -> None:
        manager = MissionManager(control_output_enabled=False)
        mission = await manager.create(MissionCreate(instruction="fly forward", mode="dry_run"))
        await manager.start(mission.mission_id)
        gateway = FakeGateway()
        bridge = FakeBridge()
        pipeline = ObservationPipeline(
            ObservationContract(
                k_frames=2,
                max_age_ms=750,
                max_sync_error_ms=80,
                world_frame="world",
                body_frame="base_link",
                camera_frame="camera_color_optical_frame",
                calibration_id="cal-test",
                max_future_skew_ms=5000,
            ),
            manager,
            gateway,
            bridge,
            500,
        )
        image_buffer = BytesIO()
        Image.new("RGB", (8, 8), color=(1, 2, 3)).save(image_buffer, format="JPEG")
        # The onboard clock can lead the Windows host slightly; the configured
        # skew budget should accept such fresh, internally synchronized samples.
        now_ms = int(time.time() * 1000) + 3000
        base = {
            "vehicle_id": "uav0",
            "capture_unix_ms": now_ms,
            "image_base64": base64.b64encode(image_buffer.getvalue()).decode("ascii"),
            "image_frame_id": "camera_color_optical_frame",
            "odometry": {
                "stamp_unix_ms": now_ms,
                "frame_id": "world",
                "child_frame_id": "base_link",
                "pose": {
                    "position": {"x": 1.0, "y": 2.0, "z": 1.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "camera_intrinsics": {
                "width": 8,
                "height": 8,
                "distortion_model": "plumb_bob",
                "k": [1.0, 0.0, 4.0, 0.0, 1.0, 4.0, 0.0, 0.0, 1.0],
                "d": [],
            },
            "body_from_camera": {
                "parent_frame_id": "base_link",
                "child_frame_id": "camera_color_optical_frame",
                "translation": {"x": 0.1, "y": 0.0, "z": 0.0},
                "rotation": {"x": 0.5, "y": -0.5, "z": 0.5, "w": -0.5},
            },
            "calibration_id": "cal-test",
            "calibration_validated": True,
        }
        first = await pipeline.ingest(OnboardObservation(sequence=10, **base))
        second = await pipeline.ingest(OnboardObservation(sequence=11, **base))
        assert first["selected_for_inference"] is False
        assert second["selected_for_inference"] is True
        for _ in range(20):
            if bridge.sent:
                break
            await asyncio.sleep(0.01)
        assert gateway.calls == 1
        assert len(bridge.sent) == 1
        assert bridge.sent[0]["type"] == "planning_preview"
        np.testing.assert_allclose(bridge.sent[0]["target_mission"], [[1.2, 2.0, 1.0, 0.1]])
        assert len(bridge.sent[0]["action_chunk"]) == 3

    asyncio.run(scenario())


def test_live_k_frame_pipeline_builds_trajectory_command_without_real_delivery() -> None:
    class FakeGateway:
        async def predict_openvla(self, _request):
            return {"action_local_delta": [[0.15, 0.0, 0.0, 0.02]]}

    class FakeBridge:
        sent = []

        async def send(self, command):
            self.sent.append(command)
            return {
                "type": "trajectory_ack",
                "mission_id": command["mission_id"],
                "sequence": command["sequence"],
                "status": "unit_test_only",
            }

    async def scenario() -> None:
        manager = MissionManager(control_output_enabled=True)
        mission = await manager.create(
            MissionCreate(instruction="fly forward", mode="live")
        )
        await manager.start(mission.mission_id)
        bridge = FakeBridge()
        pipeline = ObservationPipeline(
            ObservationContract(
                k_frames=1,
                max_age_ms=750,
                max_sync_error_ms=80,
                world_frame="world",
                body_frame="base_link",
                camera_frame="camera_color_optical_frame",
                calibration_id="cal-test",
                max_future_skew_ms=5000,
            ),
            manager,
            FakeGateway(),
            bridge,
            500,
        )
        image_buffer = BytesIO()
        Image.new("RGB", (8, 8), color=(1, 2, 3)).save(image_buffer, format="JPEG")
        now_ms = int(time.time() * 1000)
        observation = OnboardObservation(
            sequence=1,
            vehicle_id="uav0",
            capture_unix_ms=now_ms,
            image_base64=base64.b64encode(image_buffer.getvalue()).decode("ascii"),
            image_frame_id="camera_color_optical_frame",
            odometry={
                "stamp_unix_ms": now_ms,
                "frame_id": "world",
                "child_frame_id": "base_link",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 1.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            camera_intrinsics={
                "width": 8,
                "height": 8,
                "distortion_model": "plumb_bob",
                "k": [1.0, 0.0, 4.0, 0.0, 1.0, 4.0, 0.0, 0.0, 1.0],
                "d": [],
            },
            body_from_camera={
                "parent_frame_id": "base_link",
                "child_frame_id": "camera_color_optical_frame",
                "translation": {"x": 0.1, "y": 0.0, "z": 0.0},
                "rotation": {"x": 0.5, "y": -0.5, "z": 0.5, "w": -0.5},
            },
            calibration_id="cal-test",
            calibration_validated=True,
        )
        await pipeline.ingest(observation)
        for _ in range(20):
            if bridge.sent:
                break
            await asyncio.sleep(0.01)
        assert len(bridge.sent) == 1
        assert bridge.sent[0]["type"] == "trajectory_command"
        assert bridge.sent[0]["command"] == "TRACK"

    asyncio.run(scenario())
