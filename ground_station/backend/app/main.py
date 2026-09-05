import asyncio
from pathlib import Path
from uuid import UUID

from fastapi import FastAPI, Header, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .config import settings
from .mission import MissionManager
from .model_gateway import ModelGateway
from .observation_pipeline import ObservationContract, ObservationPipeline
from .onboard_bridge import OnboardBridgeClient, build_trajectory_command
from .schemas import (
    BridgeCommandName,
    BridgeCommandRequest,
    InferenceRequest,
    MissionCreate,
    MissionMode,
    MissionState,
    OnboardObservation,
    ObservationInferenceRequest,
    TaskDispatchRequest,
)
from .task_dispatch import TaskDispatcher


app = FastAPI(title="VLA Ground Station", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        f"http://{settings.host_operator_ip}:5173",
    ],
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type", "X-Operator-Token"],
)

mission_manager = MissionManager(control_output_enabled=settings.control_output_enabled)
model_gateway = ModelGateway(settings.openvla_url, settings.pi05_host, settings.pi05_port)
onboard_bridge = OnboardBridgeClient(
    settings.onboard_bridge_host,
    settings.onboard_bridge_port,
    settings.onboard_bridge_token,
)
observation_pipeline = ObservationPipeline(
    ObservationContract(
        k_frames=settings.observation_k_frames,
        max_age_ms=settings.observation_max_age_ms,
        max_future_skew_ms=settings.observation_max_future_skew_ms,
        max_sync_error_ms=settings.observation_max_sync_error_ms,
        world_frame=settings.expected_world_frame,
        body_frame=settings.expected_body_frame,
        camera_frame=settings.expected_camera_frame,
        calibration_id=settings.expected_calibration_id,
        observation_mode=settings.observation_mode,
    ),
    mission_manager,
    model_gateway,
    onboard_bridge,
    settings.onboard_command_ttl_ms,
)
task_dispatcher = TaskDispatcher(
    mission_manager,
    onboard_bridge,
    control_output_enabled=settings.control_output_enabled,
    operator_control_token=settings.operator_control_token,
    live_control_confirmation=settings.live_control_confirmation,
    command_ttl_ms=settings.onboard_command_ttl_ms,
    world_frame=settings.expected_world_frame,
    body_frame=settings.expected_body_frame,
)


async def system_snapshot(include_models: bool = True) -> dict:
    mission = await mission_manager.current()
    payload = {
        "backend": "online",
        "control_output_enabled": settings.control_output_enabled,
        "safety_lock": not settings.control_output_enabled,
        "host_interfaces": {
            "onboard_lan": settings.host_onboard_ip,
            "operator_lan": settings.host_operator_ip,
        },
        "mission": mission.model_dump(mode="json") if mission else None,
        "onboard_observation": await observation_pipeline.snapshot(),
        "task_runtime": await task_dispatcher.snapshot(),
    }
    if include_models:
        openvla, pi05 = await asyncio.gather(
            model_gateway.openvla_status(), model_gateway.pi05_status()
        )
        payload["models"] = {
            "openvla": openvla.model_dump(),
            "pi05": pi05.model_dump(),
        }
    return payload


@app.get("/api/health")
async def health() -> dict:
    return await system_snapshot(include_models=True)


@app.get("/api/missions/current")
async def current_mission() -> dict:
    mission = await mission_manager.current()
    return {"mission": mission.model_dump(mode="json") if mission else None}


@app.post("/api/missions", status_code=201)
async def create_mission(request: MissionCreate) -> dict:
    mission = await mission_manager.create(request)
    return {"mission": mission.model_dump(mode="json")}


@app.post("/api/missions/{mission_id}/start")
async def start_mission(mission_id: UUID) -> dict:
    mission = await mission_manager.start(mission_id)
    return {"mission": mission.model_dump(mode="json")}


@app.post("/api/missions/{mission_id}/hold")
async def hold_mission(mission_id: UUID) -> dict:
    mission = await mission_manager.hold(mission_id)
    return {"mission": mission.model_dump(mode="json")}


@app.post("/api/missions/{mission_id}/stop")
async def stop_mission(mission_id: UUID) -> dict:
    mission = await mission_manager.stop(mission_id)
    return {"mission": mission.model_dump(mode="json")}


@app.post("/api/inference/latest-observation")
async def latest_observation_inference(request: ObservationInferenceRequest) -> dict:
    try:
        return await observation_pipeline.infer_latest_no_motion(request.instruction, request.policy)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=503, detail="Model diagnostic failed: {}".format(type(exc).__name__)) from exc


@app.post("/api/inference/openvla")
async def openvla_inference(request: InferenceRequest) -> dict:
    try:
        result = await model_gateway.predict_openvla(request)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {
        "action_local_delta": result["action_local_delta"],
        "target_mission": result["target_mission"],
        "action_semantic": result["action_semantic"],
        "action_units": result["action_units"],
    }


@app.post("/api/inference/pi05")
async def pi05_inference(request: InferenceRequest) -> dict:
    try:
        return await model_gateway.predict_pi05(request)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc


@app.post("/api/onboard/observations", status_code=202)
async def onboard_observation(
    observation: OnboardObservation,
    x_observation_token: str | None = Header(default=None),
) -> dict:
    if (
        settings.onboard_observation_token == "REQUIRED"
        or x_observation_token != settings.onboard_observation_token
    ):
        raise HTTPException(status_code=401, detail="invalid onboard observation token")
    try:
        return await observation_pipeline.ingest(observation)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/onboard/latest/image")
async def latest_onboard_image() -> Response:
    latest = await observation_pipeline.latest_jpeg()
    if latest is None:
        raise HTTPException(status_code=404, detail="no onboard image has been accepted")
    jpeg, sequence = latest
    return Response(
        content=jpeg,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store", "X-Image-Sequence": str(sequence)},
    )


@app.get("/api/onboard/stream.mjpeg")
async def onboard_video_stream() -> StreamingResponse:
    async def frames():
        last_sequence = -1
        interval = 1.0 / max(settings.video_stream_hz, 1.0)
        while True:
            latest = await observation_pipeline.latest_jpeg()
            if latest is not None:
                jpeg, sequence = latest
                if sequence != last_sequence:
                    last_sequence = sequence
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"X-Image-Sequence: {sequence}\r\n".encode("ascii")
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                        + jpeg
                        + b"\r\n"
                    )
            await asyncio.sleep(interval)

    return StreamingResponse(
        frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.get("/api/tasks/catalog")
async def task_catalog() -> dict:
    return task_dispatcher.catalog()


@app.post("/api/tasks/dispatch")
async def dispatch_task(
    request: TaskDispatchRequest,
    x_operator_token: str | None = Header(default=None),
) -> dict:
    return await task_dispatcher.dispatch(request, x_operator_token)


@app.post("/api/bridge/commands")
async def bridge_command(request: BridgeCommandRequest) -> dict:
    mission = await mission_manager.current()
    if mission is None or mission.mission_id != request.mission_id:
        raise HTTPException(status_code=404, detail="Bridge command does not match the current mission.")
    if mission.policy != request.policy:
        raise HTTPException(status_code=409, detail="Bridge policy does not match the current mission.")
    allowed_states = {
        BridgeCommandName.TRACK: {MissionState.RUNNING},
        BridgeCommandName.HOLD: {MissionState.RUNNING, MissionState.HOLDING},
        BridgeCommandName.COMPLETE: {MissionState.RUNNING, MissionState.HOLDING},
        BridgeCommandName.EMERGENCY_STOP: {
            MissionState.ARMED,
            MissionState.RUNNING,
            MissionState.HOLDING,
        },
    }
    if mission.state not in allowed_states[request.command]:
        raise HTTPException(
            status_code=409,
            detail=f"{request.command.value} is not allowed from mission state {mission.state.value}.",
        )
    try:
        command = build_trajectory_command(request, settings.onboard_command_ttl_ms)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    if not settings.control_output_enabled or mission.mode != MissionMode.LIVE:
        reason = (
            "CONTROL_OUTPUT_ENABLED=false"
            if not settings.control_output_enabled
            else "the current mission is dry_run"
        )
        return {
            "command": command,
            "delivery": {
                "status": "safety_locked",
                "detail": f"Command validated but not sent because {reason}.",
            },
        }
    try:
        ack = await onboard_bridge.send(command)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"command": command, "delivery": ack}


@app.websocket("/ws/status")
async def status_socket(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(await system_snapshot(include_models=True))
            await asyncio.sleep(1.0 / max(settings.status_update_hz, 1.0))
    except WebSocketDisconnect:
        return


frontend_dist: Path = settings.frontend_dist
assets_dir = frontend_dist / "assets"
if assets_dir.is_dir():
    app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")


@app.get("/{full_path:path}", include_in_schema=False)
async def frontend(full_path: str):
    if full_path.startswith("api/") or full_path.startswith("ws/"):
        raise HTTPException(status_code=404, detail="Not found")
    index = frontend_dist / "index.html"
    if index.is_file():
        return FileResponse(index)
    raise HTTPException(status_code=404, detail="Frontend has not been built yet.")
