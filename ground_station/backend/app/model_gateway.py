import asyncio
import base64
from io import BytesIO
import math
import time
from typing import Any

import httpx
import msgpack
import numpy as np
from PIL import Image
import websockets

from .schemas import InferenceRequest, ServiceState


class ModelGateway:
    def __init__(self, openvla_url: str, pi05_host: str, pi05_port: int) -> None:
        self.openvla_url = openvla_url.rstrip("/")
        self.pi05_host = pi05_host
        self.pi05_port = pi05_port

    async def openvla_status(self) -> ServiceState:
        started = time.perf_counter()
        try:
            async with httpx.AsyncClient(timeout=0.8) as client:
                response = await client.get(f"{self.openvla_url}/")
                response.raise_for_status()
                payload = response.json()
            return ServiceState(
                status="online",
                detail=f"{payload.get('service', 'OpenVLA')} · {payload.get('unnorm_key', 'unknown')}",
                latency_ms=round((time.perf_counter() - started) * 1000, 2),
            )
        except Exception as exc:
            return ServiceState(status="offline", detail=f"OpenVLA unavailable: {type(exc).__name__}")

    async def pi05_status(self) -> ServiceState:
        started = time.perf_counter()
        try:
            async with httpx.AsyncClient(timeout=0.8) as client:
                response = await client.get(f"http://{self.pi05_host}:{self.pi05_port}/healthz")
                response.raise_for_status()
            return ServiceState(
                status="online",
                detail="π0.5 OpenPI policy server is healthy.",
                latency_ms=round((time.perf_counter() - started) * 1000, 2),
            )
        except Exception as exc:
            return ServiceState(status="offline", detail=f"π0.5 unavailable: {type(exc).__name__}")

    async def predict_openvla(self, request: InferenceRequest) -> dict[str, Any]:
        payload = {
            "image": request.image_base64,
            "instr": request.instruction,
            "proprio": request.proprio,
        }
        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(f"{self.openvla_url}/predict", json=payload)
                response.raise_for_status()
                result = response.json()
        except httpx.HTTPError as exc:
            raise RuntimeError(f"OpenVLA request failed: {exc}") from exc
        if result.get("status") != "success":
            raise RuntimeError(result.get("message", "OpenVLA returned an error"))
        return result

    async def predict_pi05(self, request: InferenceRequest) -> dict[str, Any]:
        try:
            image = np.asarray(
                Image.open(BytesIO(base64.b64decode(request.image_base64, validate=True))).convert("RGB"),
                dtype=np.uint8,
            )
        except Exception as exc:
            raise RuntimeError(f"Invalid RGB image: {exc}") from exc

        observation = {
            "observation/state": np.asarray(request.proprio, dtype=np.float32),
            "observation/image": image,
            "prompt": request.instruction,
        }
        uri = f"ws://{self.pi05_host}:{self.pi05_port}"
        try:
            async with websockets.connect(uri, compression=None, max_size=None, open_timeout=3.0) as socket:
                # The OpenPI server sends model metadata immediately after the handshake.
                await asyncio.wait_for(socket.recv(), timeout=3.0)
                await socket.send(_packb(observation))
                response = await asyncio.wait_for(socket.recv(), timeout=30.0)
        except Exception as exc:
            raise RuntimeError(f"π0.5 request failed: {type(exc).__name__}: {exc}") from exc

        if isinstance(response, str):
            raise RuntimeError(f"π0.5 server returned an error: {response}")
        result = _unpackb(response)
        actions = np.asarray(result.get("actions"), dtype=np.float32)
        if actions.ndim != 2 or actions.shape[0] < 1 or actions.shape[1] != 4:
            raise RuntimeError(f"π0.5 actions must have shape [horizon, 4], got {actions.shape}")
        if not np.isfinite(actions).all():
            raise RuntimeError("π0.5 returned non-finite actions")

        local_delta = actions[0]
        x, y, z, yaw_deg = request.proprio
        yaw_rad = math.radians(yaw_deg)
        dx, dy, dz, d_yaw = (float(value) for value in local_delta)
        target = [
            x + math.cos(yaw_rad) * dx - math.sin(yaw_rad) * dy,
            y + math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy,
            z + dz,
            yaw_rad + d_yaw,
        ]
        return {
            "action_local_delta": [local_delta.tolist()],
            "target_mission": [target],
            "action_chunk": actions.tolist(),
            "action_semantic": ["dx_body", "dy_body", "dz_body", "d_yaw"],
            "action_units": ["m", "m", "m", "rad"],
            "server_timing": result.get("server_timing", {}),
        }


def _pack_array(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        if value.dtype.kind in ("V", "O", "c"):
            raise ValueError(f"Unsupported ndarray dtype: {value.dtype}")
        return {
            b"__ndarray__": True,
            b"data": value.tobytes(),
            b"dtype": value.dtype.str,
            b"shape": value.shape,
        }
    if isinstance(value, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": value.item(),
            b"dtype": value.dtype.str,
        }
    return value


def _unpack_array(value: dict) -> Any:
    if b"__ndarray__" in value:
        return np.ndarray(
            buffer=value[b"data"],
            dtype=np.dtype(value[b"dtype"]),
            shape=value[b"shape"],
        )
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


def _packb(value: Any) -> bytes:
    return msgpack.packb(value, default=_pack_array)


def _unpackb(value: bytes) -> Any:
    return msgpack.unpackb(value, object_hook=_unpack_array)
