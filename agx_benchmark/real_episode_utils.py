from __future__ import annotations

import json
import statistics
import subprocess
import threading
from pathlib import Path
from typing import Any

import numpy as np


ACTION_NAMES = ("dx_body_m", "dy_body_m", "dz_body_m", "d_yaw_rad")


def _summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(values)
    p95_index = min(len(ordered) - 1, int(0.95 * len(ordered)))
    return {
        "mean": round(statistics.fmean(values), 3),
        "p95": round(ordered[p95_index], 3),
        "max": round(max(values), 3),
    }


def _body_frame_action(current_raw: np.ndarray, next_raw: np.ndarray) -> np.ndarray:
    current = current_raw[[0, 1, 2, 4]].astype(np.float64, copy=True)
    following = next_raw[[0, 1, 2, 4]].astype(np.float64, copy=True)
    current[3] = np.deg2rad(current[3])
    following[3] = np.deg2rad(following[3])
    yaw = current[3]
    rotation = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    local_position = rotation.T @ (following[:3] - current[:3])
    delta_yaw = (following[3] - yaw + np.pi) % (2.0 * np.pi) - np.pi
    return np.array([*local_position, delta_yaw], dtype=np.float64)


def load_real_episode(episode_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    episode_dir = episode_dir.resolve()
    payload = json.loads((episode_dir / "log.json").read_text(encoding="utf-8"))
    image_paths = sorted((*episode_dir.glob("*.jpg"), *episode_dir.glob("*.jpeg"), *episode_dir.glob("*.png")))
    raw = np.asarray(payload["raw_logs"], dtype=np.float64)
    processed = np.asarray(payload["preprocessed_logs"], dtype=np.float64)
    expected = int(payload["length"])
    if len(image_paths) != expected or raw.shape[0] != expected or processed.shape[0] != expected:
        raise ValueError(
            f"Episode alignment mismatch: images={len(image_paths)}, raw={raw.shape[0]}, "
            f"processed={processed.shape[0]}, declared={expected}"
        )

    samples: list[dict[str, Any]] = []
    for index, image_path in enumerate(image_paths):
        state = processed[index, [0, 1, 2, 4]].astype(np.float32)
        target = np.zeros(4, dtype=np.float64)
        if index + 1 < expected:
            target = _body_frame_action(raw[index], raw[index + 1])
        samples.append(
            {
                "index": index,
                "image_path": image_path,
                "state": state,
                "target_action": target,
            }
        )

    metadata = {
        "episode_id": payload.get("id", episode_dir.name),
        "episode_dir": str(episode_dir),
        "instruction": payload["instruction"],
        "instruction_unified": payload.get("instruction_unified"),
        "frames": expected,
        "state_semantics": ["x", "y", "z", "yaw_deg"],
        "action_semantics": list(ACTION_NAMES),
    }
    return metadata, samples


def openvla_prompt(state: np.ndarray, instruction: str) -> str:
    proprio = ",".join(str(round(float(value), 1)) for value in state)
    return f"In: Current State: {proprio}, What action should the uav take to {instruction}?\nOut:"


def action_error_metrics(predictions: np.ndarray, targets: np.ndarray) -> dict[str, Any]:
    if predictions.shape != targets.shape or predictions.ndim != 2 or predictions.shape[1] != 4:
        raise ValueError(f"Expected matching [N,4] arrays, got {predictions.shape} and {targets.shape}")
    errors = predictions - targets
    absolute = np.abs(errors)
    squared = errors**2
    l2 = np.linalg.norm(errors, axis=1)
    return {
        "samples": int(predictions.shape[0]),
        "mae_per_dimension": {name: round(float(value), 9) for name, value in zip(ACTION_NAMES, absolute.mean(axis=0))},
        "rmse_per_dimension": {
            name: round(float(value), 9) for name, value in zip(ACTION_NAMES, np.sqrt(squared.mean(axis=0)))
        },
        "mean_l2": round(float(l2.mean()), 9),
        "p95_l2": round(float(np.percentile(l2, 95)), 9),
        "max_l2": round(float(l2.max()), 9),
    }


class NvidiaSmiSampler:
    """Sample discrete-GPU utilization, board power, and total used memory."""

    def __init__(self, interval_ms: int = 100) -> None:
        self.interval_ms = interval_ms
        self.lines: list[str] = []
        self._process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None

    @staticmethod
    def snapshot() -> dict[str, float] | None:
        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,power.draw",
                    "--format=csv,noheader,nounits",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
            values = [float(value.strip()) for value in result.stdout.splitlines()[0].split(",")]
            return {"gpu_util_pct": values[0], "memory_used_mb": values[1], "power_draw_w": values[2]}
        except (FileNotFoundError, IndexError, ValueError, subprocess.SubprocessError):
            return None

    def start(self) -> None:
        try:
            self._process = subprocess.Popen(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,power.draw",
                    "--format=csv,noheader,nounits",
                    "-lms",
                    str(self.interval_ms),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError:
            return

        def consume() -> None:
            assert self._process is not None and self._process.stdout is not None
            for line in self._process.stdout:
                self.lines.append(line.strip())

        self._thread = threading.Thread(target=consume, daemon=True)
        self._thread.start()

    def stop(self) -> dict[str, Any] | None:
        if self._process is None:
            return None
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=3)
        if self._thread is not None:
            self._thread.join(timeout=1)

        utilization: list[float] = []
        memory: list[float] = []
        power: list[float] = []
        for line in self.lines:
            try:
                values = [float(value.strip()) for value in line.split(",")]
                utilization.append(values[0])
                memory.append(values[1])
                power.append(values[2])
            except (IndexError, ValueError):
                continue
        return {
            "sample_count": len(utilization),
            "gpu_util_pct": _summary(utilization),
            "memory_used_mb": _summary(memory),
            "power_draw_w": _summary(power),
        }
