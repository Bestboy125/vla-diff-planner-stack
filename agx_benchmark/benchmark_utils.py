from __future__ import annotations

import json
import os
import platform
import re
import resource
import statistics
import subprocess
import threading
import time
from pathlib import Path
from typing import Any


class TegrastatsSampler:
    """Collect Jetson-wide RAM, GPU utilization, and power samples."""

    def __init__(self, interval_ms: int = 100) -> None:
        self.interval_ms = interval_ms
        self.lines: list[str] = []
        self._process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        try:
            self._process = subprocess.Popen(
                ["tegrastats", "--interval", str(self.interval_ms)],
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

    def stop(self) -> dict[str, Any]:
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=3)
        if self._thread is not None:
            self._thread.join(timeout=1)
        return summarize_tegrastats(self.lines)


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


def summarize_tegrastats(lines: list[str]) -> dict[str, Any]:
    ram_used_mb: list[float] = []
    gpu_util_pct: list[float] = []
    gpu_power_mw: list[float] = []
    module_power_mw: list[float] = []
    for line in lines:
        if match := re.search(r"RAM\s+(\d+)/(\d+)MB", line):
            ram_used_mb.append(float(match.group(1)))
        if match := re.search(r"GR3D_FREQ\s+(\d+)%", line):
            gpu_util_pct.append(float(match.group(1)))
        if match := re.search(r"VDD_GPU_SOC\s+(\d+)mW", line):
            gpu_power_mw.append(float(match.group(1)))
        if match := re.search(r"VDD_IN\s+(\d+)mW", line):
            module_power_mw.append(float(match.group(1)))
    return {
        "sample_count": len(lines),
        "ram_used_mb": _summary(ram_used_mb),
        "gpu_util_pct": _summary(gpu_util_pct),
        "gpu_soc_power_mw": _summary(gpu_power_mw),
        "module_power_mw": _summary(module_power_mw),
        "last_sample": lines[-1] if lines else None,
    }


def latency_summary(values_ms: list[float]) -> dict[str, float]:
    ordered = sorted(values_ms)
    p50 = statistics.median(ordered)
    p95_index = min(len(ordered) - 1, int(0.95 * len(ordered)))
    return {
        "runs": len(values_ms),
        "mean_ms": round(statistics.fmean(values_ms), 3),
        "p50_ms": round(p50, 3),
        "p95_ms": round(ordered[p95_index], 3),
        "min_ms": round(min(values_ms), 3),
        "max_ms": round(max(values_ms), 3),
        "throughput_hz": round(1000.0 / statistics.fmean(values_ms), 4),
    }


def process_memory() -> dict[str, float]:
    peak_rss_kib = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    status: dict[str, float] = {"peak_rss_gib": round(peak_rss_kib / 2**20, 3)}
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                status["rss_gib"] = round(float(line.split()[1]) / 2**20, 3)
            elif line.startswith("VmSwap:"):
                status["swap_gib"] = round(float(line.split()[1]) / 2**20, 3)
    except OSError:
        pass
    return status


def base_report(model: str, model_path: Path) -> dict[str, Any]:
    return {
        "model": model,
        "model_path": str(model_path.resolve()),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": platform.node(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "environment": {
            "CONDA_PREFIX": os.environ.get("CONDA_PREFIX"),
            "CUDA_VISIBLE_DEVICES": os.environ.get("CUDA_VISIBLE_DEVICES"),
            "XLA_PYTHON_CLIENT_PREALLOCATE": os.environ.get("XLA_PYTHON_CLIENT_PREALLOCATE"),
            "XLA_FLAGS": os.environ.get("XLA_FLAGS"),
        },
    }


def write_report(report: dict[str, Any], output: Path | None) -> None:
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered, flush=True)
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
