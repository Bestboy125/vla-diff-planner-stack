"""Validate Windows OpenVLA -> bounded corridor -> WSL AirStack DROAN.

The script never forwards DROAN output to the flight controller. It preserves
the raw model action and uses only its planar direction to construct an
explicitly bounded local planning corridor.
"""

import argparse
import asyncio
import base64
import json
import math
import os
from pathlib import Path
import subprocess
import sys


BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from app.model_gateway import ModelGateway  # noqa: E402
from app.schemas import InferenceRequest  # noqa: E402


STOP_WORDS = ("stop", "hold", "停止", "悬停", "保持")


async def run(args: argparse.Namespace) -> int:
    image_path = args.image.resolve()
    request = InferenceRequest(
        image_base64=base64.b64encode(image_path.read_bytes()).decode("ascii"),
        instruction=args.instruction,
        proprio=args.proprio,
    )
    gateway = ModelGateway(args.openvla_url, "127.0.0.1", 8000)
    model = (
        await gateway.predict_openvla(request)
        if args.policy == "openvla"
        else await gateway.predict_pi05(request)
    )
    action = [float(value) for value in model["action_local_delta"][0]]
    if len(action) != 4 or not all(math.isfinite(value) for value in action):
        raise RuntimeError("OpenVLA action must be four finite values")

    report = {
        "instruction": args.instruction,
        "image": str(image_path),
        "proprio": args.proprio,
        "model": (
            "OpenVLA real 3-epoch checkpoint"
            if args.policy == "openvla"
            else "pi0.5 UAV-Flow LoRA 1 epoch"
        ),
        "raw_action_local_delta": action,
        "raw_target_mission": model["target_mission"][0],
        "flight_command_forwarded": False,
    }

    if any(word in args.instruction.lower() for word in STOP_WORDS):
        report.update(
            status="pass",
            adapter_decision="HOLD",
            planner_invoked=False,
            detail="Stop/hold is terminated at the host safety adapter; no trajectory is published.",
        )
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print("VLA_AIRSTACK_TASK_PASS " + json.dumps(report, ensure_ascii=False))
        return 0

    planar_norm = math.hypot(action[0], action[1])
    if planar_norm < args.min_direction_norm:
        report.update(
            status="fail",
            adapter_decision="REJECT_DEGENERATE_ACTION",
            planner_invoked=False,
            planar_action_norm=planar_norm,
        )
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print("VLA_AIRSTACK_TASK_FAIL " + json.dumps(report, ensure_ascii=False))
        return 1

    direction = [action[0] / planar_norm, action[1] / planar_norm]
    report.update(
        adapter_decision="PLAN_BOUNDED_CORRIDOR",
        planar_action_norm=planar_norm,
        normalized_plan_direction_xy=direction,
        planning_horizon_m=args.planning_horizon_m,
        planning_altitude_m=args.planning_altitude_m,
        planner_invoked=True,
    )

    integration_root = "/mnt/e/embodied_agent/vla_planner_project"
    shell = (
        f"source {integration_root}/airstack_wsl/scripts/airstack_sim_env.sh; "
        f"export DROAN_PLAN_DX={direction[0]:.12g} "
        f"DROAN_PLAN_DY={direction[1]:.12g} "
        f"DROAN_PLAN_LENGTH={args.planning_horizon_m:.12g} "
        f"DROAN_PLAN_ALTITUDE={args.planning_altitude_m:.12g}; "
        f"python3 {integration_root}/airstack_wsl/scripts/validate_droan_avoidance.py"
    )
    completed = subprocess.run(
        ["wsl.exe", "-d", args.wsl_distro, "--", "bash", "-lc", shell],
        capture_output=True,
        text=True,
        timeout=70,
        check=False,
    )
    output_lines = (completed.stdout + "\n" + completed.stderr).splitlines()
    planner_line = next(
        (line for line in reversed(output_lines) if line.startswith("DROAN_AVOIDANCE_")),
        None,
    )
    if planner_line is None:
        raise RuntimeError("DROAN validator returned no machine-readable result")
    planner = json.loads(planner_line.split(" ", 1)[1])
    report["airstack_droan"] = planner
    report["status"] = "pass" if completed.returncode == 0 and planner["status"] == "pass" else "fail"
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    prefix = "VLA_AIRSTACK_TASK_PASS " if report["status"] == "pass" else "VLA_AIRSTACK_TASK_FAIL "
    print(prefix + json.dumps(report, ensure_ascii=False))
    return 0 if report["status"] == "pass" else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--proprio", type=float, nargs=4, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--openvla-url", default="http://127.0.0.1:5007")
    parser.add_argument("--policy", choices=("openvla", "pi05"), default="openvla")
    parser.add_argument("--wsl-distro", default="AirStack-22.04")
    parser.add_argument("--planning-horizon-m", type=float, default=3.5)
    parser.add_argument("--planning-altitude-m", type=float, default=0.5)
    parser.add_argument("--min-direction-norm", type=float, default=1e-5)
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(run(parse_args())))
