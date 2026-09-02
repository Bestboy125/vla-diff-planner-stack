"""Run one bounded OpenVLA inference and optionally deliver it to the simulator.

This is an acceptance-test utility, not the continuous production control loop.
It requires ``--live`` before any command can leave the host.
"""

import argparse
import asyncio
import base64
import json
import math
from pathlib import Path
import sys
from uuid import uuid4


BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from app.model_gateway import ModelGateway  # noqa: E402
from app.onboard_bridge import OnboardBridgeClient, build_trajectory_command  # noqa: E402
from app.schemas import BridgeCommandRequest, InferenceRequest  # noqa: E402


def validate_action(result: dict, proprio: list[float], max_step_m: float) -> None:
    local_delta = result.get("action_local_delta")
    target = result.get("target_mission")
    if (
        not isinstance(local_delta, list)
        or len(local_delta) != 1
        or len(local_delta[0]) != 4
        or not isinstance(target, list)
        or len(target) != 1
        or len(target[0]) != 4
    ):
        raise RuntimeError("OpenVLA result must contain [1, 4] action and target arrays")
    values = [*local_delta[0], *target[0]]
    if not all(math.isfinite(float(value)) for value in values):
        raise RuntimeError("OpenVLA result contains a non-finite value")

    distance = math.dist([float(value) for value in proprio[:3]], [float(value) for value in target[0][:3]])
    if distance > max_step_m:
        raise RuntimeError(f"target step {distance:.3f} m exceeds {max_step_m:.3f} m")
    target_z = float(target[0][2])
    if not 0.1 <= target_z <= 2.0:
        raise RuntimeError(f"target altitude {target_z:.3f} m is outside [0.1, 2.0] m")


async def run(args: argparse.Namespace) -> None:
    image_path = args.image.resolve()
    request = InferenceRequest(
        image_base64=base64.b64encode(image_path.read_bytes()).decode("ascii"),
        instruction=args.instruction,
        proprio=args.proprio,
    )
    gateway = ModelGateway(args.openvla_url, "127.0.0.1", 8000)
    result = await gateway.predict_openvla(request)
    validate_action(result, args.proprio, args.max_step_m)
    summary = {
        "image": str(image_path),
        "instruction": args.instruction,
        "proprio": args.proprio,
        "action_local_delta": result["action_local_delta"],
        "target_mission": result["target_mission"],
        "live": args.live,
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if not args.live:
        print("DRY RUN: no bridge command was sent")
        return

    client = OnboardBridgeClient(args.bridge_host, args.bridge_port, args.token, args.timeout_sec)
    mission_id = uuid4()
    try:
        for sequence in range(args.count):
            command = BridgeCommandRequest(
                mission_id=mission_id,
                sequence=sequence,
                policy="openvla",
                command="TRACK",
                action_local_delta=result["action_local_delta"],
                target_mission=result["target_mission"],
            )
            ack = await client.send(build_trajectory_command(command, args.ttl_ms))
            print(json.dumps({"sequence": sequence, "command": "TRACK", "ack": ack}))
            if ack.get("status") != "accepted":
                raise RuntimeError(f"TRACK {sequence} was not accepted")
            await asyncio.sleep(args.period_sec)
    except Exception:
        hold = BridgeCommandRequest(
            mission_id=mission_id,
            sequence=args.count + 1,
            policy="openvla",
            command="HOLD",
        )
        try:
            ack = await client.send(build_trajectory_command(hold, args.ttl_ms))
            print(json.dumps({"sequence": args.count + 1, "command": "HOLD", "ack": ack}))
        except Exception as hold_error:
            print(f"safety HOLD failed: {hold_error}", file=sys.stderr)
        raise

    complete = BridgeCommandRequest(
        mission_id=mission_id,
        sequence=args.count,
        policy="openvla",
        command="COMPLETE",
    )
    ack = await client.send(build_trajectory_command(complete, args.ttl_ms))
    print(json.dumps({"sequence": args.count, "command": "COMPLETE", "ack": ack}))
    if ack.get("status") != "accepted":
        raise RuntimeError("COMPLETE was not accepted")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--proprio", required=True, type=float, nargs=4, metavar=("X", "Y", "Z", "YAW_DEG"))
    parser.add_argument("--openvla-url", default="http://127.0.0.1:5007")
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=50051)
    parser.add_argument("--token", default="REQUIRED")
    parser.add_argument("--ttl-ms", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--max-step-m", type=float, default=1.0)
    parser.add_argument("--count", type=int, default=4)
    parser.add_argument("--period-sec", type=float, default=0.25)
    parser.add_argument("--live", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))
