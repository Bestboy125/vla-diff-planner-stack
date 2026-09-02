"""Send a bounded TRACK sequence and COMPLETE to a simulated onboard bridge."""

import argparse
import asyncio
from pathlib import Path
import sys
from uuid import uuid4


BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from app.onboard_bridge import OnboardBridgeClient, build_trajectory_command  # noqa: E402
from app.schemas import BridgeCommandRequest  # noqa: E402


async def run(args: argparse.Namespace) -> None:
    client = OnboardBridgeClient(args.host, args.port, args.token, args.timeout_sec)
    mission_id = uuid4()
    print("mission_id", mission_id)
    target = [[args.x, args.y, args.z, args.yaw]]
    action = [[args.dx, args.dy, args.dz, args.d_yaw]]
    try:
        for sequence in range(args.count):
            request = BridgeCommandRequest(
                mission_id=mission_id,
                sequence=sequence,
                policy=args.policy,
                command="TRACK",
                action_local_delta=action,
                target_mission=target,
            )
            ack = await client.send(build_trajectory_command(request, args.ttl_ms))
            print(sequence, ack["status"], ack["reason"])
            if ack["status"] != "accepted":
                raise RuntimeError("TRACK was not accepted")
            await asyncio.sleep(args.period_sec)
    except Exception:
        # Never retry an ambiguous TRACK sequence. A later HOLD sequence is safe
        # whether the failed TRACK reached the onboard side or not.
        hold = BridgeCommandRequest(
            mission_id=mission_id,
            sequence=args.count + 1,
            policy=args.policy,
            command="HOLD",
        )
        try:
            ack = await client.send(build_trajectory_command(hold, args.ttl_ms))
            print(args.count + 1, ack["status"], ack["reason"])
        except Exception as hold_error:
            print("safety HOLD failed:", hold_error)
        raise

    complete = BridgeCommandRequest(
        mission_id=mission_id,
        sequence=args.count,
        policy=args.policy,
        command="COMPLETE",
    )
    ack = await client.send(build_trajectory_command(complete, args.ttl_ms))
    print(args.count, ack["status"], ack["reason"])
    if ack["status"] != "accepted":
        raise RuntimeError("COMPLETE was not accepted")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--token", required=True)
    parser.add_argument("--policy", choices=("openvla", "pi05"), default="pi05")
    parser.add_argument("--ttl-ms", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--period-sec", type=float, default=0.35)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--dx", type=float, default=0.3)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--d-yaw", type=float, default=0.0)
    return parser.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))
