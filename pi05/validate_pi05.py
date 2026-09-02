import argparse
import json
from pathlib import Path
import time

import jax
import numpy as np
from PIL import Image

from openpi.policies import policy_config
from openpi.training import config


def main() -> None:
    parser = argparse.ArgumentParser(description="Restore and run the UAV-Flow π0.5 checkpoint once.")
    parser.add_argument(
        "--checkpoint",
        default="/mnt/e/embodied_agent/uav-flow/pi05_uav_flow_lora_ep1",
    )
    parser.add_argument(
        "--image",
        default=(
            "/mnt/e/embodied_agent/uav-flow/UAV-Flow-main/UAV-Flow-main/"
            "UAV-Flow-Eval/debug.jpg"
        ),
    )
    parser.add_argument("--instruction", default="fly forward and avoid obstacles")
    args = parser.parse_args()

    checkpoint = Path(args.checkpoint)
    image_path = Path(args.image)
    if not (checkpoint / "params").is_dir():
        raise FileNotFoundError(f"Checkpoint params are missing: {checkpoint / 'params'}")
    if not image_path.is_file():
        raise FileNotFoundError(f"Test image is missing: {image_path}")

    train_config = config.get_config("pi05_uav_flow_lora")
    started = time.perf_counter()
    policy = policy_config.create_trained_policy(train_config, checkpoint)
    load_seconds = time.perf_counter() - started

    observation = {
        "observation/image": np.asarray(Image.open(image_path).convert("RGB"), dtype=np.uint8),
        "observation/state": np.asarray([0.0, 0.0, 1.5, 0.0], dtype=np.float32),
        "prompt": args.instruction,
    }

    infer_started = time.perf_counter()
    result = policy.infer(observation)
    infer_seconds = time.perf_counter() - infer_started
    actions = np.asarray(result["actions"], dtype=np.float32)
    if actions.shape != (10, 4):
        raise ValueError(f"Expected π0.5 action chunk [10, 4], got {actions.shape}")
    if not np.isfinite(actions).all():
        raise ValueError("π0.5 returned non-finite actions")

    report = {
        "status": "ok",
        "checkpoint": str(checkpoint),
        "config": "pi05_uav_flow_lora",
        "devices": [str(device) for device in jax.devices()],
        "load_seconds": round(load_seconds, 3),
        "first_inference_seconds": round(infer_seconds, 3),
        "action_shape": list(actions.shape),
        "first_action": actions[0].tolist(),
        "finite": bool(np.isfinite(actions).all()),
        "policy_timing": result.get("policy_timing", {}),
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
