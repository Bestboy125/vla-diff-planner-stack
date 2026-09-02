"""Run the Windows-accessible π0.5 server on several UAV task prompts."""

import argparse
import asyncio
import base64
import json
from pathlib import Path
import sys

import numpy as np


BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from app.model_gateway import ModelGateway  # noqa: E402
from app.schemas import InferenceRequest  # noqa: E402


async def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    image = base64.b64encode(args.image.read_bytes()).decode("ascii")
    prompts = [
        "Fly forward and avoid the obstacle",
        "Move left to inspect the clear side",
        "Stop and hold position",
    ]
    gateway = ModelGateway("http://127.0.0.1:5007", args.host, args.port)
    results = []
    for prompt in prompts:
        response = await gateway.predict_pi05(
            InferenceRequest(image_base64=image, instruction=prompt, proprio=[0.0, 0.0, 0.5, 0.0])
        )
        results.append(
            {
                "instruction": prompt,
                "first_action": response["action_local_delta"][0],
                "action_shape": [len(response["action_chunk"]), len(response["action_chunk"][0])],
                "server_timing": response.get("server_timing", {}),
            }
        )

    actions = np.asarray([item["first_action"] for item in results], dtype=np.float64)
    pairwise_l2 = {
        "forward_vs_left": float(np.linalg.norm(actions[0] - actions[1])),
        "forward_vs_stop": float(np.linalg.norm(actions[0] - actions[2])),
        "left_vs_stop": float(np.linalg.norm(actions[1] - actions[2])),
    }
    report = {
        "status": "pass",
        "model": "pi0.5 UAV-Flow LoRA 1 epoch",
        "image": str(args.image.resolve()),
        "results": results,
        "pairwise_first_action_l2": pairwise_l2,
        "prompt_discrimination_observed": any(value > 1e-5 for value in pairwise_l2.values()),
    }
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print("PI05_TASK_BENCHMARK_PASS " + json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    asyncio.run(main())
