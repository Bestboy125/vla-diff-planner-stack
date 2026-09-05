from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch
from torch import nn

from benchmark_openvla_agx import make_image


class VisionProjector(nn.Module):
    def __init__(self, vision_backbone: nn.Module, projector: nn.Module) -> None:
        super().__init__()
        self.vision_backbone = vision_backbone
        self.projector = projector

    def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
        return self.projector(self.vision_backbone(pixel_values))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/data/vla_benchmark/src/openvla"))
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--precision", choices=("bf16", "fp16"), default="bf16")
    args = parser.parse_args()

    sys.path.insert(0, str(args.source / "vla-scripts"))
    from openvla_act import OpenVLAActionAgent

    agent = OpenVLAActionAgent(
        {
            "gpu_id": 0,
            "model_path": str(args.model),
            "http_port": 0,
            "unnorm_key": "real",
            "do_sample": False,
            "attn_implementation": "sdpa",
        }
    )
    prompt = (
        "In: Current State: 0.0,0.0,0.0,0.0, What action should the uav take "
        "to fly forward and avoid obstacles?\nOut:"
    )
    dtype = torch.bfloat16 if args.precision == "bf16" else torch.float16
    inputs = agent.processor(prompt, make_image()).to(agent.device, dtype=dtype)
    module = VisionProjector(agent.model.vision_backbone, agent.model.projector).to(dtype=dtype).eval()
    pixel_values = inputs["pixel_values"].contiguous()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with torch.inference_mode():
        reference = module(pixel_values)
        torch.onnx.export(
            module,
            (pixel_values,),
            str(args.output),
            input_names=["pixel_values"],
            output_names=["projected_patch_embeddings"],
            opset_version=17,
            do_constant_folding=True,
        )
    print(
        {
            "onnx": str(args.output),
            "input_shape": list(pixel_values.shape),
            "input_dtype": str(pixel_values.dtype),
            "output_shape": list(reference.shape),
            "output_dtype": str(reference.dtype),
            "output_finite": bool(torch.isfinite(reference).all().item()),
            "output_abs_max": float(reference.abs().max().item()),
        }
    )


if __name__ == "__main__":
    main()
