from __future__ import annotations

import argparse
import gc
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image

from benchmark_openvla_agx import TensorRTVisionProjector
from benchmark_utils import TegrastatsSampler, base_report, latency_summary, process_memory, write_report
from real_episode_utils import NvidiaSmiSampler, action_error_metrics, load_real_episode, openvla_prompt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/data/vla_benchmark/src/openvla"))
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--attention", choices=("sdpa", "eager"), default="sdpa")
    parser.add_argument("--vision-trt-engine", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    sys.path.insert(0, str(args.source / "vla-scripts"))
    from openvla_act import OpenVLAActionAgent

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch CUDA is unavailable")
    torch.cuda.set_device(0)
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats(0)

    dataset, samples = load_real_episode(args.episode)
    images: list[Image.Image] = []
    decode_ms: list[float] = []
    for sample in samples:
        start = time.perf_counter()
        with Image.open(sample["image_path"]) as source:
            image = source.convert("RGB")
            image.load()
        images.append(image)
        decode_ms.append((time.perf_counter() - start) * 1000.0)

    nvidia_idle = NvidiaSmiSampler.snapshot()
    report = base_report("OpenVLA real-3ep real-episode", args.model)
    report["dataset"] = dataset
    report["framework"] = {
        "name": "PyTorch/Transformers",
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "device": torch.cuda.get_device_name(0),
        "dtype": "bfloat16",
        "attention": args.attention,
        "vision_backend": "TensorRT 10.3 BF16" if args.vision_trt_engine else "PyTorch",
        "language_and_action_backend": "PyTorch",
        "policy_latency_scope": "PIL RGB in memory -> preprocessing -> generation -> action de-tokenization",
    }
    report["nvidia_smi_before_model_load"] = nvidia_idle

    load_start = time.perf_counter()
    agent = OpenVLAActionAgent(
        {
            "gpu_id": 0,
            "model_path": str(args.model),
            "http_port": 0,
            "unnorm_key": "real",
            "do_sample": False,
            "attn_implementation": args.attention,
        }
    )
    instruction = dataset["instruction"]
    prompts = [openvla_prompt(sample["state"], instruction) for sample in samples]
    if args.vision_trt_engine:
        validation_inputs = agent.processor(prompts[0], images[0]).to(agent.device, dtype=torch.bfloat16)
        with torch.inference_mode():
            reference_features = agent.model.projector(agent.model.vision_backbone(validation_inputs["pixel_values"]))
            trt_vision = TensorRTVisionProjector(args.vision_trt_engine)
            trt_features = trt_vision(validation_inputs["pixel_values"])
            torch.cuda.synchronize()
        difference = (trt_features.float() - reference_features.float()).flatten()
        reference_flat = reference_features.float().flatten()
        report["vision_validation"] = {
            "reference_finite": bool(torch.isfinite(reference_features).all().item()),
            "tensorrt_finite": bool(torch.isfinite(trt_features).all().item()),
            "max_abs_error": float(difference.abs().max().item()),
            "mean_abs_error": float(difference.abs().mean().item()),
            "relative_l2_error": float(
                torch.linalg.vector_norm(difference).item()
                / max(torch.linalg.vector_norm(reference_flat).item(), 1e-12)
            ),
        }
        agent.model.vision_backbone = trt_vision
        agent.model.projector = torch.nn.Identity()
        del reference_features, trt_features, validation_inputs
        gc.collect()
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats(0)
    torch.cuda.synchronize()
    report["load_seconds"] = round(time.perf_counter() - load_start, 3)

    warmup_ms: list[float] = []
    for _ in range(args.warmup):
        start = time.perf_counter()
        with torch.inference_mode():
            warmup_action = np.asarray(agent.act(images[0], prompts[0]), dtype=np.float64)
        torch.cuda.synchronize()
        warmup_ms.append((time.perf_counter() - start) * 1000.0)
        if warmup_action.shape != (4,) or not np.isfinite(warmup_action).all():
            raise RuntimeError(f"Invalid warmup action: shape={warmup_action.shape}, value={warmup_action}")

    tegra = TegrastatsSampler()
    nvidia = NvidiaSmiSampler()
    tegra.start()
    nvidia.start()
    inference_ms: list[float] = []
    predictions: list[np.ndarray] = []
    for image, prompt in zip(images, prompts):
        start = time.perf_counter()
        with torch.inference_mode():
            action = np.asarray(agent.act(image, prompt), dtype=np.float64)
        torch.cuda.synchronize()
        inference_ms.append((time.perf_counter() - start) * 1000.0)
        predictions.append(action)
    nvidia_stats = nvidia.stop()
    tegra_stats = tegra.stop()

    predicted = np.stack(predictions)
    targets = np.stack([sample["target_action"] for sample in samples])
    report["warmup_ms"] = [round(value, 3) for value in warmup_ms]
    report["latency"] = {
        "image_decode": latency_summary(decode_ms),
        "policy_inference": latency_summary(inference_ms),
        "end_to_end": latency_summary([decode + infer for decode, infer in zip(decode_ms, inference_ms)]),
    }
    report["output"] = {
        "shape": list(predicted.shape),
        "finite": bool(np.isfinite(predicted).all()),
        "first_action": predicted[0].tolist(),
        "last_action": predicted[-1].tolist(),
        "first_actions": predicted.tolist(),
    }
    report["imitation_error"] = action_error_metrics(predicted, targets)
    report["memory"] = {
        **process_memory(),
        "torch_peak_allocated_gib": round(torch.cuda.max_memory_allocated(0) / 2**30, 3),
        "torch_peak_reserved_gib": round(torch.cuda.max_memory_reserved(0) / 2**30, 3),
    }
    free_bytes, total_bytes = torch.cuda.mem_get_info(0)
    report["memory"].update(
        {"cuda_free_gib_end": round(free_bytes / 2**30, 3), "cuda_total_gib": round(total_bytes / 2**30, 3)}
    )
    report["tegrastats"] = tegra_stats
    report["nvidia_smi"] = nvidia_stats
    write_report(report, args.output)


if __name__ == "__main__":
    main()
