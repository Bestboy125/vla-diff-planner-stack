from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

from benchmark_openpi_agx import make_config
from benchmark_utils import TegrastatsSampler, base_report, latency_summary, process_memory, write_report
from real_episode_utils import NvidiaSmiSampler, action_error_metrics, load_real_episode


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/data/vla_benchmark/src/openpi"))
    parser.add_argument("--model-name", choices=("pi0", "pi05"), required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--denoise-steps", type=int, default=10)
    parser.add_argument("--noise-seed", type=int, default=0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    os.environ.setdefault("XLA_FLAGS", "--xla_gpu_autotune_level=0 --xla_gpu_enable_triton_gemm=false")
    sys.path.insert(0, str(args.source / "src"))
    sys.path.insert(0, str(args.source / "packages" / "openpi-client" / "src"))
    import jax
    from openpi.policies import policy_config

    gpu_devices = [device for device in jax.devices() if device.platform == "gpu"]
    if not gpu_devices:
        raise RuntimeError(f"JAX GPU is unavailable; detected devices: {jax.devices()}")
    device = gpu_devices[0]
    dataset, samples = load_real_episode(args.episode)

    images: list[np.ndarray] = []
    decode_ms: list[float] = []
    for sample in samples:
        start = time.perf_counter()
        with Image.open(sample["image_path"]) as source:
            image = np.asarray(source.convert("RGB")).copy()
        images.append(image)
        decode_ms.append((time.perf_counter() - start) * 1000.0)

    horizon = 10 if args.model_name == "pi05" else 50
    noises = [
        np.random.default_rng(args.noise_seed + sample["index"]).standard_normal((horizon, 32), dtype=np.float32)
        for sample in samples
    ]
    observations = [
        {"observation/state": sample["state"], "observation/image": image, "prompt": dataset["instruction"]}
        for sample, image in zip(samples, images)
    ]

    nvidia_idle = NvidiaSmiSampler.snapshot()
    report = base_report(f"OpenPI {args.model_name} UAV-Flow real-episode", args.checkpoint)
    report["dataset"] = dataset
    report["framework"] = {
        "name": "OpenPI/JAX",
        "jax": jax.__version__,
        "device": str(device),
        "platform": device.platform,
        "denoise_steps": args.denoise_steps,
        "noise_seed_rule": "seed + frame_index",
        "noise_seed": args.noise_seed,
        "policy_latency_scope": "RGB ndarray in memory -> transforms -> denoising -> output transforms",
    }
    report["nvidia_smi_before_model_load"] = nvidia_idle

    train_config = make_config(args.model_name)
    load_start = time.perf_counter()
    policy = policy_config.create_trained_policy(
        train_config,
        args.checkpoint,
        sample_kwargs={"num_steps": args.denoise_steps},
    )
    report["load_seconds"] = round(time.perf_counter() - load_start, 3)

    warmup_ms: list[float] = []
    for _ in range(args.warmup):
        start = time.perf_counter()
        warmup = policy.infer(observations[0], noise=noises[0])
        jax.block_until_ready(warmup["actions"])
        warmup_ms.append((time.perf_counter() - start) * 1000.0)

    tegra = TegrastatsSampler()
    nvidia = NvidiaSmiSampler()
    tegra.start()
    nvidia.start()
    inference_ms: list[float] = []
    first_actions: list[np.ndarray] = []
    output_shapes: list[list[int]] = []
    for observation, noise in zip(observations, noises):
        start = time.perf_counter()
        result = policy.infer(observation, noise=noise)
        actions = np.asarray(jax.block_until_ready(result["actions"]), dtype=np.float64)
        inference_ms.append((time.perf_counter() - start) * 1000.0)
        first_actions.append(actions[0])
        output_shapes.append(list(actions.shape))
    nvidia_stats = nvidia.stop()
    tegra_stats = tegra.stop()

    predicted = np.stack(first_actions)
    targets = np.stack([sample["target_action"] for sample in samples])
    report["warmup_ms"] = [round(value, 3) for value in warmup_ms]
    report["latency"] = {
        "image_decode": latency_summary(decode_ms),
        "policy_inference": latency_summary(inference_ms),
        "end_to_end": latency_summary([decode + infer for decode, infer in zip(decode_ms, inference_ms)]),
    }
    report["output"] = {
        "per_frame_shape": output_shapes[0],
        "all_shapes_equal": all(shape == output_shapes[0] for shape in output_shapes),
        "first_actions_shape": list(predicted.shape),
        "finite": bool(np.isfinite(predicted).all()),
        "first_action": predicted[0].tolist(),
        "last_action": predicted[-1].tolist(),
        "first_actions": predicted.tolist(),
    }
    report["imitation_error"] = action_error_metrics(predicted, targets)
    report["memory"] = process_memory()
    stats = device.memory_stats() or {}
    report["memory"].update(
        {
            "jax_bytes_in_use_gib": round(float(stats.get("bytes_in_use", 0)) / 2**30, 3),
            "jax_peak_bytes_in_use_gib": round(
                float(stats.get("peak_bytes_in_use", stats.get("bytes_in_use", 0))) / 2**30, 3
            ),
            "jax_bytes_limit_gib": round(float(stats.get("bytes_limit", 0)) / 2**30, 3),
        }
    )
    report["tegrastats"] = tegra_stats
    report["nvidia_smi"] = nvidia_stats
    write_report(report, args.output)


if __name__ == "__main__":
    main()
