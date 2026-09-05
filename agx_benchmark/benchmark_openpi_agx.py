from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

from benchmark_utils import TegrastatsSampler, base_report, latency_summary, process_memory, write_report


def make_observation() -> dict:
    yy, xx = np.mgrid[0:224, 0:224]
    image = np.stack((xx % 256, yy % 256, ((xx + yy) // 2) % 256), axis=-1).astype(np.uint8)
    return {
        "observation/state": np.zeros(4, dtype=np.float32),
        "observation/image": image,
        "prompt": "fly forward and avoid obstacles",
    }


def make_config(model_name: str):
    from openpi import transforms
    from openpi.models import pi0_config
    from openpi.policies import uav_flow_policy
    from openpi.training import config

    is_pi05 = model_name == "pi05"
    asset_id = "local/uav_flow_real" if is_pi05 else "uav_flow_real"
    horizon = 10 if is_pi05 else 50
    model = pi0_config.Pi0Config(
        pi05=is_pi05,
        action_dim=32,
        action_horizon=horizon,
        paligemma_variant="gemma_2b_lora",
        action_expert_variant="gemma_300m_lora",
        pytorch_compile_mode=None,
    )
    return config.TrainConfig(
        name=f"{model_name}_uav_flow_agx_benchmark",
        model=model,
        data=config.SimpleDataConfig(
            assets=config.AssetsConfig(asset_id=asset_id),
            data_transforms=lambda current_model: transforms.Group(
                inputs=[uav_flow_policy.UAVFlowInputs(model_type=current_model.model_type)],
                outputs=[uav_flow_policy.UAVFlowOutputs()],
            ),
            base_config=config.DataConfig(prompt_from_task=True),
        ),
        ema_decay=None,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/data/vla_benchmark/src/openpi"))
    parser.add_argument("--model-name", choices=("pi0", "pi05"), required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--denoise-steps", type=int, default=10)
    parser.add_argument("--fixed-noise-seed", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    # The generic aarch64 JAX 0.5.3 CUDA plugin contains an autotuner helper
    # kernel without an sm_87 image. Disabling GEMM autotuning avoids that
    # helper while leaving all model computation on the Orin GPU.
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
    sampler = TegrastatsSampler()
    report = base_report(f"OpenPI {args.model_name} UAV-Flow", args.checkpoint)
    report["framework"] = {
        "name": "OpenPI/JAX",
        "jax": jax.__version__,
        "device": str(device),
        "platform": device.platform,
        "denoise_steps": args.denoise_steps,
        "fixed_noise_seed": args.fixed_noise_seed,
    }

    train_config = make_config(args.model_name)
    load_start = time.perf_counter()
    policy = policy_config.create_trained_policy(
        train_config,
        args.checkpoint,
        sample_kwargs={"num_steps": args.denoise_steps},
    )
    report["load_seconds"] = round(time.perf_counter() - load_start, 3)

    observation = make_observation()
    fixed_noise = None
    if args.fixed_noise_seed is not None:
        horizon = 10 if args.model_name == "pi05" else 50
        fixed_noise = np.random.default_rng(args.fixed_noise_seed).standard_normal(
            (horizon, 32), dtype=np.float32
        )
    warmup_ms: list[float] = []
    outputs: list[np.ndarray] = []
    for _ in range(args.warmup):
        start = time.perf_counter()
        result = policy.infer(observation, noise=fixed_noise)
        warmup_ms.append((time.perf_counter() - start) * 1000)
        outputs.append(np.asarray(result["actions"], dtype=np.float64))

    sampler.start()
    latency_ms: list[float] = []
    for _ in range(args.runs):
        start = time.perf_counter()
        result = policy.infer(observation, noise=fixed_noise)
        latency_ms.append((time.perf_counter() - start) * 1000)
        outputs.append(np.asarray(result["actions"], dtype=np.float64))

    latest = outputs[-1]
    report["warmup_ms"] = [round(value, 3) for value in warmup_ms]
    report["latency"] = latency_summary(latency_ms)
    report["output"] = {
        "shape": list(latest.shape),
        "first_action": latest[0].tolist(),
        "finite": bool(all(np.isfinite(value).all() for value in outputs)),
        "deterministic_max_abs_delta": round(
            float(max(np.max(np.abs(value - outputs[0])) for value in outputs)), 9
        ),
    }
    report["memory"] = process_memory()
    try:
        stats = device.memory_stats() or {}
        report["memory"]["jax_bytes_in_use_gib"] = round(float(stats.get("bytes_in_use", 0)) / 2**30, 3)
        report["memory"]["jax_peak_bytes_in_use_gib"] = round(
            float(stats.get("peak_bytes_in_use", stats.get("bytes_in_use", 0))) / 2**30, 3
        )
        report["memory"]["jax_bytes_limit_gib"] = round(float(stats.get("bytes_limit", 0)) / 2**30, 3)
    except (AttributeError, RuntimeError):
        pass
    report["tegrastats"] = sampler.stop()
    write_report(report, args.output)


if __name__ == "__main__":
    main()
