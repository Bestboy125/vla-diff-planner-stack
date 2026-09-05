from __future__ import annotations

import argparse
import gc
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image

from benchmark_utils import TegrastatsSampler, base_report, latency_summary, process_memory, write_report


class TensorRTVisionProjector(torch.nn.Module):
    """Execute the static OpenVLA vision+projector subgraph with TensorRT."""

    def __init__(self, engine_path: Path) -> None:
        super().__init__()
        system_packages = "/usr/lib/python3.10/dist-packages"
        if system_packages not in sys.path:
            sys.path.append(system_packages)
        import tensorrt as trt

        self._trt = trt
        self._logger = trt.Logger(trt.Logger.ERROR)
        self._runtime = trt.Runtime(self._logger)
        self._engine = self._runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self._engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {engine_path}")
        self._context = self._engine.create_execution_context()
        self.input_name = "pixel_values"
        self.output_name = "projected_patch_embeddings"
        self.output_shape = tuple(self._engine.get_tensor_shape(self.output_name))

    def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
        inputs = pixel_values.to(dtype=torch.bfloat16).contiguous()
        output = torch.empty(self.output_shape, dtype=torch.bfloat16, device=inputs.device)
        if not self._context.set_tensor_address(self.input_name, inputs.data_ptr()):
            raise RuntimeError("TensorRT rejected the OpenVLA vision input address")
        if not self._context.set_tensor_address(self.output_name, output.data_ptr()):
            raise RuntimeError("TensorRT rejected the OpenVLA vision output address")
        stream = torch.cuda.current_stream(inputs.device).cuda_stream
        if not self._context.execute_async_v3(stream):
            raise RuntimeError("TensorRT OpenVLA vision execution failed")
        return output


def make_image() -> Image.Image:
    yy, xx = np.mgrid[0:256, 0:256]
    image = np.stack((xx, yy, (xx + yy) // 2), axis=-1).astype(np.uint8)
    return Image.fromarray(image, mode="RGB")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/data/vla_benchmark/src/openvla"))
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--attention", choices=("sdpa", "eager"), default="sdpa")
    parser.add_argument("--vision-trt-engine", type=Path)
    parser.add_argument("--compile-backend", choices=("none", "cudagraphs", "inductor"), default="none")
    parser.add_argument(
        "--compile-scope",
        choices=("forward", "language-model", "vision-backbone", "projector", "vision-projector"),
        default="forward",
    )
    args = parser.parse_args()

    sys.path.insert(0, str(args.source / "vla-scripts"))
    from openvla_act import OpenVLAActionAgent

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch CUDA is unavailable")
    torch.cuda.set_device(0)
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats(0)
    sampler = TegrastatsSampler()

    report = base_report("OpenVLA real-3ep", args.model)
    report["framework"] = {
        "name": "PyTorch/Transformers",
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "device": torch.cuda.get_device_name(0),
        "dtype": "bfloat16",
        "attention": args.attention,
        "compile_backend": args.compile_backend,
        "compile_scope": args.compile_scope,
        "vision_backend": "TensorRT 10.3 BF16" if args.vision_trt_engine else "PyTorch",
    }

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
    image = make_image()
    prompt = "In: Current State: 0.0,0.0,0.0,0.0, What action should the uav take to fly forward and avoid obstacles?\nOut:"
    if args.vision_trt_engine:
        validation_inputs = agent.processor(prompt, image).to(agent.device, dtype=torch.bfloat16)
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
    if args.compile_backend != "none":
        if args.compile_scope == "forward":
            agent.model.forward = torch.compile(
                agent.model.forward,
                backend=args.compile_backend,
                fullgraph=False,
                dynamic=False,
            )
        elif args.compile_scope == "language-model":
            agent.model.language_model.forward = torch.compile(
                agent.model.language_model.forward,
                backend=args.compile_backend,
                fullgraph=False,
                dynamic=False,
            )
        elif args.compile_scope in ("vision-backbone", "vision-projector"):
            agent.model.vision_backbone.forward = torch.compile(
                agent.model.vision_backbone.forward,
                backend=args.compile_backend,
                fullgraph=False,
                dynamic=False,
            )
            if args.compile_scope == "vision-projector":
                agent.model.projector.forward = torch.compile(
                    agent.model.projector.forward,
                    backend=args.compile_backend,
                    fullgraph=False,
                    dynamic=False,
                )
        else:
            agent.model.projector.forward = torch.compile(
                agent.model.projector.forward,
                backend=args.compile_backend,
                fullgraph=False,
                dynamic=False,
            )
    torch.cuda.synchronize()
    report["load_seconds"] = round(time.perf_counter() - load_start, 3)

    warmup_ms: list[float] = []
    actions: list[np.ndarray] = []
    for _ in range(args.warmup):
        if args.compile_backend == "cudagraphs":
            torch.compiler.cudagraph_mark_step_begin()
        start = time.perf_counter()
        with torch.inference_mode():
            action = np.asarray(agent.act(image, prompt), dtype=np.float64)
        torch.cuda.synchronize()
        warmup_ms.append((time.perf_counter() - start) * 1000)
        actions.append(action)

    sampler.start()
    latency_ms: list[float] = []
    for _ in range(args.runs):
        if args.compile_backend == "cudagraphs":
            torch.compiler.cudagraph_mark_step_begin()
        start = time.perf_counter()
        with torch.inference_mode():
            action = np.asarray(agent.act(image, prompt), dtype=np.float64)
        torch.cuda.synchronize()
        latency_ms.append((time.perf_counter() - start) * 1000)
        actions.append(action)

    stacked = np.stack(actions)
    report["warmup_ms"] = [round(value, 3) for value in warmup_ms]
    report["latency"] = latency_summary(latency_ms)
    report["output"] = {
        "shape": list(stacked[-1].shape),
        "first": stacked[-1].tolist(),
        "finite": bool(np.isfinite(stacked).all()),
        "deterministic_max_abs_delta": round(float(np.max(np.abs(stacked - stacked[0]))), 9),
    }
    report["memory"] = {
        **process_memory(),
        "torch_peak_allocated_gib": round(torch.cuda.max_memory_allocated(0) / 2**30, 3),
        "torch_peak_reserved_gib": round(torch.cuda.max_memory_reserved(0) / 2**30, 3),
    }
    try:
        free_bytes, total_bytes = torch.cuda.mem_get_info(0)
        report["memory"]["cuda_free_gib_end"] = round(free_bytes / 2**30, 3)
        report["memory"]["cuda_total_gib"] = round(total_bytes / 2**30, 3)
    except RuntimeError:
        pass
    report["tegrastats"] = sampler.stop()
    write_report(report, args.output)


if __name__ == "__main__":
    main()
