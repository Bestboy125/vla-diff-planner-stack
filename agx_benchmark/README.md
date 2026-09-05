# AGX Orin VLA benchmark

This directory contains deterministic, batch-size-one inference benchmarks for
the three UAV-Flow checkpoints deployed to `/data/vla_benchmark` on the Jetson
AGX Orin 64GB.

The OpenVLA benchmark uses PyTorch and Transformers. The π0 and π0.5 benchmarks
use their native OpenPI JAX path so their LoRA adapters remain active. Each run
reports load time, cold/JIT warmup time, steady-state latency and throughput,
framework allocator usage, process RSS, output validity, and sampled Jetson
RAM/GPU-utilization/power data from `tegrastats`.

The π0 checkpoint uses the OpenPI defaults recorded by its parameter tree:
32 internal action dimensions, 50-action horizon, and LoRA PaliGemma/action
expert variants. π0.5 uses the release contract's 10-action horizon.

On JetPack 6.2.1, the generic JAX 0.5.3 aarch64 CUDA plugin aborts in its
`RepeatBufferKernel` autotuning helper because that helper has no sm_87 image.
The benchmark therefore sets `--xla_gpu_autotune_level=0` and disables Triton
GEMM. The model still executes on CUDA, but the reported OpenPI latency should
be interpreted as a conservative result for this stock wheel.

## Measured result (2026-09-03)

The board ran in MAXN mode with `jetson_clocks`, batch size 1, bf16, and 20
steady-state policy calls per model after warmup. All outputs were finite; no
run used swap or raised an out-of-memory error.

- OpenVLA: mean 575.110 ms, p95 577.981 ms, 1.739 calls/s, 19.205 GiB peak
  process RSS, 14.428 GiB peak PyTorch allocation, 92.3% mean GPU utilization,
  and 31.43 W mean GPU_SOC power.
- π0: mean 498.115 ms, p95 502.333 ms, 2.008 calls/s, 17.472 GiB peak process
  RSS, 6.353 GiB peak JAX allocation, 93.4% mean GPU utilization, and 33.32 W
  mean GPU_SOC power.
- π0.5: mean 522.506 ms, p95 529.282 ms, 1.914 calls/s, 16.954 GiB peak
  process RSS, 6.656 GiB peak JAX allocation, 85.1% mean GPU utilization, and
  33.10 W mean GPU_SOC power.

Policy-call rates are not per-action rates: OpenVLA returns one 4-D action, π0
returns a 50-by-4 action chunk under the assumed native default, and π0.5
returns a 10-by-4 chunk.

## Board layout

- Conda environment: `/data/conda_envs/vla_bench`
- Models: `/data/vla_benchmark/models`
- Framework sources: `/data/vla_benchmark/src/openvla` and
  `/data/vla_benchmark/src/openpi`
- Benchmark scripts: `/data/vla_benchmark/src/agx_benchmark`
- Raw results: `/data/vla_benchmark/results/openvla.json`, `pi0.json`, and
  `pi05.json`
- Supporting evidence: `system-info.txt`, `model-size-manifest.tsv`,
  `pip-freeze.txt`, and per-model `.log` files in the same results directory

Activate the environment with:

```bash
source /data/miniconda3/etc/profile.d/conda.sh
conda activate /data/conda_envs/vla_bench
```

## Optimization follow-up

The validated output-equivalent OpenVLA optimization uses PyTorch 2.8 for the
language/generation path and TensorRT 10.3 BF16 for the static vision+projector
subgraph. It reduced the 20-run mean from 575.110 ms to 540.830 ms (5.961%)
while producing the same final action.

For π0 and π0.5, reducing diffusion denoising from 10 steps to 5 lowered fixed-
noise mean latency by 15.964% and 15.324%, respectively. This changes the action
output and is therefore an opt-in speed/quality profile rather than an
equivalent runtime replacement.

See `AGX_OPTIMIZATION_REPORT_2026-09-03.md` for accepted profiles, rejected
experiments, memory/power effects, numerical checks, and raw-result paths.
