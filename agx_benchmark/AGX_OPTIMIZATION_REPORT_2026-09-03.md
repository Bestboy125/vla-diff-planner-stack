# AGX Orin VLA inference optimization report

Date: 2026-09-03  
Target: Jetson AGX Orin Developer Kit 64GB, JetPack 6.2.1 / L4T 36.4.7,
CUDA 12.6, MAXN with `jetson_clocks`  
Protocol: batch size 1, bf16, fixed synthetic observation, model-specific
warmup, then 20 steady-state policy calls in a dedicated process

## Accepted OpenVLA optimization

The best output-equivalent configuration is a hybrid runtime:

- PyTorch 2.8 / CUDA 12.6 retains preprocessing, the Llama language model,
  autoregressive generation, and action de-tokenization.
- TensorRT 10.3 BF16 executes the static vision-backbone and multimodal
  projector subgraph for a fixed `1x6x224x224` image tensor.
- Mean policy latency decreased from 575.110 ms to 540.830 ms, a 5.961%
  reduction. Policy update rate increased from 1.7388 Hz to 1.8490 Hz, a
  6.338% gain.
- P50 is 540.380 ms and P95 is 546.658 ms.
- The four-dimensional action is bit-for-bit identical to the baseline for all
  measured calls. The TensorRT visual feature tensor is finite with 0.45998%
  relative L2 error against the PyTorch bf16 feature tensor.
- GPU_SOC energy estimated over one policy call decreased slightly from 18.078
  J to 17.817 J (1.442%), although mean GPU_SOC power increased from 31.43 W to
  32.94 W.
- PyTorch's allocator peak decreased from 14.428 GiB to 12.931 GiB, but this
  excludes TensorRT-owned memory. Whole-system RAM increased from about 19.99
  GB to 21.55 GB and end-of-run CUDA free memory was 1.61 GiB lower. The hybrid
  is a latency optimization, not a total-memory optimization.

The serialized engine is stored at
`/data/vla_benchmark/engines/openvla/vision_projector_bf16.engine`. Its ONNX
source is `vision_projector_bf16.onnx`. TensorRT engine generation took about
204 seconds once; loading the complete hybrid policy took 11.495 seconds.

## Other OpenVLA experiments

- PyTorch 2.8 alone: 570.982 ms mean, 0.718% faster than the PyTorch 2.3
  baseline. This small difference is not enough to justify an environment
  migration by itself.
- PyTorch 2.3 CUDA Graph on the vision backbone: 552.839 ms mean, 3.872% faster.
  The first compilation call took about 16.9 seconds; later actions were exactly
  equal to baseline.
- Full-model and language-model CUDA Graph tracing failed on the custom
  multimodal attention-mask shape transition used during generation.
- PyTorch 2.8 Inductor could not trace timm's dynamic intermediate-layer set
  construction in the fused vision backbone.
- The available Jetson FlashAttention 2.8.3 wheel had a C++/CUDA ABI symbol
  mismatch with PyTorch 2.8 and was removed from the experimental environment.
- A full end-to-end TensorRT conversion was not accepted. The custom
  autoregressive generation loop, changing KV-cache shapes, action-token
  de-tokenization, and checkpoint-specific Python logic are not a safe static
  TensorRT graph. The hybrid visual subgraph is the validated boundary.

## π0 and π0.5 optimization

The JAX model function was already JIT-compiled. Enabling XLA Triton GEMM made
π0 slower (about 1096 ms in the smoke test), and autotuning level 1 failed on
Orin because several cuDNN helpers lacked an sm_87 kernel image. The accepted
runtime flags therefore remain:

```bash
XLA_FLAGS="--xla_gpu_autotune_level=0 --xla_gpu_enable_triton_gemm=false"
XLA_PYTHON_CLIENT_PREALLOCATE=false
```

A five-step denoising profile provides an optional speed/quality tradeoff:

- π0 with fixed noise seed 0: 10 steps measured 501.023 ms mean and 506.745 ms
  P95; 5 steps measured 421.040 ms mean and 427.071 ms P95. Latency decreased
  15.964%, update rate increased 18.999%, and estimated GPU_SOC energy per call
  decreased from 16.606 J to 14.447 J.
- π0.5 with fixed noise seed 0: 10 steps measured 526.309 ms mean and 529.737 ms
  P95; 5 steps measured 445.660 ms mean and 450.735 ms P95. Latency decreased
  15.324%, update rate increased 18.100%, and estimated GPU_SOC energy per call
  decreased from 17.406 J to 15.349 J.
- All four fixed-noise runs were deterministic, finite, swap-free, and completed
  without OOM.
- The outputs changed when denoising steps changed. First-action L-infinity
  deltas were 0.00907 for π0 and 0.00288 for π0.5 under this single synthetic
  input. These values are not task-quality metrics.

Five-step denoising must remain an opt-in profile until replay-set action error,
closed-loop simulation success, collision/constraint metrics, and staged flight
tests show that the lower step count is acceptable.

## Recommended deployment profiles

- OpenVLA production candidate: PyTorch 2.8 plus the TensorRT 10.3 BF16 visual
  engine. This is the only measured optimization that preserved the final action
  exactly.
- OpenVLA low-change fallback: PyTorch 2.3 plus vision-only CUDA Graph.
- π0/π0.5 quality-first: retain 10 denoising steps and the existing safe XLA
  flags.
- π0/π0.5 latency-first experiment: use 5 denoising steps only behind a runtime
  option and only after closed-loop quality validation.

Raw reports are under `/data/vla_benchmark/results/`, including
`openvla_torch28_trt_vision.json`, `openvla_cudagraphs_vision.json`,
`pi0_steps10_fixed.json`, `pi0_steps5_fixed.json`, `pi05_steps10_fixed.json`,
and `pi05_steps5_fixed.json`.
