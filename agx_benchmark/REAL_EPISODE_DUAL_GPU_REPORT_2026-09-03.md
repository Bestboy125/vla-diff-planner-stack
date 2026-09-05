# UAV-Flow Real Episode: AGX Orin 64GB vs RTX 4090D 48GB

Date: 2026-09-03

## Input data

- Source: `data/real/2025-05-03_16-45-45`
- Frames: 46 aligned JPEG images and state records
- Instruction: `Move closer to the streetlight on the left side`
- Unified instruction: `Advance toward the streetlight from the left side`
- State contract: `[x, y, z, yaw_deg]`, selected from columns `[0, 1, 2, 4]`
- Target action: adjacent raw poses transformed into the current body frame as
  `[dx_body_m, dy_body_m, dz_body_m, d_yaw_rad]`; the final-frame target is zero,
  matching the training dataset convention.

The earlier synthetic benchmark used one deterministic gradient image, zero state,
and a fixed generic instruction. This report replaces those inputs with every frame,
state, and instruction from one complete real flight episode.

## Runtime configuration

| Platform | OpenVLA | π0 / π0.5 |
|---|---|---|
| Jetson AGX Orin 64GB | PyTorch 2.8.0, CUDA 12.6, BF16, Transformers 4.40.1 SDPA | JAX 0.5.3, BF16/XLA, 10 denoise steps; Orin-safe XLA flags |
| Jetson AGX Orin 64GB, optimized | TensorRT 10.3 BF16 vision encoder + projector; PyTorch 2.8 for the language model, KV cache, autoregressive generation, and action de-tokenization | — |
| RTX 4090D 48GB under WSL2 | PyTorch 2.8.0, CUDA 12.8, BF16, Transformers 4.40.1 SDPA | JAX 0.5.3, BF16/XLA, 10 denoise steps; native/default RTX XLA kernels |

π-family noise is deterministic per frame: `noise_seed = 0 + frame_index`.
The reported policy latency starts with an already-decoded RGB image in host memory
and includes model preprocessing, inference/denoising, and output transforms.

## Real-episode inference performance

| Model | Hardware | Load (s) | Warmup/JIT (ms) | Mean (ms) | P50 (ms) | P95 (ms) | Min–Max (ms) | Rate (Hz) | JPEG decode (ms) | Framework peak (GiB) | GPU util. | Power reading (W) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| OpenVLA | AGX Orin | 9.648 | 1055.390 / 577.025 | 578.118 | 577.837 | 584.657 | 571.839–593.648 | 1.7298 | 1.114 | 14.434 alloc. | 91.03% | 31.431 GPU_SOC |
| OpenVLA hybrid | AGX Orin, TRT vision + PT language/action | 11.457 | 716.796 / 545.479 | **541.337** | 541.137 | 542.361 | 540.683–546.881 | **1.8473** | 0.909 | 12.935 PT alloc.* | 95.95% | 32.969 GPU_SOC |
| OpenVLA | RTX 4090D | 7.518 | 864.281 / 231.971 | **225.297** | 224.290 | 244.504 | 203.989–249.054 | **4.4386** | 0.387 | 14.433 alloc. | 46.19% | 198.129 board |
| π0 | AGX Orin | 17.214 | 9195.808 JIT | 505.471 | 505.509 | 511.531 | 500.001–516.105 | 1.9784 | 1.060 | 6.530 JAX | 92.50% | 33.120 GPU_SOC |
| π0 | RTX 4090D | 7.775 | 16922.749 JIT | **90.015** | 89.899 | 94.505 | 87.765–95.034 | **11.1093** | 0.667 | 6.518 JAX | 83.54% | 253.912 board |
| π0.5 | AGX Orin | 6.393 | 9385.717 JIT | 532.399 | 532.433 | 536.939 | 526.437–546.117 | 1.8783 | 1.035 | 6.638 JAX | 93.57% | 32.913 GPU_SOC |
| π0.5 | RTX 4090D | 8.233 | 12457.021 JIT | **96.709** | 96.386 | 99.973 | 94.402–102.567 | **10.3403** | 0.504 | 6.786 JAX | 84.36% | 263.277 board |

## Hardware speedup on the same real episode

| Model | AGX mean | RTX 4090D mean | RTX speedup | Latency reduction | Output delta across hardware |
|---|---:|---:|---:|---:|---:|
| OpenVLA | 578.118 ms | 225.297 ms | **2.566×** | 61.03% | max absolute delta = **0** |
| OpenVLA hybrid AGX vs native RTX | 541.337 ms | 225.297 ms | **2.403×** | 58.38% | max absolute delta = **0** |
| π0 | 505.471 ms | 90.015 ms | **5.615×** | 82.19% | mean / max abs. delta = 0.000491 / 0.004353 |
| π0.5 | 532.399 ms | 96.709 ms | **5.505×** | 81.84% | mean / max abs. delta = 0.000540 / 0.006300 |

The small π-family output deltas arise from GPU/XLA numerical differences; all
outputs were finite. OpenVLA selected the same action tokens and produced identical
four-dimensional actions on both platforms.

## TensorRT hybrid effect on AGX

| Metric | Native PyTorch 2.8 | TensorRT vision + PyTorch remainder | Change |
|---|---:|---:|---:|
| Mean policy latency | 578.118 ms | **541.337 ms** | **-6.362%** |
| P95 policy latency | 584.657 ms | **542.361 ms** | **-7.234%** |
| Policy rate | 1.7298 Hz | **1.8473 Hz** | **+6.793%** |
| Estimated GPU_SOC energy/call | 18.171 J | **17.847 J** | **-1.781%** |
| PyTorch allocator peak | 14.434 GiB | **12.935 GiB*** | -1.499 GiB* |
| tegrastats system RAM | 20,218 MB | 21,495 MB | +1,277 MB |
| Final action difference over 46 frames | reference | **0 changed frames; max delta 0** | action-equivalent |
| First real-frame vision feature relative L2 | reference | 0.7146% | finite |

`*` TensorRT-owned memory is outside the PyTorch allocator. The hybrid lowers
latency and estimated per-call GPU_SOC energy, but it increases whole-system RAM
and must not be described as a total-memory optimization.

## Single-step imitation error on this episode

| Model | Hardware | MAE dx (m) | MAE dy (m) | MAE dz (m) | MAE yaw (rad) | Mean per-frame L2 |
|---|---|---:|---:|---:|---:|---:|
| OpenVLA | AGX / RTX | 0.191036 | 0.081073 | **0.014591** | **0.003588** | 0.219865 |
| OpenVLA hybrid | AGX Orin | 0.191036 | 0.081073 | **0.014591** | **0.003588** | 0.219865 |
| π0 | AGX | **0.092738** | **0.084139** | 0.018708 | 0.004821 | **0.143801** |
| π0 | RTX 4090D | 0.092874 | 0.084088 | 0.018721 | 0.004845 | 0.143871 |
| π0.5 | AGX | 0.104472 | 0.090610 | 0.024401 | 0.004087 | 0.162479 |
| π0.5 | RTX 4090D | 0.104496 | 0.090845 | 0.024426 | 0.004082 | 0.162789 |

These are open-loop, one-step imitation errors for one 46-frame episode. They are
useful as a sanity check, not as a replacement for full-dataset task success,
collision, constraint, and closed-loop trajectory metrics.

## Memory and measurement notes

- RTX 4090D had an unrelated resident workload using about 18.4–18.9 GB before
  each benchmark. It was not terminated. Whole-card peak minus starting occupancy
  was approximately 14.93 GB for OpenVLA, 8.20 GB for π0, and 10.26 GB for π0.5.
- Framework-owned peaks are more comparable: OpenVLA used about 14.43 GiB on both
  devices; π-family JAX peaks were approximately 6.5–6.8 GiB.
- AGX uses unified memory. Its process RSS, framework allocator, and tegrastats RAM
  are different views and must not be added.
- AGX `GPU_SOC` rail power and RTX whole-board `power.draw` are not equivalent
  power domains. The RTX values also include the unrelated resident workload, so
  energy-per-call should not be compared directly from this run.

## Result files

- `results/real_episode/agx_openvla_pt28.json`
- `results/real_episode/agx_openvla_pt28_trt_vision.json`
- `results/real_episode/agx_pi0_steps10.json`
- `results/real_episode/agx_pi05_steps10.json`
- `results/real_episode/rtx4090d_openvla_pt28.json`
- `results/real_episode/rtx4090d_pi0_steps10.json`
- `results/real_episode/rtx4090d_pi05_steps10.json`

The superseded RTX OpenVLA PyTorch 2.7 sensitivity run is retained as
`results/real_episode/rtx4090d_openvla_pt27.json`; it is not used in the main
hardware comparison.
