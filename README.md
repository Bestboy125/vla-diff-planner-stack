# VLA–Diff-Planner UAV Integration

This repository contains the host-side integration, simulation adapters, and
deployment contracts for a hierarchical autonomous-UAV stack:

- a remote vision-language-action (VLA) policy provides low-rate semantic
  decisions;
- the onboard computer continuously uploads camera and localization data;
- Diff-Planner performs high-rate local obstacle avoidance and trajectory
  optimization; and
- PX4Ctrl remains the downstream trajectory-tracking controller.

The design keeps model inference away from direct actuator control. VLA output
is converted into bounded local goals, validated again onboard, and then passed
to the local planner. The default configuration is preview-only and does not
arm the vehicle, change the PX4 flight mode, or publish live flight goals.

## Repository layout

- `ground_station/` — FastAPI backend, React operator console, OpenVLA/π0.5
  adapters, K-frame inference loop, and the host-to-onboard protocol client.
- `Diff-Planner/` — Git submodule containing the onboard planner fork and the
  incremental ROS 1 `vla_diff_bridge` package.
- `isaac_sim_windows/` — Isaac Sim scene and Pegasus/AirStack integration
  helpers for Windows.
- `airstack_wsl/` — ROS 2, AirStack, PX4 SITL, and WSL integration scripts used
  by the simulation workflow.
- `pi05/` — π0.5 policy-server launcher and contract validation utilities.
- `onboard_semantic_stage/` and `onboard_mobile_gateway_stage/` — staged ROS
  packages for semantic localization and mobile telemetry integration.
- `docs/` — architecture, protocol, validation, and deployment notes.

Model weights, datasets, virtual environments, build products, runtime logs,
generated media, machine-specific configuration, and remote-desktop access
files are intentionally excluded from version control. AerialClaw is also a
separate local project and is not part of this repository.

## Architecture

```text
Operator browser
      │ HTTP / WebSocket / MJPEG
      ▼
Windows ground station
  ├── OpenVLA or π0.5 inference
  ├── K-frame receding-horizon mission loop
  ├── camera/pose/calibration validation
  └── bounded trajectory-command client
               │ TCP/NDJSON commands
               │ HTTP observation uplink
               ▼
Onboard vla_diff_bridge (ROS 1)
  ├── authentication, sequence, TTL, frame, and limit checks
  ├── preview-only topics by default
  └── live goal topics only after explicit safety gates
               ▼
FAST-LIO/EKF + Diff-Planner + traj_server + PX4Ctrl
```

The host receives RGB images, camera calibration, body-to-camera transforms,
odometry, and planner feedback. Every K accepted image frames, the selected VLA
policy is invoked with the current instruction and synchronized vehicle state.
Only the first bounded action of a returned action chunk is used before the next
perception cycle.

## Safety model

The checked-in settings fail closed:

- command output is disabled by default;
- credentials and calibration identifiers default to `REQUIRED`;
- public examples contain no deployment addresses or tokens;
- live host output and live onboard publication require independent explicit
  gates; and
- startup wrappers contain no MAVROS arm, PX4 mode-switch, or automatic takeoff
  command.

Do not treat a successful preview or simulation test as authorization for
flight. Real-flight deployment requires validated camera intrinsics/extrinsics,
time synchronization, coordinate-frame checks, geofencing, an operator takeover
path, and a staged test plan.

## Clone

```bash
git clone --recurse-submodules <repository-url>
cd vla-planner-project
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Ground-station setup

On Windows PowerShell:

```powershell
cd ground_station/backend
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt

cd ..\frontend
npm install
npm run build
```

Create a private local configuration from
`ground_station/ground_station.local.ps1.example`, or generate matching host and
onboard files with explicit deployment parameters:

```powershell
.\ground_station\initialize_preview_config.ps1 `
  -HostOnboardIp <HOST_ONBOARD_IP> `
  -HostOperatorIp <HOST_OPERATOR_IP> `
  -OnboardBridgeHost <ONBOARD_IP> `
  -CalibrationId <VALIDATED_CALIBRATION_ID>
```

The generated host file is ignored by Git. The generated onboard environment
file is written under `artifacts/`, which is also ignored. Never copy either
file into a commit.

Start the safe operator preview:

```powershell
.\ground_station\start_vla_full_preview.ps1 -Policy OpenVLA
```

The OpenVLA model installation is external to this repository. Set its location
through the documented local environment variables rather than editing tracked
scripts. See `ground_station/README.md` for backend endpoints and development
commands.

## Onboard integration

The Diff-Planner submodule adds `src/integration/vla_diff_bridge` as an
incremental catkin package. Public launch defaults only accept loopback or
`REQUIRED` values; provide the actual host address, tokens, topics, frame names,
and validated calibration ID through a private onboard environment file.

The preview launcher intentionally keeps VLA goals isolated from the live
planner/control topics. Review the submodule README and
`src/integration/vla_diff_bridge/docs/` before any hardware deployment.

## Simulation

The recorded development path uses Isaac Sim on Windows, PX4 SITL and AirStack
inside WSL, and the same Windows VLA backend used by the deployment stack. The
simulation helpers cover scene loading, Pegasus vehicle spawning, ROS 2 sensor
bridging, camera/odometry validation, K-frame OpenVLA inference, DROAN obstacle
avoidance, and trajectory-execution checks.

Simulation dependencies and simulator assets are not vendored. Refer to
`isaac_sim_windows/` and `airstack_wsl/` for the local setup and validation
scripts.

## Jetson AGX Orin 64GB inference benchmark

OpenVLA, π0, and π0.5 were deployed independently to a Jetson AGX Orin
Developer Kit (64GB unified memory) and exercised through their native CUDA
inference paths. This benchmark measures policy-call latency and board resource
requirements; it does not measure task success, action quality, or flight
safety.

### Tested platform and model storage

- JetPack 6.2.1 / L4T 36.4.7, Ubuntu 22.04, CUDA 12.6, MAXN power mode, and
  `jetson_clocks` enabled during measurement.
- A dedicated Python 3.10 Conda environment was created at
  `/data/conda_envs/vla_bench` on the board's Samsung 970 EVO Plus SSD.
- Model weights were copied to `/data/vla_benchmark/models`: OpenVLA 14.05 GiB,
  π0 8.88 GiB, and π0.5 5.90 GiB. File counts and total byte counts matched the
  source copies after transfer. Weights remain external and are not committed
  to this repository.
- Each test used batch size 1, bf16, a deterministic synthetic RGB observation,
  a four-dimensional zero state, and the instruction `fly forward and avoid
  obstacles`.
- Results below contain 20 steady-state policy calls after model-specific
  warmup. Each model ran in a separate process; `tegrastats` sampled system RAM,
  GPU utilization, GPU_SOC power, and temperature during the steady-state
  interval.

### Native inference frameworks

- **OpenVLA:** PyTorch 2.3.0 CUDA, Transformers 4.40.1, timm 0.9.10, bf16, and
  PyTorch scaled-dot-product attention. The adapter checkpoint is loaded through
  the project's OpenVLA inference path.
- **π0:** OpenPI's native JAX/Flax CUDA path with its LoRA PaliGemma/action-expert
  parameters active and 10 denoising steps. Because the checkpoint does not
  record its original policy configuration, this run uses OpenPI's native
  50-action-horizon default as an explicit benchmark assumption.
- **π0.5:** OpenPI's native JAX/Flax CUDA path with LoRA active, 10 denoising
  steps, and the checkpoint's explicit 10-action horizon.

JAX's installation documentation supports Linux aarch64 CUDA wheels, but the
generic JAX 0.5.3 CUDA plugin hit the known `RepeatBufferKernel` autotuning
failure on Orin sm_87. The π benchmarks therefore used
`--xla_gpu_autotune_level=0` and disabled Triton GEMM while retaining CUDA
execution. Their measured latency should be treated as conservative for the
stock wheel. See the corresponding
[JAX issue](https://github.com/jax-ml/jax/issues/22723),
[OpenPI AGX Orin report](https://github.com/Physical-Intelligence/openpi/issues/582),
and [JAX installation documentation](https://github.com/jax-ml/jax/blob/main/docs/installation.md).

### Timing results

| Model | Load | Cold/JIT warmup | Mean | P50 | P95 | Policy calls/s | Output per call |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| OpenVLA | 10.354 s | 1.028 s first call | 575.110 ms | 574.699 ms | 577.981 ms | 1.739 | `[4]` |
| π0 | 7.017 s | 9.000 s JIT | **498.115 ms** | **496.812 ms** | **502.333 ms** | **2.008** | `[50, 4]` |
| π0.5 | **6.267 s** | 8.955 s JIT | 522.506 ms | 521.545 ms | 529.282 ms | 1.914 | `[10, 4]` |

Policy-call rates are not per-action rates. OpenVLA returns one four-dimensional
action, π0 returns a 50-by-4 action chunk under the native-default assumption,
and π0.5 returns a 10-by-4 chunk. The online controller consumes action chunks
according to its receding-horizon policy, so direct per-action throughput claims
would be misleading.

### Memory, utilization, and power

| Model | Peak process RSS | Framework allocator peak | Steady system RAM | Mean GPU utilization | Mean GPU_SOC power | Peak GPU_SOC power |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| OpenVLA | 19.205 GiB | 14.428 GiB | 19,985.7 MB | 92.3% | 31.43 W | 34.17 W |
| π0 | 17.472 GiB | **6.353 GiB** | **15,967.1 MB** | 93.4% | 33.32 W | 36.17 W |
| π0.5 | **16.954 GiB** | 6.656 GiB | 16,661.4 MB | 85.1% | 33.10 W | 36.55 W |

Jetson uses unified system memory. Process RSS, whole-system RAM from
`tegrastats`, and framework allocator bytes describe different views of memory
and must not be added together or interpreted as discrete-GPU VRAM. All three
models completed the warmup and 20 measured calls with finite outputs, zero
swap usage, and no out-of-memory errors.

### Capability conclusions

- The 64GB AGX Orin can run each model locally with useful memory headroom.
  Keeping all three resident concurrently is not recommended because their
  processes plus the operating system would approach the unified-memory limit
  and compete for GPU resources.
- π0 delivered the highest policy update rate: its 498.115 ms mean latency was
  13.4% lower than OpenVLA. π0.5 was 9.1% lower than OpenVLA and had the lowest
  observed process-RSS peak.
- OpenVLA was stable and highly GPU-saturating, but its PyTorch allocator peak
  was more than twice the JAX peaks observed for the two π models.
- For this software stack, π0 is the best latency-first choice. π0.5 is the
  preferable short-action-chunk option and had the smallest process footprint.
  Model selection for flight still requires real observation/action replay,
  closed-loop simulation, safety-bound validation, and staged vehicle tests.

## Verification

Relevant checks include:

```powershell
cd ground_station/backend
.\.venv\Scripts\python.exe -m pytest

cd ..\frontend
npm run build
```

ROS launch and catkin validation must be run on a compatible Ubuntu/ROS 1 host.
Before publishing, scan both the current tree and the branch history for tokens,
private endpoints, generated artifacts, model weights, and machine-specific
paths.

## Publication notes

Only the dedicated sanitized public-release branches are intended for GitHub.
Do not push internal deployment branches, all tags, or the complete local Git
history. The excluded `remote_desktop` directory contains machine-specific
access details and must remain local.

## License and upstream projects

The Diff-Planner submodule is distributed under its own GPL-3.0 license and
retains its upstream notices. Other third-party components remain subject to
their respective licenses. This integration repository does not grant rights
to external model weights, datasets, Isaac Sim assets, PX4, ROS, AirStack, or
other separately obtained dependencies.
