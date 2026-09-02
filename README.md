# VLA–Diff-Planner Simulation Stack

This branch records the simulation-validated integration of remote VLA
inference, Isaac Sim, Pegasus, PX4 SITL, AirStack/DROAN, and Diff-Planner.
It is a sanitized source snapshot: datasets, model weights, virtual
environments, simulator installations, generated media, local addresses,
credentials, and remote-desktop access files are not included.

## Validated architecture

```text
Isaac Sim on Windows
  ├── USD scene and Pegasus UAV
  ├── RGB/depth camera and vehicle state
  └── ROS 2 sensor publication
             │
             ▼
AirStack + DROAN + PX4 SITL in WSL
  ├── sensor and frame bridge
  ├── obstacle-aware corridor/trajectory optimization
  └── simulated flight execution
             ▲
             │ bounded semantic goal every K frames
Windows VLA ground station
  ├── OpenVLA or π0.5 inference
  ├── K-frame receding-horizon loop
  ├── coordinate conversion and safety limits
  └── FastAPI/React observation and mission interface
```

The same trajectory contract is represented by the Diff-Planner submodule for
ROS 1 deployment. VLA output is a short body-frame displacement and yaw change,
not a motor, attitude, or actuator command.

## Repository layout

- `isaac_sim_windows/` — scene loading, Pegasus spawning, camera checks, and
  Windows Isaac Sim launch helpers.
- `airstack_wsl/` — ROS 2 bridge, PX4 SITL, AirStack/DROAN launchers, K-frame
  OpenVLA task execution, and validation tools.
- `ground_station/` — FastAPI backend, React console, model adapters, and
  dry-run trajectory protocol.
- `Diff-Planner/` — ROS 1 planner fork as a Git submodule.
- `pi05/` — π0.5/OpenPI launch and policy-contract utilities.
- `onboard_semantic_stage/` — semantic target localization prototype.

## Clone

```bash
git clone --branch simulation-validated --recurse-submodules \
  https://github.com/Bestboy125/vla-diff-planner-stack.git
```

## Local configuration

Tracked files use loopback or `REQUIRED` placeholders. Configure machine paths
and addresses through environment variables, including:

- `OPENVLA_PROJECT_ROOT` and `OPENVLA_MODEL_PATH`;
- `PI05_WSL_DISTRO`, `PI05_WSL_USER`, `PI05_OPENPI_ROOT`, and
  `PI05_CHECKPOINT_PATH`;
- `OPENVLA_URL`, `HOST_ONBOARD_IP`, and `HOST_OPERATOR_IP`; and
- private bridge, observation, and operator tokens where required.

Never commit a generated local configuration file.

## Simulation workflow

The intended startup order is:

1. start Isaac Sim and load the configured USD scene;
2. spawn the Pegasus vehicle and validate RGB/depth and pose publication;
3. start PX4 SITL and the AirStack interface in WSL;
4. validate camera, odometry/TF, MAVLink, and ROS 2 topic continuity;
5. start the Windows VLA backend in dry-run mode;
6. execute K-frame semantic inference and send bounded local targets to the
   simulation planner; and
7. record images, model latency, planner output, vehicle state, and task result.

Refer to `airstack_wsl/README.md`, `airstack_wsl/VALIDATION.md`, and
`isaac_sim_windows/` for the machine-specific setup steps. Paths shown in
historical notes are examples and should be supplied through local configuration.

## Safety boundary

This branch is evidence of a simulation integration path, not a real-flight
acceptance result. Its ground-station output defaults to disabled/dry-run, and
the Diff-Planner bridge defaults to preview-only. Do not use simulation success
as authorization for arming or flight.

## Verification

The source tree includes checks for:

- FastAPI protocol and inference adapters;
- VLA command schema, TTL, sequencing, and coordinate conversion;
- RGB and state stream continuity;
- AirStack/DROAN obstacle-avoidance output;
- PX4 SITL trajectory execution; and
- static PowerShell, Bash, Python, and ROS launch syntax.

Model weights, datasets, Isaac Sim binaries/assets, WSL virtual disks, ROS build
outputs, `node_modules`, virtual environments, logs, and recorded videos are
excluded.

## Upstream and licensing

Diff-Planner and its derived planner code retain their GPL-3.0 license and
upstream notices. Isaac Sim, PX4, ROS, AirStack, OpenVLA, OpenPI/π0.5, model
weights, datasets, and simulator assets remain subject to their own licenses.
