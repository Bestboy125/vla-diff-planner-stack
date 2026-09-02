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
