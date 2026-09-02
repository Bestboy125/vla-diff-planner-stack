# Isaac Sim 6.0.1 headless server environment

This directory contains deterministic preflight and smoke tests for the RTX 5090 server.

Server path layout:

- `/opt/isaac-sim-6.0.1`: stable runtime path.
- `/opt/isaac-sim-data`: cache, logs, artifacts, and download metadata.
- Physical storage is `/root/autodl-tmp/isaac-sim` because the root filesystem is only 30 GB.

Run checks in this order:

```bash
source /opt/isaac-sim-tools/isaac_sim_env.sh
cd /opt/isaac-sim-tools
./preflight.sh
./run_hello.sh
./run_rgbd_smoke.sh
```

The preflight intentionally refuses to start Isaac Sim when CUDA is visible but NVIDIA Vulkan graphics capability is unavailable. A headless machine does not need a desktop, but its outer container must expose an accessible `/dev/nvidia-modeset` and a working NVIDIA Vulkan ICD.

RGB-D success artifacts are written to `/opt/isaac-sim-data/artifacts/rgbd`.
