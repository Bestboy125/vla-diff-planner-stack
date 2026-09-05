# π0.5 UAV-Flow inference contract

Checkpoint: configure `PI05_CHECKPOINT_PATH` in the private local environment.

The release is an OpenPI/Orbax checkpoint at step 58,000 (about 1.04 epochs). Its parameter metadata proves:

- π0.5 flow-matching model with PaliGemma and action-expert LoRA parameters.
- Internal model action dimension: 32. A restore audit confirmed the global
  `action_in_proj.kernel` shape is `(32, 1024)`.
- Effective dataset state and action dimensions: 4, from `assets/local/uav_flow_real/norm_stats.json`.
- No continuous `state_proj` parameters, consistent with π0.5 discrete state input.

The reconstructed inference config therefore uses:

- `Pi0Config(pi05=True, action_dim=32, action_horizon=10)`.
- `paligemma_variant="gemma_2b_lora"`.
- `action_expert_variant="gemma_300m_lora"`.
- asset id `local/uav_flow_real` (matching the checkpoint directory layout).
- one RGB image, a four-value state `[x, y, z, yaw_deg]`, and a text prompt.
- output crop from internal 32 dimensions to UAV action `[dx_body, dy_body, dz_body, d_yaw]`.

`action_horizon=10` is the only runtime value that cannot be proven from Orbax parameter shapes. It matches the standard π0.5 LeRobot configuration and does not affect checkpoint parameter restoration. Recover the original training `config.py` if an exact horizon audit is required.

Orbax `_METADATA.write_shape` records the per-shard write shape. The action input
projection showed `(8, 1024)` because the original training checkpoint was sharded
across four devices; treating that value as the global action dimension produces a
restore mismatch against the actual `(32, 1024)` array.

The browser backend sends this raw OpenPI observation:

```text
observation/image: uint8 H×W×3 RGB
observation/state: float32[4]
prompt: string
```

The OpenPI policy server returns `actions: float[horizon, 4]`. The backend validates finite values, exposes the complete chunk for diagnostics, and transports it unchanged in the host-to-onboard message. The onboard `vla_diff_bridge` performs world-frame accumulation and latest-odometry look-ahead filtering, then publishes one validated future target to Diff-Planner. The backend does not select the waypoint.

Start the local policy server from PowerShell:

```powershell
.\pi05\start_pi05_server.ps1
```

The script disables JAX GPU preallocation so π0.5 can coexist with OpenVLA on the 48 GiB GPU. It serves OpenPI's WebSocket protocol on port 8000; `GET /healthz` is the readiness probe used by the ground-station backend.
