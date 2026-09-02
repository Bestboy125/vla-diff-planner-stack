# Windows VLA + AirStack + Isaac Sim 可行性验证（2026-08-29）

## 结论

技术链路在 Isaac Sim + PX4 SITL 中已经完成实际闭环绕障飞行并自动降落；这仍不等于“模型可直接控制真机”的验收标准。

已通过的闭环：

`Isaac RGB/Depth -> Windows/WSL 传输 -> Windows OpenVLA 每 K 步推理 -> 有界局部走廊 -> AirStack DROAN -> 安全门控 -> PX4 SITL 执行 -> 连续重规划 -> 任务完成 -> 降落`

DROAN 默认输出仍隔离在 `/robot_1/vla/optimized_trajectory`。只有仿真执行器逐段通过安全门控后才转发给轨迹控制器；最终 PX4 状态为未解锁且 `AUTO.LOITER`。

## 实测结果

### 运行时

- Isaac Sim 5.1 Windows GUI、Pegasus Iris、PX4 SITL、MAVLink、AirStack 原生 WSL 节点均在线。
- RGB、Depth 与 CameraInfo：320×240；最终独立复核 RGB 约 2.22 Hz、Depth 约 2.52 Hz。
- IMU：约 107 Hz；里程计：约 13.2 Hz；AirStack tracking point：约 20.1 Hz。
- TF：`map -> base_link -> camera_front` 可解析。
- MAVLink：connected=true、armed=false、mode=AUTO.LOITER。
- 证据：`artifacts/runtime_320_validation.txt`。

VLA 只负责低频语义/局部方向更新，AirStack/PX4 负责 20 Hz 以上的局部修正与控制。因此 2.5 Hz 可用于本阶段的分层闭环验证，但实机图像链应改为 JPEG/H.264 和压缩深度，不建议继续传原始帧。

### AirStack DROAN

- 障碍深度：中心最近约 1.107 m。
- 碰撞点 6、自由点 1793、未知点 1。
- 生成 15 点优化段。
- OpenVLA 严格 dry-run 中，DROAN 横向绕行 1.714 m、垂直偏差为 0，预测杆心净空 1.660 m。
- 证据：`artifacts/vla_airstack_forward_task_320.json`。

仿真专用参数：`seen_radius=4.0 m`、`ht=5.0 s`、`downsample_scale=1`、`max_pitch_degrees=0`，无穷深度填充为 8 m。它们用于当前定高基准和无纹理天空的有限仿真量程。实机必须恢复“未知空间不可通行”的策略，并使用真实双目/深度相机标定与有效性掩码。

### 实际闭环飞行

- OpenVLA 的实测平面方向为 `[0.9124, 0.4092]`，执行器据此生成 6 m 定高走廊。
- DROAN 连续重规划，17 个安全段被转发，19 个候选因净空、进度、连续性等门控被拒绝。
- 要求的杆心安全距离为 0.830 m；飞行实测最小距离为 1.087 m。
- 最后一个获准段的预测杆心净空为 1.493 m，最大高度误差 0.064 m，包含 12 个碰撞证据点。
- 最终高度 0.187 m、`armed=false`、`mode=AUTO.LOITER`；图像流在任务结束后仍连续发布。
- 证据：`artifacts/pole_task_closed_loop_pass.json`；严格规划证据：`artifacts/pole_task_hover_pitch0_strict.json`。

### OpenVLA 3 epoch

- 真实检查点与 `unnorm_key=real` 服务在线，动作协议 `[dx_body, dy_body, dz_body, d_yaw]` 正常。
- 同一基准帧上，“向前避障”“向左观察”“停止保持”三条指令返回完全相同动作：
  `[0.0005089, 0.0002283, -0.0001588, 0.0003671]`。
- 平面动作模长只有 0.000558 m。组合适配器保留原始动作，只把其平面方向归一化为 3.5 m 的显式规划走廊；没有把微小动作伪装成模型直接输出的大位移。

结论：服务和协议通过，语言条件控制能力未通过。

### OpenVLA 每 K 步连续闭环

- K 的工程定义为“已通过安全门控并提交给控制器的 DROAN 局部轨迹段数”，本轮 `K=3`。
- 真实调用 OpenVLA 9 次，对应提交计数 `0,3,6,9,12,15,18,21,24`；使用的 RGB 序号为 `27,30,33,36,39,42,46,49,52`，没有复用旧帧。
- 推理时延最小 173 ms、平均 211 ms、最大 268 ms；推理前最大帧龄 371 ms。
- 实时 proprio 包含位置、高度和 yaw；每轮 body-frame 动作按当前 yaw 旋转到 map frame，再从当前位置生成新的 4.5 m 走廊。
- 完成 25 个安全轨迹段，拒绝 10 个候选；实际最小杆心距离 1.060 m，目标距离 1.397 m（阈值 1.400 m）。
- 最终高度 0.185 m、`armed=false`、`AUTO.LOITER`。STOP 独立测试在解锁前返回 `HOLD`，未转发飞行命令。
- 证据：`artifacts/openvla_k3_droan_closed_loop.json`、`artifacts/openvla_kstep_stop_hold.json`。

需要如实说明：9 次推理的原始动作仍完全相同。因此本轮证明了“新图像→重复真实推理→坐标转换→局部走廊→DROAN→飞行”的系统闭环有效，但没有证明当前 3-epoch 权重会随画面或指令产生有效策略变化。

### π0.5 1 epoch

- 真实 LoRA 检查点从 E 盘恢复；首次恢复约 335 s。
- 首次 JAX 推理约 6.97 s；热推理约 123–131 ms。
- 三条指令的首动作两两 L2 距离为 0.068、0.124、0.129，存在提示词区分。
- 但两次“向前”测试均给出负 `dx_body`，且数值有明显随机变化。
- 证据：`artifacts/pi05_benchmark_320.json`、`artifacts/pi05_airstack_forward_task_320.json`。

结论：接口、速度和提示词区分通过，动作方向语义仍未通过。

### 停止任务

`Stop and hold position` 在主机安全适配器终止，决策为 `HOLD`，不发布全局计划，也不调用 DROAN，更不发送飞行命令。证据：`artifacts/vla_airstack_stop_task.json`。

## 未通过项与风险

1. OpenVLA 对不同指令输出一致，不能直接执行任务。
2. π0.5 虽区分提示词，但“向前”动作的 x 方向错误，需检查数据坐标系、动作归一化和训练轮次。
3. 已用已知电线杆几何校正相机内参和 map 点云位置，但真机仍需重新做真实相机内外参标定。
4. Pegasus 的里程计原点在落地静止时约为 0.18 m；仿真执行器已使用 0.25 m 专用触地阈值，最终自动解锁通过。
5. 绕障执行只在 Isaac/PX4 SITL 中开启；真机接口仍应保持隔离，直到下述真机门槛全部通过。

## 再验证命令

Windows 启动 Isaac：

```powershell
E:\embodied_agent\vla_planner_project\isaac_sim_windows\start_pegasus_airstack_gui.bat
```

WSL 启动并检查运行时：

```powershell
wsl -d AirStack-22.04 -- bash /mnt/e/embodied_agent/vla_planner_project/airstack_wsl/scripts/start_airstack_runtime.sh --validate
```

OpenVLA + DROAN 隔离任务：

```powershell
ground_station\backend\.venv\Scripts\python.exe ground_station\backend\tools\vla_airstack_task_validation.py `
  --policy openvla `
  --image artifacts\vla_benchmark_rgb_320.jpg `
  --instruction "Fly forward and avoid the obstacle" `
  --proprio 0 0 0.5 0 `
  --output artifacts\vla_airstack_forward_task_320.json
```

仿真实际闭环（会在 PX4 SITL 中解锁和飞行）：

```powershell
wsl -d AirStack-22.04 -- bash -lc 'source /mnt/e/embodied_agent/vla_planner_project/airstack_wsl/scripts/airstack_sim_env.sh; python3 /mnt/e/embodied_agent/vla_planner_project/airstack_wsl/scripts/execute_vla_droan_pole_task.py --output /mnt/e/embodied_agent/vla_planner_project/artifacts/pole_task_closed_loop_pass.json'
```

OpenVLA 每 K 步连续推理闭环（默认 `K=3`）：

```powershell
wsl -d AirStack-22.04 -- bash /mnt/e/embodied_agent/vla_planner_project/airstack_wsl/scripts/run_openvla_kstep_task.sh
```

π0.5 提示词基准：

```powershell
ground_station\backend\.venv\Scripts\python.exe ground_station\backend\tools\pi05_task_benchmark.py `
  --image artifacts\vla_benchmark_rgb_320.jpg `
  --output artifacts\pi05_benchmark_320.json
```

## 下一验收门槛

只有同时满足以下条件才允许把 `/robot_1/vla/optimized_trajectory` 转发到 `/robot_1/trajectory_controller/trajectory_segment_to_add`：

- 模型在固定验证集上的方向正确率、停止召回率达到预设阈值；
- 相机深度和 TF 外参通过已知障碍几何误差测试；
- 每个 waypoint 满足高度、速度、加速度、最大步长和地理围栏；
- 轨迹时间戳新鲜且任务序号单调；
- 失联、推理超时、全轨迹碰撞时自动 HOLD/LAND；
- 先在桩上/系留环境执行，再进行自由飞行。
