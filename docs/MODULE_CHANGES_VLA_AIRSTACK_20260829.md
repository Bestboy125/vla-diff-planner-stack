# 模块改动说明（2026-08-29）

## Windows Isaac Sim

### `isaac_sim_windows/run_pegasus_airstack.py`

- 加载最小 USD 基准场景，不再运行时删除已加载场景 prim，避免渲染产品失效。
- 创建地面、灯光、固定障碍和仿真远端背景几何。
- Pegasus Iris 同时启用 PX4 MAVLink、ROS 位姿/IMU 和相机。
- 绕过 Isaac Sim 5.1 下 Pegasus 旧 ROS CameraHelper 不出图的问题，直接从 Camera 读取 RGB/float-depth 并发布。
- 修正 float RGB 到 uint8 的缩放；显式设置 USD Camera 到机体前向的外参。
- 显式设置焦距和水平/垂直光圈，并统一解析深度代理射线与 CameraInfo 的方形像素模型。
- 验证分辨率改为 320×240，降低 Windows/WSL 原始图像传输压力。

### `isaac_sim_windows/scenes/vla_airstack_benchmark.usda`

- 最小、可重复的 USD 场景入口，避免街区原点围栏和建筑随机影响验收。

### `isaac_sim_windows/start_pegasus_airstack_gui.bat`

- 场景指向 E 盘基准 USD。
- Isaac、Pegasus、缓存和 WSL ROS Discovery Server 启动参数保持在 D/E 盘路径。

## Windows/WSL 传感器边界

### `airstack_wsl/scripts/isaac_sensor_domain_bridge.py`

- 转发 RGB、CameraInfo、Depth、Pose、Twist、IMU。
- 图像改为 best-effort 源 QoS，状态在 WSL 域升级为 reliable。
- CameraInfo 注入 0.12 m 虚拟双目基线，并按 320×240 校正。
- CameraInfo 固定 `fx=fy=160 px`，消除深度点云在 map 中的横向偏移。
- 图像在域边界重打 wall-clock 时间戳，解决 Camera 与 TF 时钟不一致。

### `airstack_wsl/scripts/depth_to_disparity.py`

- 将 Isaac `32FC1` 米制深度转换为 DROAN 要求的 `stereo_msgs/DisparityImage`。
- 新增 `--sim-max-range-m`；当前 launch 使用 8 m，仅用于把 Isaac 的 `+inf` 天空映射为有限仿真量程。

## AirStack 规划与控制

### `airstack_wsl/launch/local_airstack_vla.launch.xml`

- VLA 全局计划输入为 `/robot_1/vla/global_plan`，避免与其他规划器争用。
- DROAN 输出改到 `/robot_1/vla/optimized_trajectory`，与真实控制器物理隔离。
- 仿真参数：障碍扩张半径 0.45 m、seen radius 4 m、规划时域 5 s、`downsample_scale=1`、`max_pitch_degrees=0`。
- 在 DROAN GL 接口新增 `max_pitch_degrees` 参数；本次定高测试禁用上下绕障候选，仅允许水平侧绕。

### `airstack_wsl/launch/airstack_flight_native.launch.py`

- 引用项目内可版本化 launch。
- 启动 depth-to-disparity 适配器。
- 发布 `base_link -> camera_front` 静态 optical TF。

### 启停脚本

- `start_airstack_flight.sh` 使用项目内 launch。
- `stop_airstack_flight.sh`、`stop_isaac_sensor_bridge.sh` 修正动态 E 盘脚本路径的进程匹配。

## 验证与主机适配

### `airstack_wsl/scripts/validate_droan_avoidance.py`

- 发布可配置方向、长度和高度的名义路径。
- 统计真实 depth/disparity/expanded 图、DROAN collision/free/unseen 点。
- 等待新路径稳定后才接收优化段，避免把上一次规划的残留输出误判为本次结果。
- 将轨迹变换到 map frame，并严格检查横向绕行、垂直偏差、杆心净空和碰撞证据；仍明确 `vehicle_commanded=false`。

### `airstack_wsl/scripts/inspect_droan_cloud.py`

- 分层统计 disparity、expanded cloud 与 map cloud，用已知电线杆坐标验证投影和下采样配置。

### `airstack_wsl/scripts/execute_vla_droan_pole_task.py`

- 仅用于 Isaac/PX4 SITL，接收隔离的 DROAN 优化段并通过安全门控后转发 `ADD_SEGMENT`。
- 门控新鲜度、有限值、定高、杆心净空、look-ahead 连续性、前向进度、碰撞证据和横向绕行；速度钳制为 0.20–0.45 m/s。
- 飞过电线杆后裁剪旧全局路径，避免规划器重新指向出生点。
- 完成任务后降落、确认未解锁，并恢复 `AUTO.LOITER`；Pegasus 专用触地阈值为 0.25 m。
- 最终实测：17 个轨迹段获准执行，19 个候选被拒绝，真实最小杆心距离 1.087 m，最终 `armed=false`、`AUTO.LOITER`。

### `airstack_wsl/scripts/execute_openvla_kstep_droan_task.py`

- 将单次 OpenVLA 方向升级为每 K 个已执行安全轨迹段重新推理；默认 `K=3`。
- 每轮直接使用最新 ROS RGB 和里程计调用 Windows OpenVLA，校验动作契约、有限值、幅值、平面模长和相邻方向变化。
- 按实时 yaw 执行 body→map 旋转，从当前位置创建新的 4.5 m 局部走廊，并丢弃旧走廊对应的异步规划结果。
- 保存每轮 RGB 序号、帧龄、推理耗时、proprio、原始动作和新走廊，结果可用于验证 K 周期是否真实生效。
- STOP/HOLD 在解锁前结束；模型/图像/规划超时或安全门控失败时降落恢复；几何目标完成后自动降落和解锁。
- `K=3` 实测完成 9 次推理、25 个安全轨迹段，实际杆心净空 1.060 m，最终未解锁并处于 `AUTO.LOITER`。

### `airstack_wsl/scripts/run_openvla_kstep_task.sh`

- 提供连续推理任务启动入口；`OPENVLA_INFERENCE_EVERY_K`、`OPENVLA_TASK_INSTRUCTION` 和 `OPENVLA_TASK_OUTPUT` 可覆盖默认值。

### `airstack_wsl/scripts/capture_vla_observation.py`

- 同步捕获 RGB、深度统计和 `[x,y,z,yaw]`，生成模型输入证据。

### `airstack_wsl/scripts/validate_runtime_streams.py`

- 验证 RGB/CameraInfo 尺寸一致且至少 320×240，不再硬编码 640×480。

### `airstack_wsl/scripts/validate_flight_execution.py`

- 新增 `AIRSTACK_TAKEOFF_HOLD_SEC` 和 `AIRSTACK_SKIP_LINE`，用于受控悬停诊断。
- 通用起降/直线飞行验证此前已通过；电线杆任务使用上述独立执行器及其专用落地高度判据。

### `ground_station/backend/tools/vla_airstack_task_validation.py`

- 支持 `openvla` 与 `pi05`。
- 保存原始模型动作；运动任务只提取平面方向，生成显式长度/高度限制的规划走廊。
- STOP/HOLD 在主机端直接终止。
- 调用 WSL DROAN 验证，但永不转发控制器。

### `ground_station/backend/tools/pi05_task_benchmark.py`

- 同一帧执行 forward/left/stop 三任务。
- 验证 `[10,4]` action chunk、热推理时延和提示词间首动作距离。
