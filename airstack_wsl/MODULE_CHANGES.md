# AirStack WSL/Isaac 模块改动说明

## 1. WSL 基础环境

- 新建独立发行版 `AirStack-22.04`，VHDX 位于 `D:\WSL\AirStack-22.04`。
- 启用 systemd，默认用户为 `airstack`。
- Ubuntu、ROS 2、rosdep 与 GitHub Raw 下载使用可用镜像。
- ROS 2 Humble、OpenVDB 9.1、AirStack common/robot/GCS 均安装在 D 盘 VHDX 内。

## 2. AirStack 公共运行环境

文件：`D:\AirStackWSL\scripts\airstack_env.sh`

- 依次 source Humble、common、robot 和 GCS overlay。
- 固定 `ROS_DOMAIN_ID=42`、Fast DDS 与 Discovery Server 11811。
- 新增 `ROBOT_NAME=${ROBOT_NAME:-robot_${AIR_STACK_DOMAIN_ID}}`，解决原生 launch 缺少 Docker Compose 注入变量的问题。

## 3. Robot bringup

文件：`/home/airstack/AirStack/robot/ros_ws/src/robot_bringup/config/core.perspective`

- 将 behavior tree command 配置由 `/root/AirStack/robot/ros_ws/install/...` 改为 `/home/airstack/AirStack/.native_ws/robot/install/...`。
- 将 fixed trajectory 配置做同样的 WSL overlay 路径适配。

文件：`/home/airstack/AirStack/robot/ros_ws/src/robot_bringup/launch/robot.launch.xml`

- 将 domain bridge YAML 从 Docker 根目录改为 WSL 源码目录。

验证：`sim.launch.xml` 可启动 disparity、point cloud、RViz2 和 RQT；启动阶段无路径错误。

## 4. GCS bringup

文件：`/home/airstack/AirStack/gcs/ros_ws/src/gcs_bringup/config/gcs.perspective`

- 将 GCS command 与 fixed trajectory 配置由 Docker install 路径改为 `.native_ws/gcs/install`。

验证：GCS 可启动 RViz2、RQT 和 domain_bridge；启动阶段无路径错误。

## 5. Fast DDS 跨系统通信

文件：`D:\AirStackWSL\config\airstack-fastdds-discovery.service`

- 在 WSL systemd 中常驻 Fast DDS Discovery Server，监听 UDP 11811。
- Windows 侧动态读取 WSL NAT 地址，避免重启后地址变化。
- Windows 与 WSL 均使用 Humble、Domain 42 和 Fast DDS。

## 6. Isaac Sim 5.1 与 Pegasus

文件：`D:\AirStackWSL\start_isaac_sim_gui.bat`

- 使用 `D:\IsaacSim-5.1\_build\windows-x86_64\release`，不覆盖 Isaac Sim 6。
- 启动并检查 Discovery Server，动态设置 `ROS_DISCOVERY_SERVER`。
- 将 Humble ROS Bridge 的 `lib` 加入 PATH，修复 `rmw_fastrtps_cpp.dll` 加载失败。
- 通过 `--ext-folder` 和 `--enable pegasus.simulator` 加载固定 Pegasus 5.1.0。
- TEMP、pip/Kit、扩展和 Warp 缓存均重定向到 D 盘。

验证：Pegasus、ROS 2 Bridge 均显示 `started`，Stage 到达 `app ready`；GUI 中可见 Pegasus Simulator 的 PX4、ArduPilot 和 ROS 2 后端面板。

## 7. 旧 Docker 方案

- `D:\AirStack`、旧 Zenoh bridge 和 `airstack_windows_integration` 已从原运行路径移除。
- 已解压的 1,050 个场景文件迁移到 `D:\AirStackWSL\scenes`，校验总字节数为 4,870,072,468。
- 因系统策略禁止永久递归删除，旧方案保存在 `D:\AirStackWSL\quarantine\old-docker-solution`，不会参与任何启动流程。
- Docker Desktop 服务保持停止，未启动它来删除停止容器的元数据，也未影响其他 Docker 项目或全局镜像。

## 8. Isaac 场景、Pegasus 与相机

文件：`isaac_sim_windows/run_pegasus_airstack.py`

- 自动加载 `D:\AirStackWSL\scenes\RetroNeighborhood\RetroNeighborhood_Export.usd`。
- 在 `/World/robot_1` 生成 Iris/Pegasus，启用 PX4 TCP lockstep、ROS 2 state/TF 和前视相机。
- RGB/Depth 设置为 320×240；相机安装位姿为机体前方 0.30 m，并显式设置焦距和水平/垂直光圈。

文件：`D:\AirStackWSL\PegasusSimulator\extensions\pegasus.simulator\...\ros2_camera_graph.py`

- 新增可配置 ROS 2 QoS Profile 节点并连接 RGB/CameraInfo Helper，解决 Windows→WSL 大图像 UDP 分片全部丢失。

## 9. 跨域传感器与命名空间

文件：`scripts/isaac_sensor_domain_bridge.py`、`scripts/airstack_sim_env.sh`

- Isaac 保持 Domain 42 + Discovery Server；AirStack 改为 Domain 43 + Simple Discovery。
- 桥接 RGB、Depth、CameraInfo、pose、twist、IMU；每类只保留最新帧，防止图像与高频状态互相挤压。
- RGB/CameraInfo 源端使用 Reliable；Pegasus 状态与 IMU 按发布端使用 Best Effort。
- 修正 CameraInfo 尺寸、K/P 和 ROI 到实际 320×240，并强制 `fy=fx=160 px`，使点云投影与 Isaac 相机模型一致。
- 固定 `ROBOT_NAME=robot_1`，不再由 ROS 域号推导机器人名称。

## 10. AirStack 原生飞行栈

文件：`launch/airstack_interface_native.launch.py`、`launch/airstack_flight_native.launch.py`

- 原生启动 MAVROS、robot_interface、里程计转换、固定轨迹生成器、起降规划器、trajectory_controller、PID 和 behavior。
- 发布 `base_link -> camera_front` 静态 TF。
- 为未启用的 depth/disparity 使用两个不同占位 topic，修复上游“同 topic 不同消息类型”崩溃。
- CameraInfo 在 Domain 43 以 Reliable 发布，满足 DROAN 订阅要求。

## 11. 启停与验证

文件：`scripts/start_airstack_runtime.sh`、`scripts/stop_airstack_runtime.sh`

- 提供幂等启动和精确进程停止；检测 Isaac 重启后无 TCP 连接的旧 PX4，并只重启 instance 0。

文件：`scripts/validate_runtime_streams.py`、`scripts/validate_flight_execution.py`

- 前者验证图像/标定、Pegasus 状态、IMU、MAVLink、里程计、TF 和控制流频率。
- 后者复位 PID 积分器，验证 OFFBOARD、解锁、AirStack 起飞、固定直线轨迹、降落和自动解锁；异常时优先 AirStack 降落，随后回退 PX4 `AUTO.LAND`。

## 12. VLA + DROAN 电线杆闭环（2026-08-29）

文件：`launch/local_airstack_vla.launch.xml`

- `downsample_scale=1`，消除 320×240 disparity 被二次缩放后产生的障碍地图横向偏移。
- `ht=5 s`，使局部轨迹覆盖约 4.2 m，可在进入电线杆前产生完整绕行候选。
- `max_pitch_degrees=0`，本次定高基准只允许水平绕障；该参数已加入 DROAN GL 源码并重新编译。
- 优化轨迹继续发布到隔离话题 `/robot_1/vla/optimized_trajectory`，只有仿真专用执行器通过安全门控后才转发控制器。

文件：`scripts/validate_droan_avoidance.py`、`scripts/inspect_droan_cloud.py`

- 严格验证 map-frame 变换、障碍碰撞证据、横向绕行量、定高误差和杆心净空。
- 可分别检查 disparity、expanded cloud 和 map cloud，定位传感器投影及下采样错误。

文件：`scripts/execute_vla_droan_pole_task.py`

- 仿真专用闭环执行 OpenVLA 方向走廊、DROAN 重规划和 PX4 轨迹跟踪。
- 对每个候选检查新鲜度、有限值、定高、净空、连续性、前向进度、碰撞证据和绕行量；速度限制为 0.20–0.45 m/s。
- 越过障碍后将全局计划裁剪为“当前位置到终点”，防止旧起点导致返航候选。
- Pegasus 地面静止高度约 0.18 m，因此本执行器使用 0.25 m 触地阈值；结束后必须未解锁并恢复 `AUTO.LOITER`。

验证：最终运行转发 17 个安全轨迹段、拒绝 19 个不安全/无效段，实际最小杆心距离 1.087 m（阈值 0.830 m），最终高度 0.187 m、`armed=false`、`mode=AUTO.LOITER`。证据为 `artifacts/pole_task_closed_loop_pass.json`。

### OpenVLA 每 K 步连续推理

文件：`scripts/execute_openvla_kstep_droan_task.py`、`scripts/run_openvla_kstep_task.sh`

- 从 ROS 订阅最新 RGB 和里程计，由 WSL 直接调用 Windows `OpenVLA /predict`；启动前校验 `unnorm_key=real` 的 4 维动作语义与单位。
- 将 K 定义为“已通过安全门控并提交控制器的 DROAN 局部轨迹段数”；默认 `K=3`，可由 `--inference-every-k` 或 `OPENVLA_INFERENCE_EVERY_K` 修改。
- 每 K 段必须等待一个新 RGB 序号，记录帧龄、推理时延、实时 proprio、原始动作、body/map 方向和方向变化。
- 每轮使用实时 yaw 将 `[dx_body,dy_body]` 旋转到 map frame，并从当前位置重建 4.5 m 有界局部走廊；清除旧走廊残留规划结果。
- 模型服务超时、动作非有限/退化/越界、相邻方向变化超过 75°、图像过期或 DROAN 失去安全轨迹时自动进入降落恢复。
- STOP/HOLD 指令在解锁前终止；几何任务到达后自动降落、解锁并恢复 `AUTO.LOITER`。

实测 `K=3`：OpenVLA 调用 9 次，触发轨迹计数严格为 `0,3,6,...,24`；RGB 序号严格递增；推理时延 173–268 ms、平均 211 ms，最大输入帧龄 371 ms。最终执行 25 个安全段，最小实际杆心距离 1.060 m，落地后 `armed=false`、`AUTO.LOITER`。证据为 `artifacts/openvla_k3_droan_closed_loop.json`。

文件：`scripts/record_openvla_rgb_video.py`

- 直接订阅 OpenVLA 实际使用的 ROS RGB 话题，按接收时间补帧并叠加运行时间/源帧序号，避免桌面窗口遮挡导致录屏失真。
- 录制复测采集 206 个源图像，生成 320×240、10 FPS、H.264、85 s 的视频；对应飞行完成 8 次 OpenVLA 推理和 22 个安全段，实际最小杆心距离 1.127 m，最终未解锁。
- 视频：`artifacts/openvla_k3_droan_openvla_rgb_20260829.mp4`；飞行结果：`artifacts/openvla_k3_droan_recorded_20260829.json`。

## 13. 保留限制

- `macvo_ros2` 仍按设计跳过，需要独立 TensorRT/模型环境。
- 当前 320×240 RGB/Depth 约 2.2–2.5 Hz，瓶颈是 RetroNeighborhood 场景的 Windows Isaac 渲染，不是 ROS 桥；更高频 VLA 推理前应进一步降低场景渲染负载或使用独立低分辨率 render product。
- 本次通过的是固定场景中的仿真闭环，不等价于真机放飞许可；实机仍需独立完成模型语言条件评估、真实标定、时延/丢包、地理围栏和系留测试。
