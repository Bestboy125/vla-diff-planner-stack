# AirStack 原生部署验证记录

## 已通过

- Ubuntu Jammy WSL rootfs 镜像 SHA256 校验通过。
- WSL 发行版为 WSL2，虚拟磁盘位于 D 盘。
- ROS 2 Humble talker/listener 本机测试连续收发成功。
- AirStack 主仓库提交与指定提交一致。
- 所有 Git 子模块提交一致，包括 Pegasus `f40897f...`。
- `common` 9 个包编译成功。
- `robot` 44 个包编译成功；仅按设计跳过 `macvo_ros2`。
- `gcs` 6 个包编译成功。
- OpenVDB 9.1.0 动态库与 CMake 配置来自 `/usr/local`。
- 机器人可执行文件动态链接检查没有 `not found`。
- `robot_bringup sim.launch.xml` 与 `gcs_bringup gcs.launch.xml` 能完成 launch 参数解析。
- GCS 的 `pytak`、`paho-mqtt`、`utm`、`yaml` 和可选 `aiohttp` 运行时导入成功。
- Windows Isaac 自带 Humble `rclpy` 发布、WSL ROS 2 订阅成功。
- WSL ROS 2 发布、Windows Isaac 自带 Humble `rclpy` 订阅成功。
- systemd Fast DDS 服务启动成功，UDP 11811 正常监听。
- Isaac Sim 5.1 Release 源码构建成功，版本为 `5.1.0-rc.19+mr.0.47d886f2.local`。
- 无窗口启动时 `pegasus.simulator-5.1.0`、`isaacsim.ros2.bridge-4.12.4` 均正常启动，Pegasus World 初始化并到达 `app ready`。
- 图形界面实际启动成功，窗口持续响应；RTX 4090 D 正常渲染，验证时约 57 FPS。
- GUI 中可见 Pegasus Simulator 面板及 PX4、ArduPilot、ROS 2 后端入口。
- 修复 Humble Bridge DLL 路径后，不再出现 `rmw_fastrtps_cpp.dll` 加载错误。
- 使用 Isaac Sim 5.1 自带 Humble `rclpy` 再次验证 Windows→WSL 和 WSL→Windows 双向消息收发，两个方向均以退出码 0 完成。
- `ROBOT_NAME=robot_42` 后，Robot sim bringup 成功启动 disparity、point cloud、RViz2、RQT。
- 修复 Docker 绝对路径后，Robot 和 GCS 启动阶段均无 RQT 配置路径错误。
- GCS bringup 成功启动 RViz2、RQT、domain_bridge。
- AirStack 场景资产迁移到 `D:\AirStackWSL\scenes`：1,050 个文件、4,870,072,468 字节。
- 旧 Docker 方案从原路径移除，Docker Desktop 服务保持停止。

## 上游测试结果

对 DROAN、本地轨迹控制、MAVROS 接口、VDB 和探索规划器执行了 `colcon test`。共运行 16 项：0 个 error，7 个 failure。失败项均为已有源码的 flake8、lint_cmake 或 uncrustify 风格检查；编译、链接、ROS 类型解析和通信没有因此失败。本部署未擅自格式化上游固定提交。

## 版本兼容性结论

Isaac Sim 6.0.1 能启动，但无法解析 Pegasus 5.1.0 对 `omni.isaac.core` 的依赖。该结果是确定的版本不兼容，不是网络问题。因此完整仿真使用 Isaac Sim 5.1；Isaac 6 仅保留供其他项目使用。

## 真实业务闭环（2026-08-28）

- 加载 USD：`RetroNeighborhood_Export.usd`。
- 生成飞行器：`/World/robot_1` Iris/Pegasus。
- PX4 v1.14.3 与 Pegasus TCP 4560 lockstep 建立成功。
- MAVROS：`connected=true`；飞行结束 `armed=false`。
- RGB 与 CameraInfo 均为 640×480，frame 为 `camera_front`；Reliable 源传输通过。
- 运行频率实测：RGB 约 1.69 Hz、CameraInfo 约 29.74 Hz、Pegasus pose 约 114.9 Hz、IMU 约 113.9 Hz、里程计约 13.66 Hz、AirStack tracking 约 20.0 Hz、PID command 约 19.98 Hz。
- TF：`map -> base_link` 动态变换与 `base_link -> camera_front`（x=0.30 m）静态变换均可查询。
- AirStack 飞行测试：悬停推力 0.500；最高高度 0.515 m；固定直线轨迹实际水平位移至少 0.351 m，随后继续到约 x=0.95 m；最终高度 0.0032 m；自动解锁成功。
- `validate_runtime_streams.py` 输出 `RUNTIME_VALIDATION_PASS`，`validate_flight_execution.py` 输出 `FLIGHT_VALIDATION_PASS`。

## 图形验收结论

已经完成：

1. 无窗口加载 `pegasus.simulator`，依赖解析、Python 模块、ROS 2 Bridge 和 Pegasus World 初始化成功。
2. 图形界面启动并保持响应，Pegasus 扩展处于启用状态。
3. GUI 运行期间分别启动 Robot/GCS，关键进程可运行并可干净停止。

上述 USD、飞行器、PX4、相机、里程计/TF、MAVLink 和 AirStack 轨迹执行项目现已全部完成。Isaac Sim 窗口进程持续响应；功能验收以真实传感器与飞行状态数据为准，不以空白 Stage 探针替代。

短时 Robot/GCS 测试使用 SIGINT 自动结束。上游 `rqt_behavior_tree/PyConsole` 在控件尚未完全初始化时会打印 `_console_widget` 关闭告警，但相关进程仍报告 `finished cleanly`；这不是启动失败或通信错误。

## VLA + DROAN 电线杆闭环（2026-08-29）

- 320×240 RGB/Depth 在闭环结束后仍连续发布，独立复核约为 2.22/2.52 Hz。
- CameraInfo 为 `fx=fy=160 px`；DROAN map cloud 与已知电线杆位置对齐。
- OpenVLA 方向走廊经 DROAN 定高侧绕优化，并由仿真执行器逐段做新鲜度、净空、连续性、进度和碰撞证据门控。
- 最终运行获准执行 17 个轨迹段，拒绝 19 个候选；真实最小杆心距离 1.087 m，高于 0.830 m 阈值。
- 最终落地高度 0.187 m，PX4 `armed=false`、`mode=AUTO.LOITER`。
- 机器可读结果：`artifacts/pole_task_closed_loop_pass.json`。

## OpenVLA 每 K 步连续推理闭环（2026-08-29）

- 使用 `K=3`，OpenVLA 实际推理 9 次，触发点严格为累计提交轨迹段 `0,3,6,...,24`。
- 每轮 RGB 序号严格递增；推理时延 173–268 ms、平均 211 ms，最大输入帧龄 371 ms。
- 共提交 25 个安全 DROAN 轨迹段，拒绝 10 个候选；实际最小杆心距离 1.060 m，高于 0.830 m 阈值。
- 任务终点距离 1.397 m，满足 1.400 m 完成阈值；最终高度 0.185 m、`armed=false`、`AUTO.LOITER`。
- STOP/HOLD 指令独立测试在解锁前终止，`flight_command_forwarded=false`。
- 机器可读结果：`artifacts/openvla_k3_droan_closed_loop.json`、`artifacts/openvla_kstep_stop_hold.json`。

### 视频复测

- 重新从出生点运行并同步录制 OpenVLA 实际消费的 ROS RGB 流；桌面窗口捕获因被前景窗口遮挡而未作为证据。
- 有效视频包含 206 个源 RGB 帧，转码为 320×240、10 FPS、H.264、85 s，并通过完整逐帧解码与时间序列联系图检查。
- 对应闭环：8 次 OpenVLA 推理、22 个安全轨迹段、14 个候选被拒绝，实际最小杆心距离 1.127 m。
- 最终高度 0.186 m、`armed=false`、`AUTO.LOITER`。
- 证据：`artifacts/openvla_k3_droan_openvla_rgb_20260829.mp4`、`artifacts/openvla_k3_droan_recorded_20260829.json`。
