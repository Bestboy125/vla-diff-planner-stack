# AirStack 原生 WSL 与 Windows Isaac Sim 部署

## 1. 方案边界

本部署不使用 Docker Desktop。AirStack、ROS 2 Humble、规划器和 GCS 运行在独立的 WSL2 发行版 `AirStack-22.04` 中；Isaac Sim 5.1 运行在 Windows 主机上。Windows/WSL 传感器侧通过 Fast DDS Discovery Server 通信，完整 AirStack 图使用 WSL 本地 Simple Discovery，并由专用桥只转发仿真传感器消息。

部署数据位于 D 盘：

- WSL 虚拟磁盘：`D:\WSL\AirStack-22.04\ext4.vhdx`
- AirStack 源码（位于上述虚拟磁盘内）：`/home/airstack/AirStack`
- Windows 管理脚本与 Pegasus 扩展：`D:\AirStackWSL`
- AirStack USD 场景资产：`D:\AirStackWSL\scenes`
- Isaac Sim 5.1 源码与构建：`D:\IsaacSim-5.1`
- Packman 依赖缓存：`D:\packman-repo`
- Isaac 缓存：`D:\IsaacSimCache`

AirStack 固定在提交 `278acbffaf748cd6e0102b3a25cfea544e031c83`。Pegasus 子模块固定在提交 `f40897f1640bfafa45a9220731cc58c7dfbde33d`。

## 2. 模块说明

### WSL 基础系统

使用 Ubuntu 22.04 Jammy 官方 WSL rootfs 镜像导入，不占用 C 盘默认发行版目录。启用了 systemd，默认用户为 `airstack`。Ubuntu、ROS 2 和 rosdep 使用清华 TUNA 镜像。

### ROS 2 与 AirStack

安装 ROS 2 Humble Desktop、MAVROS、grid_map、domain_bridge、Foxglove Bridge、GStreamer 和构建工具。源码采用三层原生 overlay：

- `common`：AirStack 公共消息和工具，共 9 个包。
- `robot`：规划、建图、MAVROS、轨迹控制和 bringup，共 44 个包。
- `gcs`：任务管理、TAK 和 rqt GCS，共 6 个包。

Ubuntu 22.04 自带 OpenVDB 8.1 与该 AirStack 提交不兼容，因此按照 AirStack Dockerfile 的版本约束，在 `/usr/local` 源码安装 OpenVDB 9.1.0。机器人构建明确链接 ABI 9。

`macvo_ros2` 默认跳过，因为它要求独立的 NVIDIA/TensorRT 用户态环境和模型权重；这不影响 DROAN、VDB、全局探索、MAVROS、轨迹控制及仿真接口。后续启用 MAC-VO 时应单独建立 GPU 环境，不能直接混入当前系统 Python。

### Fast DDS 通信

`airstack-fastdds-discovery.service` 使用 ROS Humble 自带 Fast DDS 2.6，在 WSL 的 loopback 和动态 NAT IPv4 上监听 UDP 11811。Windows 启动器每次读取当前 WSL 地址，并为 Isaac 设置：

- `ROS_DISTRO=humble`
- `ROS_DOMAIN_ID=42`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DISCOVERY_SERVER=<当前 WSL IPv4>:11811`

Isaac 和桥源位于 Domain 42；AirStack、MAVROS、规划器与桥目标位于 Domain 43。`isaac_sensor_domain_bridge.py` 通过 localhost TCP 传递 CDR 数据，并在 Domain 43 重新发布 RGB、CameraInfo、Pegasus pose/twist 和 IMU。这避免大型 AirStack 图压垮 Discovery Server 的端点分发，同时不依赖 Windows/WSL UDP 组播。

### Isaac Sim 与 Pegasus

AirStack 固定提交中的 Pegasus 扩展版本为 5.1.0，并依赖 `omni.isaac.core`。Isaac Sim 6 已移除该扩展，因此不能直接兼容。部署使用官方 Isaac Sim `v5.1.0` 源码构建，并保留原有 Isaac 6 环境，不覆盖它。

Pegasus 扩展位于 `D:\AirStackWSL\PegasusSimulator\extensions\pegasus.simulator`，启动时通过 `--ext-folder` 加载。

当前入口自动加载 `RetroNeighborhood_Export.usd`，在 `/World/robot_1` 生成 Iris/Pegasus，并建立 PX4 TCP lockstep。前视相机输出 320×240 RGB/Depth；桥接层将图像、深度和 CameraInfo 送入 AirStack 域，并固定 `fx=fy=160 px`，使 CameraInfo、深度点云与实际图像一致。

Isaac Sim 5.1 Release 产物版本为 `5.1.0-rc.19+mr.0.47d886f2.local`。Windows 启动器还将 TEMP、Omniverse/Kit、扩展、pip 和 Warp 内核缓存全部指向 D 盘；其中 C 盘显示的 `AppData\Local\ov` 与 `.nvidia-omniverse` 实际为指向 `D:\IsaacSimCache` 的目录联接。

### WSL 原生路径适配

上游 AirStack 的 RQT perspective 和 domain bridge 参数包含 Docker 专用 `/root/AirStack` 绝对路径。本部署保留指定 Git 基线，在以下运行配置中改为 WSL 原生路径：

- Robot RQT 配置改为 `.native_ws/robot/install/...`。
- GCS RQT 配置改为 `.native_ws/gcs/install/...`。
- Robot domain bridge 参数改为 `/home/airstack/AirStack/robot/ros_ws/src/...`。
- ROS 域与机器人身份已解耦：`ROS_DOMAIN_ID=43`，`ROBOT_NAME=robot_1`，PX4 instance 0 对应 MAVLink system 1。

完整修改清单见 `MODULE_CHANGES.md`。

## 3. 常用命令

重新构建全部 ROS 包：

```powershell
wsl.exe -d AirStack-22.04 -u airstack -- bash /mnt/d/AirStackWSL/scripts/build_native.sh all
```

执行原生环境验收：

```powershell
wsl.exe -d AirStack-22.04 -u airstack -- bash /mnt/d/AirStackWSL/scripts/verify_native.sh
```

启动 Windows Isaac/Pegasus 图形入口：

```powershell
E:\embodied_agent\vla_planner_project\isaac_sim_windows\start_pegasus_airstack_gui.bat
```

Isaac 场景就绪后，启动 WSL 全部运行模块并做数据验证：

```powershell
wsl.exe -d AirStack-22.04 -- bash -lc "/home/airstack/AirStack/start_airstack_runtime.sh --validate"
```

重复执行安全飞行闭环验证：

```powershell
wsl.exe -d AirStack-22.04 -- bash -lc "source /home/airstack/AirStack/airstack_cli_env.sh && python3 /home/airstack/AirStack/validate_flight_execution.py"
```

执行 OpenVLA 每 K 个安全轨迹段重新推理的仿真闭环（默认 `K=3`）：

```powershell
wsl.exe -d AirStack-22.04 -- bash /mnt/e/embodied_agent/vla_planner_project/airstack_wsl/scripts/run_openvla_kstep_task.sh
```

可通过环境变量修改周期和指令：

```powershell
wsl.exe -d AirStack-22.04 -- bash -lc "OPENVLA_INFERENCE_EVERY_K=5 OPENVLA_TASK_INSTRUCTION='Fly forward and avoid the utility pole' /mnt/e/embodied_agent/vla_planner_project/airstack_wsl/scripts/run_openvla_kstep_task.sh"
```

这里的一个“步”定义为一个经过净空、定高、连续性、进度和碰撞证据门控后，实际提交给 AirStack 控制器的 DROAN 局部轨迹段。每达到 K 步，执行器必须取得比上一轮更新的 RGB，再调用 Windows `OpenVLA /predict` 并从当前位置重建局部走廊。

## 4. 启动顺序

1. 执行 `start_pegasus_airstack_gui.bat`；脚本会启动 Discovery Server、加载 USD、生成 Iris/Pegasus 并监听 TCP 4560。
2. 执行 `start_airstack_runtime.sh --validate`；脚本依次启动传感器域桥、PX4、MAVROS/接口和 AirStack 飞行栈。
3. 需要实飞复验时执行 `validate_flight_execution.py`，它会起飞、执行短直线、降落并在异常时请求 `AUTO.LAND`。
4. 停止 WSL 运行模块可执行 `/home/airstack/AirStack/stop_airstack_runtime.sh`；该脚本保留 Windows Isaac 窗口。

## 5. 已知说明

WSL NAT 地址可能在 Windows 重启或 `wsl --shutdown` 后变化。Windows Isaac 启动脚本会自动重算地址，不要把旧地址手工写死。

Windows 当前存在 localhost 代理提示。它不影响已验证的 Fast DDS 通信；如需消除提示，应统一 Windows 代理监听端口与 WSL `autoProxy` 配置，而不应改 ROS 发现参数。

旧 Docker 方案已从原运行路径移除。由于本机策略拒绝永久递归删除，旧源码、Zenoh 和 Windows-Docker 测试文件被隔离在 `D:\AirStackWSL\quarantine\old-docker-solution`；Docker Desktop 服务和 `docker-desktop` WSL 发行版保持停止。场景资产已迁移到新方案，不在隔离目录内。

真实业务闭环已完成：USD 场景、Pegasus/Iris、RGB/CameraInfo、Pegasus 状态与 IMU、MAVLink/PX4、MAVROS 里程计、TF、AirStack 固定轨迹生成、轨迹控制、PID、起飞、水平飞行和降落均经过实测。最新数值见 `VALIDATION.md`。
