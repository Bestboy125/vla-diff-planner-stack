# OpenVLA + USB 相机 + 机载 Diff-Planner 分步启动

本说明中的启动命令由操作员手工执行。编写和验证本说明时没有启动 ROS、PX4Ctrl、运动控制或起飞。
先完成 `REAL_FLIGHT_TEST_SOP.md` 的无桨、定位、坐标方向、故障保护和人工接管检查。
下文不提供解锁、起飞或模式切换命令；这些由现场飞行负责人按已验证流程处理。

## 2026-09-04 安全修订（需要地面重启后生效）

- 六向平移现在通过 `/drone_0_traj_server/operator_yaw_hold` 固定为该次动作下发时的里程计航向，保持目标 `yaw` 不变、`yaw_dot=0`，不再在 0.5 秒后朝运动方向转头；重规划保留锁定。后续有时间戳的新 VLA/主动旋转航向命令解除该锁定。控制误差仍可能导致实际航向小幅波动，不等于物理角度绝对不变。
- VLA 一键启动链选择单独编译的 `traj_server_heading_hold`，ROS 节点名和位置控制话题不变。新 bridge 没有发现航向保持订阅者时拒绝六向移动，避免新旧程序混用导致静默转向。需在地面安全停止旧会话后，用原机载一键命令重启；不在飞行中重启或替换控制器。
- 人工六向移动的 `distance_m` 范围改为 **0.05–2.0 m（含边界）**，网页、后端和机载 `max_operator_step_m=2.0` 一致。VLA 的 `max_goal_step_m=1.0` 不变；上下移动仍受目标高度 0.1–2.0 m 约束。源文件更新后需要在地面安全停止旧会话并重启后端及机载桥接，单独刷新网页不能更新后端校验。
- 按操作员最新要求恢复原自动起飞流程：VLA 启动链使用 `vla_diff_bridge/px4ctrl_vla.launch`，加载原 PX4Ctrl YAML 后覆盖：相对起飞高度 **0.8 m**、`enable_auto_arm=true`、`no_RC=false`。网页和后端也统一为 0.8 m，其他起飞高度拒绝下发。
- 机载 bridge 在起飞请求前检查 PX4Ctrl 参数一致、MAVROS 连接状态新鲜、里程计新鲜；不再要求先人工解锁。PX4Ctrl 原有的地面、静止、遥控器检查以及飞控解锁检查不变，不绕过失败检查。
- **网页起飞是实际动作入口**：LIVE 请求通过检查后发送 `/px4ctrl/takeoff_land`，PX4Ctrl 会尝试切换 Offboard、自动解锁并爬升 0.8 m。bridge 接收回执不代表飞控已执行成功。启动脚本本身不发送起飞请求；不能把启动与起飞混为一谈。
- 新增“实时帧推理（不下发）”按钮，使用同一条机载观测中的 USB 图像和对齐里程计，调用 `/api/inference/latest-observation`。不创建任务、不发送 bridge 命令、不发送 planner goal；正在运行任务、观测过期或推理忙时拒绝诊断。输入语言使用上方自由指令文本框。
- 单次诊断返回动作、耗时、源帧序号及完成时帧龄；结果只用于检查模型，不能证明避障、轨迹执行或飞行安全。不要把“发送具身任务”或 dry-run 的 planner preview 当成这个无下发诊断入口。
- 修改不会热更新正在运行的 PX4Ctrl；仅改 ROS 参数也不能可靠改变已缓存的控制器配置。操作员须在地面且未解锁时停止旧启动会话，再用原两条一键命令重启主机后端和机载栈、刷新网页。不要在空中重启。
- 重启后只读复核 `/px4ctrl/auto_takeoff_land` 与 `/vla_diff_bridge/takeoff_height_m`，必须确认自动解锁为 true、两端高度为 0.8 m。旧进程仍运行时不得按新配置判断可起飞。

## 更新后的一键入口（优先使用）

旧文件名保留，但已经更新为 USB + image_odom。先在 Windows PowerShell 启动主机：

```powershell
& "E:\embodied_agent\vla_planner_project\ground_station\start_vla_full_preview.ps1" -Policy OpenVLA -EnableLiveControl -LiveControlConfirmation I_ACCEPT_REAL_FLIGHT_CONTROL
```

等待模型 ready 和网页后端就绪，再在机载 SSH 终端执行：

```bash
bash /home/nv/Diff-planner/sh_files/start_onboard_vla_full_preview.sh --live I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION
```

这两条是连接真实控制链的启动命令，必须在地面、未解锁状态执行。它们不发送解锁、起飞、模式切换或自动航点指令。
Windows 入口保留 300 ms NTP 启动门禁；机载入口读取私有配置中的 MID-360 编号和 image_odom 模式，自动启动/复用 USB、MAVROS、FAST-LIO、EKF，再启动 bridge、Diff-Planner 和 PX4Ctrl。已有 bridge/控制栈时拒绝重复启动。
两端均省略 Live 参数时是锁定/隔离预览，机载默认预览不启动 PX4Ctrl。不要同时运行一键入口和下面的同名分步节点。
网页仍默认 Dry-run；自然语言在“VLA 具身任务 → 自由指令”输入，实际下发还需网页令牌与 Live 确认。
退出只清理由对应脚本启动的进程，不杀复用节点；主机模型由原模型启动器管理，网页退出不自动卸载模型。飞机仍在空中时不要关闭控制栈，先人工接管、降落、上锁。

## 0. 本次配置与限制

- 主机：`E:\embodied_agent\vla_planner_project`，机载：`nv@192.168.5.5:/home/nv/Diff-planner`。
- 主机与机载绝对时差上限为 300 ms。Live 操作台启动前用三个有效 NTP 样本检查，间隔 10 秒；KoD、超时或未同步 NTP 响应直接失败。此检查不是飞行中监控，也不要求/证明 Windows 时间服务已经同步。
- 观测未来时间戳容差为 300 ms；图像/odom 同机对齐仍为 80 ms，最大观测年龄仍为 750 ms。
- 指令 TTL 仍为 500 ms，机载 watchdog 仍为 1000 ms，过期航点剔除不变。机载快约 200 ms 会消耗约 200 ms 指令 TTL；遇到 expired 应排查时钟和网络，不能现场继续放宽 TTL。
- `VLA_ACTION_CHUNK_SAMPLE_COUNT=8` 默认保留，可改为 6。OpenVLA 若只返回一个动作，则没有可采样的多点动作块；不能把 K=5 理解为执行五步。
- USB 标定确认 ID：`kingsen-ks2a418-usb-640x480-20260904-v1`，确认标志来自操作员确认，不代表本次重新进行了物理标定。
- 当前操作员明确选择 `image_odom` 测试模式：仅使用图像、语言和 FAST-LIO/EKF 状态，不做目标三维定位。主机与机载私有配置均已设置 `VLA_OBSERVATION_MODE=image_odom`。该模式不查询相机安装 TF，发送 `body_from_camera=null`、`calibration_validated=false`，不伪造外参或声明完整标定。图像、CameraInfo、飞机里程计仍必须有效，配置 ID 仍需匹配。以后做目标三维定位时，双端切回 `calibrated` 并验证真实外参。

不要运行 `run_single_lio.sh` 代替以下步骤。更新后的一键入口见上文；底层 `run_diff_px4ctrl_multipoint_vla_preview.sh` 虽然名称有 preview，实际会启动 PX4Ctrl，只由显式 Live 一键分支调用。

## 1. Windows：启动 OpenVLA

在 PowerShell 中执行：

```powershell
Set-Location E:\embodied_agent\vla_planner_project
.\ground_station\start_vla_backend.ps1 -Policy OpenVLA
```

脚本已修复为自动读取 `ground_station/ground_station.local.ps1`，显式命令行路径参数仍优先。
当前私有配置已填入磁盘上存在的 OpenVLA-UAV 源码、虚拟环境和 real-3ep 权重。
等待显示 `OpenVLA is ready`；本次未实际加载模型，路径存在不等于 GPU 推理已验证。

只读检查：

```powershell
Invoke-RestMethod http://127.0.0.1:5007/
```

## 2. Windows：先启动锁定输出的操作台

另开 PowerShell：

```powershell
Set-Location E:\embodied_agent\vla_planner_project
.\ground_station\start_operator_console.ps1
```

网页为 `http://127.0.0.1:8080`。先保持 Dry-run，不提交 Live 任务。
8080 已占用时应先确认并正常停止旧操作台，不要盲目终止不明进程。

## 3. 机载：分终端启动传感器与定位

每个新 SSH 终端先进入 Bash 并加载工作区（避免在 Zsh 里 source setup.bash）：

```bash
bash
unset _CATKIN_SETUP_DIR
source /opt/ros/noetic/setup.bash
cd /home/nv/Diff-planner
source devel/setup.bash
```

分别在独立终端运行，保留各终端，不要重复启动已有节点：

```bash
# 终端 A：ROS master
roscore
```

```bash
# 终端 B：MAVROS（当前机载 launch 默认串口 /dev/ttyTHS1:921600）
roslaunch mavros px4.launch
```

```bash
# 终端 C：MID-360 驱动 + FAST-LIO；编号来自当前机载 BD_LIST
export BD_LIST=47MDM630020460
roslaunch faster_lio mapping_mid360.launch
```

```bash
# 终端 D：EKF，依赖 MAVROS IMU 和 FAST-LIO
roslaunch ekf ekf_lidar.launch
```

```bash
# 终端 E：USB 相机，默认 640x480、30 FPS、MJPEG
roslaunch vla_diff_bridge vla_usb_camera.launch
```

在检查终端执行（本次 image_odom 模式不需要发布相机安装外参）：

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /ekf/ekf_odom
rostopic hz /laserMapping/cloud_registered
rostopic hz /vla_usb_camera/image_raw
```

`hz` 持续运行，每项观察后 Ctrl-C，再查下一项。
必须看到 MAVROS connected=True、armed=False，定位和图像持续更新，机体/世界坐标轴方向已核实。
仅在以后使用 calibrated 模式时才执行 `rosrun tf tf_echo base_link vla_usb_camera_optical_frame` 并要求真实外参一致。

## 4. 无桨：预览闭环

另开并初始化机载终端：

```bash
source /home/nv/.config/vla_stack.env
export VLA_OBSERVATION_MODE=image_odom
export VLA_START_USB_CAMERA=false
bash src/integration/vla_diff_bridge/scripts/run_vla_fastlio_diff_preview.sh
```

该脚本启动隔离 Diff-Planner 预览、bridge 和观测上行，不启动 PX4Ctrl。
USB 相机已由终端 E 启动，避免重复启动。
网页提交 Dry-run 的 OpenVLA 自由指令，确认图像、定位、模型和预览结果都正常，随后停止任务。

## 5. 仅在前置验证通过后：切换实际执行链路

保持未解锁。先停止第 4 步的任务及预览脚本，确认相关 preview planner/bridge/uplink 节点已退出。
停止第 2 步的锁定操作台；重新启动后不保留旧任务。
在 Windows 新终端执行以下**允许实机请求的启动命令**：

```powershell
Set-Location E:\embodied_agent\vla_planner_project
.\ground_station\start_operator_console.ps1 -EnableLiveControl -LiveControlConfirmation I_ACCEPT_REAL_FLIGHT_CONTROL
```

启动前自动执行 300 ms NTP 门禁；失败时不会启动 Live 后端。网页仍默认 Dry-run。

然后在已初始化的机载 Bash 终端执行以下**会启动 PX4Ctrl 的命令**：

```bash
source /home/nv/.config/vla_stack.env
export VLA_OBSERVATION_MODE=image_odom
export VLA_START_USB_CAMERA=0
export VLA_BRIDGE_MODE=live
export ENABLE_FLIGHT_STACK=I_UNDERSTAND_THIS_STARTS_PX4CTRL
export ENABLE_VLA_LIVE_CONTROL=I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION
export MULTIPOINT_START_PLAN=0
export MULTIPOINT_BACK_PLAN=0
export MULTIPOINT_AUTO_PLANNING=0
export MULTIPOINT_AUTO_LANDING=0
export VLA_ACTION_CHUNK_SAMPLE_COUNT=8
bash sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
```

它启动 USB 观测上行、live bridge、Diff-Planner、traj_server、PX4Ctrl 和禁用自动任务的 multipoint。
它不等于“只预览”；虽然没有直接发送解锁或起飞命令，但已连接真实控制链。
脚本要求启动前后均未解锁。不要在已起飞后重启此脚本。
必须由现场飞行负责人完成控制器/遥控器接管、起飞和模式切换检查后，才允许下发 VLA 任务。

## 6. 在哪里输入语言任务

打开 `http://127.0.0.1:8080`，右侧：

1. “任务对话与控制” → “VLA 具身任务” → “自由指令”。
2. 文本框填写任务；该文本原样传给 OpenVLA 的 `instr`，无需改源码。
3. “策略”选择 OpenVLA；先 Dry-run 验证。
4. 实机时选择 Live，填入私有配置中的 `OPERATOR_CONTROL_TOKEN`（不要发到聊天或日志）以及确认短语 `I_ACCEPT_REAL_FLIGHT_CONTROL`。
5. 飞行负责人允许后点击“确认并下发实机任务”。这会创建并启动持续 K 帧推理任务，不是单次推理。

优先采用与你的微调数据一致的语言和简单任务。语言中写“前进 0.5 米后停止”不构成硬距离/自动停止保证。
默认 K=5 表示累计新图像触发推理；推理忙时不并行堆积请求，不是飞机执行五步才更新。
“暂停 VLA”或“停止任务”会停止后续推理调度，但不等于立即物理刹停；应检查机载停止响应，飞手准备接管。
若持续出现 watchdog、超时、机体/世界坐标不一致或里程计缺失，不进入实飞，也不现场放宽保护阈值。
六向移动和左右旋转使用网页“原子任务”，不经过 OpenVLA；“绕目标飞行”仍是 VLA 语言模板，不是确定圆心/半径的轨迹控制器。不测试目标三维定位不代表能严格按指定半径绕飞。

## 7. 日志与结束

- OpenVLA 日志：`artifacts/vla_backend/openvla_stdout.log` 与 `openvla_stderr.log`。
- 操作台日志：`artifacts/ground_station/backend_stdout.log` 与 `backend_stderr.log`。
- 机载日志：启动脚本打印的 `/tmp/vla_diff_full_stack_时间戳/`。
- 不要在空中直接 Ctrl-C 控制栈；先由飞手接管、降落并确认上锁，再停止进程。
- Windows 结束模型服务可用 `ground_station/stop_vla_backend.ps1`，仅处理该启动器记录的进程。
