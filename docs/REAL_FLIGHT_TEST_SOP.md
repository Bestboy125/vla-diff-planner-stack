# VLA + FAST-LIO + Diff-Planner 实机测试 SOP

适用架构：机载 RealSense / MID-360 / FAST-LIO / EKF / Diff-Planner，Windows 主机 OpenVLA 后端，Wi-Fi 双向通信。

本文采用逐级放权原则。任一级未满足通过条件，不得进入下一级。首次实机测试不得直接运行完整的 `sh_files/run_single_lio.sh`，因为该脚本会连续启动 MAVROS、FAST-LIO、EKF、Diff-Planner、PX4Ctrl 和 multipoint 任务，无法在传感器、规划器和控制器之间设置人工门禁。

## 1. 总体流程

```text
代码与参数冻结
    ↓
G0 场地、人员、硬件和故障保护检查
    ↓
G1 拆桨台架：图像 + 位姿 + VLA + preview 回传
    ↓
G2 拆桨台架：Diff-Planner shadow/preview 优化
    ↓
G3 人工起飞悬停：VLA 与规划器只旁路观察
    ↓
G4 人工目标 + Diff-Planner 低速短距离控制
    ↓
G5 单次 VLA 粗目标 + Diff-Planner 执行
    ↓
G6 K 帧连续 VLA + Diff-Planner 受限任务
    ↓
降落、停机、日志复盘和是否放行下一架次
```

## 2. 当前实飞前阻断项

以下三项未关闭前，只允许 G1/G2，不允许飞行：

1. 相机画面目前主要是近距离墙面或机体结构。必须调整安装角度并确认无遮挡、无保护膜、无明显过曝。
2. 机载时钟比 Windows 主机约快 3 秒。正式测试应使用 NTP/chrony/PTP 将偏差压到 100 ms 以内，不应依赖测试时的 5 秒容差。
3. 当前 `body/base_link -> camera` 外参来自仓库中的 VINS 初值，标识为 `comm-test-unvalidated-handeye`。必须完成实机手眼标定、静态复核和方向检查，生成正式 calibration ID。

建议同时将 `run_single_lio.sh` 拆成四个独立入口：

- `run_sensors_localization.sh`：MAVROS 遥测、MID-360、FAST-LIO、EKF、RealSense。
- `run_vla_preview.sh`：图像/位姿上报和 `/vla/preview_*` 回传。
- `run_diff_planner_shadow.sh`：只输出优化轨迹预览，不接控制器。
- `run_flight_control.sh`：PX4Ctrl 与最终控制话题；只允许飞行负责人在对应门禁通过后启动。

## 3. 人员与职责

每次有桨测试至少两人，推荐三人：

- 飞行负责人：唯一有权决定解锁、起飞、切换模式和降落。
- 安全飞手：全程手持遥控器，手指保持在模式切换开关附近，能够立即切回 Position/Altitude/Manual 并人工降落。
- 软件观察员：只操作 Windows、机载终端和任务网页，口头报告链路、位姿、模型和规划器状态，不操作遥控器。

开始前约定清晰口令：`准备`、`解锁`、`起飞`、`接管`、`降落`、`急停`。只有飞行负责人可发布 `解锁` 与 `起飞` 口令。

## 4. G0：场地和故障保护门禁

### 检查项目

- 室外空旷场地，无人、无车辆、无电线和强反光玻璃；建立警戒区。
- 桨叶、机臂、电机、重心、电池固定和传感器固定完好。
- 遥控器连接稳定，模式开关至少包含 Position/Altitude/Manual 与 Land/RTL 中的安全选项。
- QGroundControl 中确认电池、RC 丢失、数据链丢失、Offboard 丢失和地理围栏动作。
- 记录 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT`、`COM_DL_LOSS_T`、`NAV_DLL_ACT`、RC loss、低电量和 geofence 参数，不在现场临时猜测参数值。
- 在仿真或无桨台架验证一次：Windows 后端退出、Wi-Fi 断开、机载进程退出时，系统不会持续使用旧轨迹。
- 模型路径、Git commit、参数快照、相机 calibration ID 和操作人员写入本次测试记录。

### 通过条件

- 安全飞手可以在 1 秒内切回人工控制模式。
- 所有 failsafe 行为已知且经过仿真验证。
- 地理围栏和高度限制与本次场地匹配。
- 电池余量足以完成测试并保留安全返航/降落余量。

## 5. G1：拆桨通信闭环

必须拆除全部桨叶，机体固定在台架上。

### 启动顺序

1. 启动飞控、遥控器和 QGroundControl，确认 `armed=false`。
2. 启动 ROS master 与 MAVROS 遥测；不发送 arm、takeoff、mode 或 setpoint 命令。
3. 启动 MID-360、FAST-LIO、EKF 和 RealSense。
4. 检查相机、CameraInfo、FAST-LIO、EKF、TF 和时间戳。
5. 启动 Windows OpenVLA 服务和地面站后端，保持 `CONTROL_OUTPUT_ENABLED=false`。
6. 启动机载 observation uplink 与 preview-only bridge。
7. 创建 `dry_run` 任务，检查 K 帧推理和 `/vla/preview_goal`、`/vla/preview_yaw`。
8. 停止任务，确认图像/位姿继续上传，但 preview 序号停止增长。

### 推荐初始通过标准

- 原始相机稳定，不低于 10 Hz；Windows 接受帧不低于 5 Hz。
- FAST-LIO/EKF 连续更新，不低于 10 Hz，消息年龄低于 150 ms。
- 静止 60 秒位置漂移小于 0.10 m、偏航漂移小于 3°；超出则先定位原因，不进入下一阶段。
- K=5 推理无持续错误，ACK 为 `preview_published`，成功率大于 99%。
- preview 单步位移、垂直位移和偏航增量满足限制；异常值必须在桥接层被拒绝。
- `/setpoints_cmd` 不存在或无发布者；Diff-Planner、PX4Ctrl、multipoint 和 offboard 节点均未启动。
- 测试结束仍为 `armed=false`。

## 6. G2：拆桨规划器 shadow 测试

仍然拆桨，不启动 PX4Ctrl，不连接最终控制话题。

1. VLA 继续输出 `/vla/preview_*`。
2. Diff-Planner 只读取粗目标和地图，输出 `/vla/optimized_trajectory_preview`。
3. 对每个输入记录原始 VLA 粗目标、优化轨迹、碰撞检查结果、规划耗时和拒绝原因。
4. 人工检查坐标轴方向：前、左、上和正偏航是否与 ENU/FLU 定义一致。
5. 移动障碍物或遮挡相机，确认规划器能拒绝危险轨迹，而不是维持旧目标。

通过条件：连续至少 20 个测试目标无坐标系反向、无穿障、无超限，规划失败时输出 HOLD/拒绝而不是复用陈旧轨迹。

## 7. G3：人工悬停旁路观察

这是第一次有桨测试。VLA 和 Diff-Planner 必须保持 preview/shadow，不能连接控制器。

1. 安全飞手人工解锁、人工起飞至约 1 m，并保持 Position 模式悬停。
2. 软件观察员确认图像、位姿、地图、VLA 和优化轨迹在飞行状态下仍连续。
3. 分别测试机身小幅偏航和横移，核对图像、EKF、地图与 preview 方向一致。
4. 人工降落、上锁后复盘日志。

通过条件：人工悬停稳定，定位无跳变，链路无长时间中断，preview 不出现持续饱和或坐标系反向。

## 8. G4：仅 Diff-Planner 低速控制

本阶段不使用 VLA 自动目标。目标由飞行负责人手工给出，目的是单独验证 Diff-Planner 与 PX4Ctrl。

约束建议：

- 高度约 1 m。
- 单段水平距离不超过 0.5 m。
- 最大速度先限制在 0.3 m/s 左右。
- 每次只执行一个目标，完成后进入 HOLD，等待人工确认下一目标。
- 安全飞手可随时切回 Position 模式。

必须单独验证 Offboard 心跳和丢失保护。PX4 官方要求 Offboard proof-of-life 持续高于 2 Hz，丢失后的行为由 `COM_OF_LOSS_T` 和 `COM_OBL_RC_ACT` 决定；实际控制 setpoint 应使用更高频率并持续监控。

## 9. G5：单次 VLA 粗目标实飞

只有 G4 完全通过后才能进入本阶段。

1. 飞行负责人输入一条非常简单的单步指令。
2. OpenVLA 只产生一个受限粗目标，不直接产生电机或姿态控制量。
3. 桥接层检查时间戳、calibration ID、frame ID、单步距离、高度和偏航限制。
4. Diff-Planner 根据实时地图优化并输出轨迹。
5. 软件界面显示待执行目标；飞行负责人进行一次人工确认后才放行。
6. 到达目标后强制 HOLD，不自动继续第二个目标。

首次任务应是空旷环境中的短距离平移，不进行绕杆、穿越、贴近障碍或连续复杂指令。

## 10. G6：K 帧连续 VLA 任务

只有多次 G5 无异常后才启用连续模式。

- VLA 仍是低频粗目标生成器；Diff-Planner 和飞控负责高频局部规划与控制。
- 同一时刻只允许一个有效目标，新的目标通过 sequence、TTL 和 mission ID 去重。
- VLA 推理期间持续使用当前安全轨迹，但目标过期后必须 HOLD，不得无限执行旧目标。
- 任务完成、用户停止、图像超时、位姿超时、规划失败或模型错误都进入 HOLD/人工接管流程。
- 逐步增加任务长度；每次只放宽一个变量，例如距离、速度或障碍复杂度。

## 11. 立即终止条件

出现下列任一情况，软件观察员喊 `接管`，安全飞手立即切回人工模式并降落：

- FAST-LIO/EKF 位姿跳变、坐标轴反向或明显漂移。
- 图像冻结、时间戳倒退、机载与主机时钟偏差超限。
- VLA/规划器输出 NaN、超限目标、旧 sequence 或错误 frame/calibration ID。
- Wi-Fi、后端、bridge、Diff-Planner 或控制器状态不确定。
- 飞机出现振荡、漂移、非指令偏航、快速升降或接近围栏。
- 遥控器、电池、GPS/定位、QGroundControl 或 PX4 报警。
- 场地有人或障碍物进入警戒区。

除非继续转动电机会造成更严重的即时伤害，不应在空中直接 disarm；优先切回人工模式并受控降落。

## 12. 每架次结束检查

1. 确认降落、上锁、电机停止后再接近飞机。
2. 停止任务，记录最终 mission state。
3. 保存 rosbag、PX4 ULog、Windows 后端日志、模型日志和视频。
4. 记录输入频率、推理频率、规划耗时、控制频率、最大延迟、丢帧、拒绝和 failsafe 事件。
5. 对比目标轨迹、优化轨迹和实际轨迹。
6. 形成结论：通过、带条件通过或失败。失败项关闭前不得进入下一门禁。

## 13. 单次测试记录模板

```text
日期/场地：
飞行负责人/安全飞手/软件观察员：
主仓库 commit：
机载仓库 commit：
OpenVLA 模型路径/哈希：
PX4 固件版本：
calibration ID：
时钟偏差：
测试门禁：G1 / G2 / G3 / G4 / G5 / G6
任务指令：
高度/速度/距离限制：
图像/位姿/VLA/规划/控制频率：
最大端到端延迟：
异常与人工接管：
结论：通过 / 带条件通过 / 失败
下一步：
```

## 14. 官方安全依据

- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard) 要求在切入前持续提供 proof-of-life，并在信号丢失后按 Offboard-loss 参数执行故障保护。
- [PX4 Safety/Failsafe Configuration](https://docs.px4.io/main/en/config/safety) 覆盖 RC、数据链、低电量、地理围栏和 Offboard 丢失等场景。
- 参数名称和可选值会随 PX4 版本变化，现场应以飞机实际固件和 QGroundControl 显示为准，先记录再修改，并先在仿真/无桨环境验证。
