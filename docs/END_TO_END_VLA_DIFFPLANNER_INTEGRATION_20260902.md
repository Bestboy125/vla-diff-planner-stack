# VLA—Diff-Planner 前后端联调技术方案

日期：2026-09-02  
适用分支：`real-deployment`  
当前阶段：实机安全预览链路已验证；飞行控制闭环尚未验证

## 1. 目标与边界

系统要形成如下闭环：浏览器提交自然语言任务，Windows 主机持续接收机载 RGB 图像和
FAST-LIO/EKF 状态，每隔 K 个新图像帧调用一次 OpenVLA 或 π0.5，将模型动作转换为 ROS
坐标系下的局部目标，再交给机载 Diff-Planner 做避障和实时轨迹优化。执行过程中新的图像、
位姿和规划状态继续反馈给 VLA，直到任务完成、操作员停止或安全机制中止任务。

当前归档版本只验证安全预览路径。它不会发送 MAVROS 解锁、飞控模式切换、起飞或自动目标；
Windows 的 `CONTROL_OUTPUT_ENABLED=false`，机载桥为 `preview_only_mode=true`。

## 2. 系统组成

```text
操作员笔记本浏览器
  │ HTTP / WebSocket / MJPEG（<HOST_OPERATOR_IP>:8080）
  ▼
Windows 主机 FastAPI + React
  ├─ OpenVLA HTTP 服务（127.0.0.1:5007）
  ├─ π0.5 OpenPI WebSocket 服务（127.0.0.1:8000）
  ├─ K 帧推理、任务状态机、坐标变换
  └─ TCP/NDJSON 命令客户端（<HOST_ONBOARD_IP> → <ONBOARD_IP>:50051）
         ▲                                      │
         │ HTTP 观测上行                        ▼
机载 observation_uplink                机载 vla_diff_bridge
  ├─ RGB / CameraInfo                    ├─ 协议、时效、序号、坐标校验
  ├─ /ekf/ekf_odom                       ├─ preview：隔离目标话题
  ├─ TF body←camera                      └─ live：/goal + /planning/yaw
  └─ 规划预览反馈                                  │
                                                   ▼
                                      Diff-Planner → traj_server → PX4Ctrl
                                                   ▲
                       FAST-LIO 点云、EKF 位姿、MAVROS/PX4 状态
```

网络采用双接口隔离：操作端访问 Windows 的 `<HOST_OPERATOR_IP>:8080`；机载数据面使用
Windows `<HOST_ONBOARD_IP>` 与机载电脑 `<ONBOARD_IP>`。浏览器不直接访问机载电脑，也不持有
机载 TCP bridge token。

## 3. 前端和 Windows 后端

### 3.1 浏览器前端

React 页面负责：

- 通过 `/api/onboard/stream.mjpeg` 显示机载连续 RGB 视频。
- 通过 `/ws/status` 接收约 5 Hz 的图像序号、帧龄、接收 FPS、位姿、模型和任务状态。
- 通过 `/api/tasks/dispatch` 提交原子任务或 VLA 具身任务。
- 显示 `dry_run/live`、策略、任务状态和安全锁，不直接生成 ROS 消息。

原子任务包括起降、悬停、六方向平移和左右旋转；具身任务包括自由指令、按半径绕目标和
飞过目标后继续前进。当前归档仅使用 `dry_run`，不点击或执行这些动作。

### 3.2 FastAPI 后端

FastAPI 是唯一控制面入口，职责包括：

- 校验浏览器任务参数并维护单任务状态机。
- 校验机载观测 token、时间戳、标定 ID、坐标系和图像完整性。
- 管理 K 帧节流和单实例异步推理，避免模型推理阻塞图像接收。
- 统一 OpenVLA 和 π0.5 输出为 `[dx_body, dy_body, dz_body, d_yaw]`。
- 生成预览或实机轨迹消息，并等待机载 ACK。
- 向前端提供视频、状态和非敏感诊断信息。

任务状态主链为 `ARMED → RUNNING → HOLDING/SUCCEEDED/ABORTED/FAULT`。目前代码不会仅凭
模型输出自动认定复杂自然语言任务已经完成；正式实机阶段应增加任务完成检测器。在此之前，
任务结束依赖操作员停止或后端故障/超时状态，不能把“模型不再输出明显位移”当作完成信号。

## 4. 机载观测上行

`onboard_observation_uplink_node.py` 订阅：

- `/vla_usb_camera/image_raw/compressed`，或回退到 raw 图像后编码 JPEG；
- `/vla_usb_camera/camera_info`；
- `/ekf/ekf_odom`；
- `base_link ← vla_usb_camera_optical_frame` TF；
- 可选的 `/vla/optimized_trajectory_preview`。

相机回调只把最新数据放入容量为 2 的覆盖队列。HTTP 变慢时丢弃旧帧，不反压 USB 相机
图像流。工作线程组装 `onboard_observation` JSON，通过
`POST /api/onboard/observations` 上传，并在 `X-Observation-Token` 中携带认证信息。

后端只接受满足以下条件的观测：

- `sequence` 单调递增；
- 图像时间距主机当前时间不超过 750 ms，未来偏差不超过 1000 ms；
- 图像与 odom 时间差不超过 80 ms；
- `world / base_link / vla_usb_camera_optical_frame` 与主机配置一致；
- 四元数归一化，JPEG 首尾完整；
- CameraInfo、外参和 `calibration_id` 已人工确认且两端一致。

标定 ID 必须使用部署时人工确认的 `<VALIDATED_CALIBRATION_ID>`。外参由机载 TF 发布，
严禁在主机侧再次旋转图像或重复应用外参。

## 5. K 帧连续推理

默认 `K=5`。每收到一个合格的新图像序号，后端计数一次；第 K 帧且当前没有推理任务时，
以该帧和同拍位姿启动推理。推理期间图像上行继续，后端始终保留最新帧。模型空闲后使用
下一批最新 K 帧触发，不排队执行过时图像。

模型输入统一为：

```text
RGB: 当前选中的 JPEG 帧
instruction: 当前 RUNNING mission 的文本指令
proprio: [x_world, y_world, z_world, yaw_deg]
```

OpenVLA 使用 HTTP `/predict`；π0.5 使用 OpenPI WebSocket/MessagePack。π0.5 返回 action
chunk 时只取第一步执行或预览，剩余 chunk 不跨感知周期盲目执行。下一次 K 帧到达后重新推理，
形成 receding-horizon 闭环。

如果没有 `RUNNING` mission，第 K 帧仍会被接收，但推理被跳过并显示
`Kth frame received; no RUNNING mission, inference skipped`，这是正常安全状态。

## 6. 坐标系和动作转换

协议固定使用：

- `world`：FAST-LIO/EKF 和 Diff-Planner 共用的右手、Z 向上的局部世界系；
- `base_link`：ROS FLU，X 前、Y 左、Z 上；
- `vla_usb_camera_optical_frame`：X 右、Y 下、Z 前；
- 图像动作语义：`[dx_body, dy_body, dz_body, d_yaw]`，单位 `[m,m,m,rad]`。

主机利用最新位姿的 yaw 将机体系增量转为绝对世界目标：

```text
x_target = x + cos(yaw)·dx_body - sin(yaw)·dy_body
y_target = y + sin(yaw)·dx_body + cos(yaw)·dy_body
z_target = z + dz_body
yaw_target = wrap(yaw + d_yaw)
```

机载桥再次检查目标相对当前 odom 的三维步长，默认不得超过 1.0 m；目标高度必须在
0.1–2.0 m。这样模型输出即使发生尺度异常，也不能绕过机载侧边界。

## 7. Windows 到机载的轨迹协议

主机向 `<ONBOARD_IP>:50051` 建立 TCP 连接，每条消息是一行 UTF-8 JSON（NDJSON）。消息分为：

- schema v2 `planning_preview`：只进入隔离预览话题；
- schema v1 `trajectory_command`：live 模式的 `TRACK/HOLD/COMPLETE/EMERGENCY_STOP`；
- schema v3 `operator_task`：网页选择的原子任务。

每条消息包含 mission/task ID、严格递增 sequence、发送时间、TTL、frame、策略、动作语义、
局部动作和绝对目标。机载端校验来源 IP、token、TTL、序号、mission 连续性、标定 ID、观测
新鲜度和 odom 新鲜度，并返回匹配 ID 与 sequence 的 ACK。主机不能用同一个 sequence
盲目重发；超时或 rejected 应进入 HOLD/FAULT，并使用新的 sequence 进行后续恢复。

当前关键时效边界：

- 主机轨迹 TTL：500 ms；
- 机载允许的 source observation age：1000 ms；
- 机载 odom 最大年龄：250 ms；
- live 命令流 watchdog：1000 ms；
- TCP 单连接 socket timeout：2 s。

## 8. Diff-Planner 接入方式

### 8.1 当前安全预览模式

机载桥将 VLA 输出发布到 `/vla/preview_goal` 和 `/vla/preview_yaw`，而真实规划器仍监听
`/goal`。控制目标、hover-stop 和 mandatory-stop 均 remap 到 `/vla/disabled/*`，因此不会
进入真实控制链。

当前一键脚本中的 `start_diff_planner_preview:=false`，所以已验证的是“VLA 结果到达隔离预览
话题”的通信预览，而不是“隔离 Diff-Planner 已输出优化轨迹”。网页中的
`planner_preview=null` 在这种配置下是预期现象。

若要在不运动的条件下验证完整优化结果，应单独启动隔离的 Diff-Planner 实例，将：

```text
/goal          → /vla/preview_goal
/planning/yaw  → /vla/preview_yaw
/position_cmd  → /vla/optimized_trajectory_preview
```

同时确保 `/vla/optimized_trajectory_preview` 没有 PX4Ctrl subscriber。隔离实例必须使用独立
namespace/node 名称，不能与真实 `drone_0_diff_planner_node` 和 traj_server 同名。完成这一点前，
不应把当前页面称为“优化轨迹预览已闭环”。

### 8.2 目标 live 模式

live 模式下，桥把经过双重校验的绝对目标发布到 `/goal` 和 `/planning/yaw`。Diff-Planner
结合 `/laserMapping/cloud_registered`、`/ekf/ekf_odom` 和当前局部地图重新规划，traj_server
将连续 `quadrotor_msgs/PositionCommand` 输出到 `/setpoints_cmd`，由 PX4Ctrl 跟踪。

每次 K 帧推理只更新短步目标。Diff-Planner 在两次 VLA 推理之间以自己的高频 FSM、地图更新
和轨迹优化继续运行，因此形成“低频语义决策 + 高频避障规控”的分层闭环，而不是让 VLA
直接输出电机、姿态或速度控制量。

## 9. 安全门

从预览切换 live 必须同时满足：

1. Windows 使用 `-EnableLiveControl` 和固定确认短语启动；
2. `CONTROL_OUTPUT_ENABLED=true`；
3. 浏览器提供正确 operator token 和确认短语；
4. 任务本身为 `mode=live`；
5. 机载 `VLA_BRIDGE_MODE=live`；
6. `ENABLE_VLA_LIVE_CONTROL=I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION`；
7. `live_publish_enabled=true`、`preview_only_mode=false`；
8. MAVROS 已连接且启动时 `armed=False`；
9. multipoint 自动规划、自动降落继续保持关闭，除非另行进行专门审批和测试。

任何一项缺失都必须 fail closed。系统本身不负责 MAVROS arming 或 mode switch；这两项应继续
由独立、人工确认的飞行 SOP 完成，不能放进 VLA 一键脚本。

## 10. 建议的分阶段联调

### L0：离线和静态验证

- PowerShell、Bash、Python、ROS launch 语法检查；
- 前端生产构建和 FastAPI 测试；
- 协议 schema、坐标转换、TTL、乱序、NaN/Inf 测试；
- 扫描确认一键预览脚本不包含 arming、set_mode、起飞和自动目标。

### L1：观测链路

- 飞机保持未解锁；
- 启动 KINGSEN USB 相机、FAST-LIO、EKF 和观测上行；
- 检查视频连续性、帧龄、位姿、CameraInfo、TF 和两端时间差；
- 不创建 mission，不启动推理。

通过标准：图像序号持续递增、位姿更新、无 stale/sync/calibration 错误。

### L2：VLA dry-run

- 创建 `dry_run` mission；
- 每 K 帧调用 OpenVLA，再单独验证 π0.5；
- 检查动作尺度、方向、目标转换和 sequence；
- ACK 必须为 preview，真实 `/goal` 和 `/setpoints_cmd` 不得出现新增 publisher/data。

### L3：隔离 Diff-Planner 优化预览

- 只启动 namespaced preview planner，不连接 PX4Ctrl；
- 让 `/vla/preview_goal` 进入隔离 planner；
- 检查 `/vla/optimized_trajectory_preview` 连续输出并回传网页；
- 用障碍物点云验证轨迹会绕障，而非直穿障碍物。

### L4：桨叶卸除或系留条件下的控制接口验证

- 检查 live 四重安全门、ACK、HOLD 和 watchdog；
- 验证 `/setpoints_cmd` 的频率、frame、连续性和限幅；
- 只验证接口，不执行起飞和空间运动。

### L5：低风险实飞

- 空旷场地、人工遥控接管、急停和安全员就位；
- 先悬停和 0.2 m 单步，再逐渐增加任务复杂度；
- 最后才验证绕目标、飞过目标和持续 K 帧闭环。

每一级都应生成 rosbag、Windows 后端日志、模型延迟、ACK 统计和明确的通过/失败结论；前一级
未通过时不得进入下一级。

## 11. 启动和停止

Windows：

```powershell
cd <PROJECT_ROOT>
.\ground_station\start_vla_full_preview.ps1 -Policy OpenVLA
```

也可双击 `ground_station/start_vla_full_preview.cmd`。

机载：

```bash
cd ~/Diff-planner
./sh_files/start_onboard_vla_full_preview.sh
```

两个脚本都在前台监控其启动的子进程。停止时在对应终端按 `Ctrl-C`；脚本只停止自己启动的
进程。私有 token 保存在忽略提交的配置文件中，归档和日志不得包含其真实值。

## 12. 主要剩余难题

- 建立独立 namespace 的 Diff-Planner 预览实例，避免与真实 planner 节点重名。
- 为复杂具身任务增加可靠完成检测，而不是只依赖操作员停止。
- 对 OpenVLA/π0.5 输出进行数据集反归一化、动作尺度和方向的实机标定。
- 在 VLA 推理延迟波动时保证目标 TTL、watchdog 和 Diff-Planner 重规划连续性。
- 处理动态障碍、目标短时遮挡和语义目标重识别。
- 建立端到端 rosbag/日志关联 ID，使 observation、inference、command、ACK 和轨迹可追溯。
