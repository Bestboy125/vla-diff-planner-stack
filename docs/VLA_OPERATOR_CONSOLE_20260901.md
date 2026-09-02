# VLA 实机操作台增量说明

## 1. 数据流

机载 `onboard_observation_uplink_node.py` 持续上传 JPEG、CameraInfo、body←camera 外参和 FAST-LIO/EKF odom。主机 FastAPI 接收后保留最新帧，并通过 `/api/onboard/stream.mjpeg` 向浏览器提供连续视频；`/ws/status` 以默认 5 Hz 推送位姿、帧龄、接收帧率、模型状态和任务状态。

连续 VLA 任务仍采用每 K 个新图像序号推理一次。Dry-run 输出 schema v2 `planning_preview`；只有 mission 为 live 且主机 `CONTROL_OUTPUT_ENABLED=true` 时才输出 schema v1 `trajectory_command/TRACK`。机载桥仍会再次检查 TTL、序号、坐标系、步长、当前 odom 新鲜度和自身 live 开关。

## 2. 原子任务

网页支持起飞、降落、悬停、前后左右上下移动和左右旋转。主机把选择转换为 schema v3 `operator_task`。机载端基于最新机体系 FLU 位姿完成转换：前为 +X、左为 +Y、上为 +Z；平移目标旋转到 world/ENU 后发布给 Diff-Planner，旋转任务保持位置只改变 yaw。

起飞/降落只发布 `quadrotor_msgs/TakeoffLand` 给 PX4Ctrl，不包含 MAVROS arming 或 mode 命令。起飞高度必须与机载 PX4Ctrl 配置高度在 0.05 m 内一致。平移单步上限默认 1.0 m，旋转上限默认 90°，目标高度必须在 0.1–2.0 m 范围内。

## 3. 具身任务

前端提供三种入口：自由自然语言、按指定半径/方向/圈数绕目标、飞过指定目标后继续前进给定距离。后两种由后端规范化成明确指令，再建立 OpenVLA 或 π0.5 mission。目标识别、连续动作预测和完成判断仍依赖模型与后续任务状态机；当前停止由操作员按钮触发。

## 4. 安全门

默认双击启动只设置 `CONTROL_OUTPUT_ENABLED=false`。Live 请求同时要求：

1. 主机使用 `-EnableLiveControl -LiveControlConfirmation I_ACCEPT_REAL_FLIGHT_CONTROL` 启动。
2. 浏览器提交正确 `OPERATOR_CONTROL_TOKEN` 和确认短语。
3. 机载启动脚本设置 `VLA_BRIDGE_MODE=live`。
4. 机载设置 `ENABLE_VLA_LIVE_CONTROL=I_ACCEPT_VLA_AND_OPERATOR_GOAL_PUBLICATION`。
5. 机载桥同时设置 `live_publish_enabled=true`、`preview_only_mode=false`、`operator_task_enabled=true`。

任一条件缺失都会返回 locked/rejected，不发布控制目标。

## 5. 启动文件

- `ground_station/start_vla_backend.cmd`：双击启动 OpenVLA。
- `ground_station/start_vla_backend.ps1`：支持 OpenVLA、π0.5 或 Both，等待健康检查并记录 PID。
- `ground_station/stop_vla_backend.ps1`：只停止上述脚本记录的进程。
- `ground_station/start_operator_console.cmd`：双击启动网页后端并打开浏览器，默认 dry-run。
- `ground_station/start_operator_console.ps1`：加载私有本地配置、构建前端并启动 FastAPI。
- `ground_station/ground_station.local.ps1.example`：不含真实密钥的配置模板。
- `Diff-Planner/sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh`：机载统一启动脚本，默认 preview；live 必须显式双重确认。

## 6. 本轮验证边界

只执行了 Python/PowerShell 语法检查、React 生产构建、FastAPI dry-run 单元测试、纯协议测试和 ROS launch 静态解析。本轮没有下发或执行任何起飞、降落、移动、旋转、绕飞或 VLA 具身任务，也没有执行 MAVROS 解锁和模式切换。
