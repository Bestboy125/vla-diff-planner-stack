# 2026-09-09 实飞后修复与三端部署

## 实测日志

机载日志目录：`/home/nv/.ros/log/3596763a-ac34-11f1-a5dc-3c6d662bcdfc/`。

- 扫描：17:55:29 出现 `ACTIVE when in simple state DONE`，17:55:32 出现 `SimpleActionClient received DONE twice`。
  原完成回调立即发送新目标，随后 actionlib 将新目标状态覆盖为 DONE，后续结果无法正常驱动航线。
- 第二次起飞：17:56:43 PX4 返回 ARM rejected，但原 PX4Ctrl 仍进入 AUTO_TAKEOFF，后续请求不能重新进入起飞分支。
  日志的 `Kill-switch activated?` 是通用提示，不能据此断定具体解锁拒绝原因；未解除任何飞控检查。
- 现有页面缺少独立停止按钮；HOLD 原分支没有直接广播原子 action 取消，扫描状态异常时更难完成取消。
- 降落结束时 PX4Ctrl 的 MANUAL_CTRL 状态先于 MAVROS 的 disarmed 更新到达，可能误报手动接管。

## 本次修改

- 前端：独立停止按钮，不受普通请求 busy、选中任务参数或图像上行状态限制；保留 token/live 确认检查。
- 后端：`POST /api/tasks/stop`，只构造 HOLD；默认 dry-run。前端显示最近四项任务的运行状态，便于同时看到停止请求和原任务取消结果。
- Bridge：取消全部语义执行器和底层 atomic action，再发布可恢复悬停；等待任务取消回报，5 秒无确认明确报告 cancel_unconfirmed 并阻止新运动任务。
- Bridge：增加起飞请求/完成/失败反馈；必须 fresh MANUAL_CTRL 且未解锁才能请求新起飞；不自动重试。
- 扫描：不在 action 回调中发送新航点；定时器确认 SimpleGoalState.DONE 后推进，增加航点超时及实时飞行状态检查。
- PX4Ctrl：模式切换或解锁请求失败即退出起飞状态，不再错误地停留在 AUTO_TAKEOFF；保留所有 RC、静止、落地和飞控安全检查。
- 降落：已进入降落流程时允许最多 1 秒等待 disarmed 消息，避免消息到达顺序造成误报，不重发 LAND。

## 部署范围与备份

- Windows：本仓库 `ground_station/backend/app`、`frontend/src`、`frontend/dist` 已更新；没有启动 Windows 实机控制服务。
- Dell：`/home/oem/vla-ground-station/ground_station` 同步源文件和构建产物。
  新镜像 `vla-ground-station:field-repair-20260909`，运行配置保留；旧容器保留为 `vla-ground-station-before-field-repair-20260909`。
  源文件备份 `/home/oem/field-repair-20260909/ground-station-before.tar.gz`，包含原配置，应私密保管。
- 机载：只部署到 `/home/nv/Diff-planner`，未将板载运行组件部署至 Dell。
  三处文件：`src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py`、
  `src/perception/semantic_scan_orbit_mission/scripts/semantic_scan_orbit_node.py`、
  `src/realflight_modules/px4ctrl/src/PX4CtrlFSM.cpp`。
  原源码与二进制备份 `/home/nv/field-repair-20260909/backup`；测试保存于相邻 `tests` 目录。
- 提交归档时已将机载修复同步至主仓库 `onboard_scan_stage`、`onboard_control_stage` 和 `Diff-Planner` 子模块。`tmp/repair_*` 仅为编辑副本，不是部署源。

## 验证及生效边界

Windows 后端 74 项、前端 10 项测试通过；前端生产构建成功。
机载新增 7 项修复测试、9 项交接测试通过，Python 语法检查通过，PX4Ctrl 编译退出码为 0。
三端对应文件做 SHA256 核对。

没有发送实际飞行命令或绕过解锁检查。机载进程未热重启，必须由操作者在确认落地、未解锁后完整重启原控制栈再验证。
离线测试不等同于实飞验证；PX4 具体 prearm 拒绝原因仍需在再次拒绝时检查飞控状态消息。
