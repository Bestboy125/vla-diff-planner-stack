# 旋转收尾与降落交接修复（2026-09-09）

## 原因

09:50:41 绕飞报告成功后，atomic_skill_executor 仍以 20 Hz 发布最后的
`yaw_target`。traj_server 的末端航向收敛因而一直认为收到新鲜目标，持续输出
PositionCommand。新的单次 yaw 被覆盖；PX4Ctrl 留在 CMD_CTRL，拒绝多次 LAND。

## 修改

- 原子技能成功、失败、抢占和异常退出均释放 yaw，发布与释放使用同一把锁。
- 末端航向收敛增加目标误差/角速度结束条件，以及 10 秒总时限。
- 操作员左右旋转使用独立 `operator_rotation` 通道：固定当前 world XYZ，
  以当前测量航向开始、沿原有角速度/角加速度限幅转向。普通 yaw 与迟到轨迹
  不得覆盖该通道；指令租约 0.5 秒，Bridge 最长运行 15 秒。
- Bridge 用新鲜里程计确认角误差小于 0.07 rad 持续 0.5 秒；超过 0.5 m
  位置偏差、失去新鲜 FCU/里程计/PX4Ctrl 状态则停止。手动接管不被覆盖。
- 降落先取消任务并通知 traj_server 暂停输出；等待暂停确认、PX4Ctrl
  AUTO_HOVER 状态以及连续至少 0.7 秒没有 PositionCommand，再发送一次 LAND。
- 等待悬停最多 10 秒；发送后 3 秒内必须看到 AUTO_LAND；之后以 FCU
  未解锁状态作为结束回报。无确认不自动重发；总降落等待上限 120 秒。
- HOLD/STOP 可以取消待执行交接。降落期间/失败后保持输出暂停，防止旧目标复活；
  必须在确认未解锁且 FCU 状态新鲜时，新的显式 TAKEOFF 才能释放该暂停。
- 回传带 task_id 的 `landing_wait_hover`、`landing_requested`、`landing`、
  `land_complete`、`land_failed`、`land_cancelled`、`rotation_started`、
  `rotation_complete`、`rotation_failed`、`rotation_cancelled`。
  沿现有观测上行到地面站任务 runtime；日志增加 command 和 task_id。

## 机载部署涉及文件

- `src/integration/atomic_skill_executor/scripts/atomic_skill_server.py`
- `src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py`
- `src/diff_planner/plan_manage/src/traj_server.cpp`
- `src/realflight_modules/px4ctrl/src/px4ctrl_node.cpp`

PX4Ctrl 仅增加 10 Hz `/px4ctrl/fsm_state` 只读状态发布，**没有放宽
AUTO_LAND 的准入条件，也没有更改其解锁、模式切换、起降控制逻辑**。
对应入口源码在主仓库 `onboard_control_stage/px4ctrl/src/px4ctrl_node.cpp` 保存。
机载 traj_server 与本地历史版本有 dt 差异，部署时保留了机载原有计算。

备份目录：`/home/nv/control-handoff-backup-20260909/`，包含改前源码及两个运行二进制。
原有启动命令不变；必须在落地、未解锁时重启整套控制栈，不能只热更新 Bridge。
未重新启动前不要混用新旧组件。未启动飞控或发布实机动作进行测试。

## 离线验证

- Bridge unittest（包括交接确认、拒绝、超时、单次发送、旋转完成/漂移）。
- 原子技能实际 execute/publish_yaw 方法测试（成功及异常后不再发布）。
- 机载编译 `traj_server_heading_hold`、`traj_server_yaw_test`、`px4ctrl_node`。
- C++ 实际回调测试：旧 yaw 不能抢旋转、测量航向初始化、租约失效、
  暂停门限、末端收敛时限以及原平移航向规则。

离线通过不代表已经完成实飞验证。后续仍需人工按安全流程验证悬停旋转、
绕飞结束后旋转、降落交接，以及手动接管/失联时的停止行为。
