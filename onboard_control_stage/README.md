# 2026-09-09 机载控制修复快照

`px4ctrl/src` 保存本次机载 PX4Ctrl 两个修改文件，适用于现有的
`Diff-planner/src/realflight_modules/px4ctrl/src`，不是完整独立 ROS 包。

扫描最新源码保存于主仓库 `onboard_scan_stage/semantic_scan_orbit_mission`；
Bridge 最新源码在 `Diff-Planner` 子模块。不要将 `tmp` 或旧备份用作部署源。
本次归档不改变机载运行状态。

回归测试（无 ROS master、无飞行动作）：

```text
python -m unittest discover -s onboard_control_stage/tests
```

覆盖扫描连续航点、返线稳定与航向对齐、取消、地图过期/坐标系检查、障碍航点
跳过及连续跳过限制、起飞结果与降落状态到达顺序。

换行采用固定 X、沿 +Y 连接，保留每条 6 m 扫描线，移除旧内凹曲线的额外 X
折返。被占用航点不计作到达，任务完成回报包含 skipped_waypoints 和 coverage_complete。
