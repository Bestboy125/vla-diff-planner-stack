# 项目分支约定

## `simulation-validated`

保存此前在 Windows Isaac Sim + WSL AirStack/PX4 SITL 环境中跑通的 K 帧 OpenVLA 闭环、相机/里程计桥、避障验证和录制脚本。该分支只用于复现实验、回归和算法对比。

## `real-deployment`（当前）

用于机载 FAST-LIO、真实 CameraInfo/TF、主机 K 帧 VLA 推理与 Diff-Planner 隔离规划预览的适配。当前阶段禁止解锁、起飞和运动控制；`CONTROL_OUTPUT_ENABLED=false`、`live_publish_enabled=false`。

机载部署单元固定为 Diff-Planner 子模块中的 `src/integration/vla_diff_bridge`，真实分支不要求覆盖 Diff-Planner 核心源码。

`Diff-Planner` 作为子模块保存自己的同名分支。切换根分支后应同步子模块到根提交记录的版本：

```bash
git submodule update --init --recursive
```

模型权重、数据集、录屏、日志、虚拟环境和 Isaac 缓存不进入 Git。
