# 实机安全预览归档记录（2026-09-02）

## 归档范围

本归档对应 Windows 与机载端两个一键脚本均由操作员确认启动正常后的代码状态。归档包含：

- Windows OpenVLA/FastAPI/React 安全预览一键启动；
- 机载 ROS、MAVROS、FAST-LIO、EKF、RealSense、VLA bridge、Diff-Planner、PX4Ctrl 和
  multipoint 的安全预览启动编排；
- MAVROS 节点存在性和 `connected=True, armed=False` 等待修复；
- 端到端联调技术方案。

不包含私有 bridge、observation 或 operator token；这些值位于 Git 忽略的本地配置中。

## Git 基线

- 顶层仓库归档前提交：`0533040992a2258042adab249734980cd27e4394`
- Windows 本地 Diff-Planner 提交：`70bab68924ea5a475338e10fce819774cf93ed09`
- 机载 `<ONBOARD_WORKSPACE>` 提交：`e868f710273661f08536137264a14484c2d4c9de`
- 两个 Diff-Planner 提交因分别提交而 SHA 不同，但归档启动脚本内容一致。
- `start_onboard_vla_full_preview.sh` SHA-256：
  `123f09a8c81f9a75f19b1460c65f465052e9c543de11807780635743d05f97e8`

归档完成后，以 annotated tag `real-deployment-preview-verified-20260902` 标记顶层仓库，
以 `onboard-preview-verified-20260902` 标记 Windows Diff-Planner 子仓库和机载部署仓库。

## 已验证事实

- 两个一键启动脚本由操作员确认可正常启动。
- MAVROS 受控遥测测试约 2 秒达到 `connected=True, armed=False`。
- 机载脚本 Bash 语法和 Git diff 检查通过。
- Windows 脚本 PowerShell 语法检查通过。
- 观测上行曾验证图像序号持续增长、FAST-LIO/EKF 位姿更新、标定 ID 匹配和 OpenVLA 在线。

## 验证边界

该归档不是飞行验收记录。没有在本轮归档中执行：

- MAVROS arming 或 mode switch；
- 起飞、降落、平移、旋转、绕飞或具身任务；
- VLA live 目标下发；
- 隔离 Diff-Planner 优化轨迹闭环；
- PX4Ctrl 实际飞行跟踪。

当前 preview 配置将 VLA 目标隔离到 `/vla/preview_*`，且
`start_diff_planner_preview=false`。因此它证明通信、感知、模型和安全门技术栈可用，但不能证明
VLA 轨迹已经经过 Diff-Planner 优化，也不能证明实机运动安全。

## 恢复原则

恢复时优先从 annotated tag 创建新分支，不直接 detached HEAD 修改归档：

```bash
git switch -c recovery/preview-20260902 real-deployment-preview-verified-20260902
git submodule update --init --recursive
```

机载仓库可从 `onboard-preview-verified-20260902` 创建恢复分支。恢复私有配置时必须重新核对
IP、token、标定 ID 和 preview 模式，不能从文档或日志复制历史密钥。
