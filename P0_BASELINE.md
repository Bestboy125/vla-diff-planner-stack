# P0 离线基线

在连接无人机控制前，必须完成以下检查。

## 1. OpenVLA 基准

使用 OpenVLA 虚拟环境运行：

```powershell
cd <OPENVLA_PROJECT_ROOT>
.\.venv\Scripts\python.exe .\vla-scripts\benchmark_openvla.py `
  --image <真实RGB图像路径> `
  --instruction "向前飞行并与障碍物保持安全距离" `
  --proprio 0 0 0 0 `
  --warmup 2 `
  --runs 100 `
  --output <PROJECT_ROOT>\artifacts\openvla_real_benchmark.json
```

验收条件：

- 输出形状为 `[4]`，且语义为 `[dx_body, dy_body, dz_body, d_yaw]`。
- 所有输出均为有限值。
- `bound_violation_count` 为 0。
- 记录模型加载时间、p50/p95/max 延迟和峰值显存。

## 2. 机载 ROS 契约

在机载电脑执行并保存结果：

```bash
rostopic list
rostopic info <RGB_TOPIC>
rostopic info <ODOM_TOPIC>
rostopic hz <RGB_TOPIC>
rostopic hz <ODOM_TOPIC>
rosrun tf view_frames
```

把结果填写到 `deployment_contract.yaml`。所有 `REQUIRED` 字段填写完毕前，不连接飞控命令。
