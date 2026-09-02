# Isaac Sim 4.5 服务器端无界面部署

本目录用于在无桌面的 GPU 服务器上验证 Isaac Sim 的 RTX 渲染，以及产出可供 VLA/规划链路消费的 RGB、深度和相机位姿样例。

## 当前服务器状态（2026-08-27）

- 已下载并通过 ZIP CRC 校验：Isaac Sim 4.5.0 standalone。
- NFS 永久副本：`/opt/data/private/isaac-sim/4.5.0`。
- 本地 SSD 运行副本：`/opt/isaac-sim-4.5.0`。
- 已完成 `post_install.sh`。
- 两张 RTX A6000、驱动 550.127.05、CUDA 计算均可见。
- 已补齐与驱动版本一致的 NVIDIA GL/Vulkan 用户态库和 X11 运行库。
- 当前阻塞：SSH 登录到的是 Kubernetes 容器；容器只获得了 CUDA/utility 设备，没有 `/dev/nvidia-modeset` 的 cgroup 权限。因此 `vulkaninfo` 返回 `VK_ERROR_INCOMPATIBLE_DRIVER`，Isaac Sim 无法建立 RTX 渲染上下文。

这不是“服务器没有显示器”造成的。Isaac Sim 支持 headless；真正缺少的是外层容器的 NVIDIA `graphics` 能力。

## 需要服务器平台方调整

请让平台管理员重建或调整实例，使 NVIDIA Container Toolkit/Device Plugin 向容器授予 `graphics` 能力，并满足以下验收条件：

1. 容器内存在 `/dev/nvidia-modeset`。
2. 容器进程可以打开该设备，而不只是看见文件节点。
3. NVIDIA 550.127.05 图形用户态库由运行时正确注入。
4. 以下命令能够枚举 RTX A6000：

   ```bash
   VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json vulkaninfo --summary
   ```

仅在容器内运行 `mknod` 不够，因为 Kubernetes devices cgroup 会拒绝访问。平台配置中通常需要 `NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics` 或 `all`，但最终以以上四项验收结果为准。

## 权限恢复后的运行方法

把本目录复制到服务器，例如 `/opt/data/private/isaac-sim/project/isaac_sim_server`，然后执行：

```bash
cd /opt/data/private/isaac-sim/project/isaac_sim_server
chmod +x preflight.sh run_rgbd_smoke.sh
./preflight.sh
GPU_ID=0 ./run_rgbd_smoke.sh
```

成功后会生成：

- `/opt/data/private/isaac-sim/artifacts/rgbd/rgb.png`
- `/opt/data/private/isaac-sim/artifacts/rgbd/depth.npy`
- `/opt/data/private/isaac-sim/artifacts/rgbd/depth_preview.png`
- `/opt/data/private/isaac-sim/artifacts/rgbd/metadata.json`

`metadata.json` 同时记录图像尺寸、深度有效像素范围及相机世界位姿，可作为后续 Isaac Sim → VLA/后端协议适配的基准样例。

## 与现有系统的边界

本冒烟测试只验证仿真渲染与观测数据，不会启动真实无人机控制，也不会向 Diff-Planner 发布轨迹。整套闭环接入时应继续保持：Isaac Sim 提供仿真传感器和机体状态，主机 VLA 后端产生短时域轨迹，Diff-Planner 负责安全约束与在线轨迹修正；任务停止信号必须经过后端状态机和机载安全层确认。
