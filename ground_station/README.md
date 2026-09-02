# VLA Ground Station

本地主机控制台由 FastAPI 后端和 React/Vite 前端组成。它显示机载 MJPEG 视频、FAST-LIO/EKF 位姿、K 帧 VLA 结果，并提供原子任务和 VLA 具身任务对话面板。默认只允许 dry-run，实机控制输出保持锁定。

## Windows 一键启动

双击 `start_vla_backend.cmd` 启动 OpenVLA 后端。需要切换模型时可在 PowerShell 中执行：

```powershell
.\ground_station\start_vla_backend.ps1 -Policy OpenVLA
.\ground_station\start_vla_backend.ps1 -Policy Pi05
.\ground_station\start_vla_backend.ps1 -Policy Both
```

复制 `ground_station.local.ps1.example` 为 `ground_station.local.ps1`，填入本机私有 token 和已验证标定 ID；该文件已被 Git 忽略。然后双击 `start_operator_console.cmd`，脚本会启动 FastAPI 并打开 `http://127.0.0.1:8080`。

## 后端

```powershell
cd <PROJECT_ROOT>\ground_station\backend
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m uvicorn app.main:app --host 0.0.0.0 --port 8080
```

## 前端开发

```powershell
cd <PROJECT_ROOT>\ground_station\frontend
npm install
npm run dev
```

## 生产式本地运行

先执行 `npm run build`，再启动 FastAPI。后端会直接提供 `frontend/dist`。

也可以在项目根目录直接运行：

```powershell
.\ground_station\start_ground_station.ps1 -WithOpenVLA
```

不加 `-WithOpenVLA` 时只启动网页后端，便于独立调试。

模型接口：

- `POST /api/inference/openvla` 代理本机 `5007` 端口的 OpenVLA 服务。
- `POST /api/inference/pi05` 使用 OpenPI 官方 MessagePack-over-WebSocket 协议访问 `8000` 端口。
- `POST /api/bridge/commands` 把共同动作格式封装为机载轨迹协议；安全锁开启时只预览、不发送。
- `POST /api/onboard/observations` 接收机载 JPEG、FAST-LIO odom、CameraInfo、TF 外参和隔离规划器状态。
- `GET /api/onboard/latest/image` 为网页提供不缓存的最新机载图像。
- `GET /api/onboard/stream.mjpeg` 提供连续机载视频流。
- `GET /api/tasks/catalog` 返回任务类型和参数边界。
- `POST /api/tasks/dispatch` 校验/调度原子任务或创建连续 K 帧 VLA 任务。
- 两者统一返回机体系第一步动作 `[dx, dy, dz, d_yaw]` 和任务坐标系目标。
- `/ws/status` 向浏览器推送后端、任务、安全锁和模型健康状态。

允许 Windows 防火墙指定网络配置文件的 TCP 8080 后，操作笔记本可访问
`http://<HOST_OPERATOR_IP>:8080`。

环境变量：

- `OPENVLA_URL`，默认 `http://127.0.0.1:5007`
- `OPENVLA_PROJECT_ROOT` / `OPENVLA_MODEL_PATH`，仅在本地私有配置中指定源码和权重位置
- `PI05_HOST` / `PI05_PORT`，默认 `127.0.0.1:8000`
- `PI05_WSL_USER` / `PI05_OPENPI_ROOT` / `PI05_CHECKPOINT_PATH`，仅在本地私有配置中指定 WSL 运行位置
- `CONTROL_OUTPUT_ENABLED`，默认 `false`。只有使用带确认参数的启动方式才可设为 `true`。
- `OPERATOR_CONTROL_TOKEN`，网页 live 请求的独立操作令牌。
- `LIVE_CONTROL_CONFIRMATION`，默认 `I_ACCEPT_REAL_FLIGHT_CONTROL`。
- `ONBOARD_BRIDGE_HOST` / `ONBOARD_BRIDGE_PORT`，默认 `127.0.0.1:50051`
- `ONBOARD_BRIDGE_TOKEN`，与机载 `VLA_BRIDGE_AUTH_TOKEN` 相同，默认 `REQUIRED`
- `ONBOARD_COMMAND_TTL_MS`，默认 `500`
- `ONBOARD_OBSERVATION_TOKEN`，观测上行独立密钥，默认 `REQUIRED`
- `OBSERVATION_K_FRAMES`，每累计多少个新图像序号执行一次 VLA，默认 `5`
- `OBSERVATION_MAX_AGE_MS` / `OBSERVATION_MAX_SYNC_ERROR_MS`，默认 `750/80`
- `EXPECTED_WORLD_FRAME` / `EXPECTED_BODY_FRAME` / `EXPECTED_CAMERA_FRAME`，默认 `world/base_link/camera_color_optical_frame`
- `EXPECTED_CALIBRATION_ID`，必须与机载已人工确认的标定 ID 一致，默认 `REQUIRED`

Dry-run 的 OpenVLA 和 π0.5 生成 schema v2 规划预览。Live 具身任务生成 schema v1 连续轨迹，原子任务使用 schema v3；三者均受主机和机载双端开关约束。系统不提供 MAVROS 解锁或模式切换接口。

同时运行两种模型时，先在一个 PowerShell 终端启动：

```powershell
.\pi05\start_pi05_server.ps1
```

再在另一个终端启动网页与 OpenVLA：

```powershell
.\ground_station\start_ground_station.ps1 -WithOpenVLA
```
