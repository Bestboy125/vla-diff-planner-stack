# VLA Ground Station

本地主机控制台由 FastAPI 后端和 React/Vite 前端组成。默认只允许 dry-run，实机控制输出保持锁定。

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
- 两者统一返回机体系第一步动作 `[dx, dy, dz, d_yaw]` 和任务坐标系目标。
- `/ws/status` 向浏览器推送后端、任务、安全锁和模型健康状态。

允许 Windows 防火墙指定网络配置文件的 TCP 8080 后，操作笔记本可访问
`http://<HOST_OPERATOR_IP>:8080`。

环境变量：

- `OPENVLA_URL`，默认 `http://127.0.0.1:5007`
- `PI05_HOST` / `PI05_PORT`，默认 `127.0.0.1:8000`
- `CONTROL_OUTPUT_ENABLED`，默认 `false`；当前不得改为 `true`
- `ONBOARD_BRIDGE_HOST` / `ONBOARD_BRIDGE_PORT`，默认 `127.0.0.1:50051`
- `ONBOARD_BRIDGE_TOKEN`，与机载 `VLA_BRIDGE_AUTH_TOKEN` 相同，默认 `REQUIRED`
- `ONBOARD_COMMAND_TTL_MS`，默认 `500`

当前 OpenVLA 和 π0.5 都只执行 dry-run 推理，不会把轨迹发往机载电脑。

同时运行两种模型时，先在一个 PowerShell 终端启动：

```powershell
.\pi05\start_pi05_server.ps1
```

再在另一个终端启动网页与 OpenVLA：

```powershell
.\ground_station\start_ground_station.ps1 -WithOpenVLA
```
