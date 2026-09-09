# 地面站笔记本部署包（Windows 10/11 x64）

包含已编译 React 前端、FastAPI 后端、一键安装/启动脚本和自检。
不包含模型权重、ROS、Node.js、旧虚拟环境、日志或任何现有私密令牌。
首次安装需要联网下载 Python（缺少时）及 Python 依赖，不是完全离线包。

## 最简单的操作

1. 把 ZIP 复制到新笔记本，**完整解压**到可写目录，例如 `D:\VLAConsole`。
   不要直接在压缩包里运行，也不要复制另一台机器安装过的 `.venv`。
2. 双击 `deploy.cmd`。自动检测 Python 3.11–3.13 x64；缺少时使用 winget
   为当前用户安装 Python 3.12。若没有 winget，先安装 Python 3.12 x64，重新双击。
3. 安装依赖并通过本机自检后，自动打开 `http://127.0.0.1:8080`。
   默认 **控制输出关闭**。没有机载图像、模型服务显示离线属于未配置时的预期状态。
4. 后续双击 `start.cmd`；保留启动窗口，按 Ctrl+C 停止本次启动的后端。
   失败信息保留在窗口，运行日志在 `logs`。不会自动结束占用端口的其他程序。

首次安装示例（可选，不需要手动逐条执行）：

```powershell
.\deploy.ps1 -InstallOnly -PythonExe 'C:\Python312\python.exe'
.\start.ps1
```

重新运行 deploy 不覆盖 `config.json`。更新版本建议解压到新目录，单独复制自己的
`config.json` 后重新安装；不要覆盖正在运行的包。运行时依赖直接版本固定，间接依赖
由 pip 解析。包内 manifest.json 记录发布文件哈希，ZIP 外另有 SHA256 校验文件。

## 先确定是否真的需要迁移后端

如果旧电脑继续运行后端，新笔记本只是操作屏，直接在新笔记本浏览器访问
`http://旧电脑可访问的局域网IP:8080` 即可；需要旧电脑监听 LAN 且防火墙允许该笔记本。
这种方式无需本部署包，也无需改机载观测上行地址。不要同时开两个 Live 操作端。

以下步骤用于**把地面站后端也迁到新笔记本**。

## 对接机载电脑 192.168.5.5

关闭启动窗口，用记事本编辑 `config.json`，不要修改 `config.example.json`：

- `ListenHost` 改成 `0.0.0.0`，用于接收机载上行。`Port` 默认 8080。
- `HOST_ONBOARD_IP`、`HOST_OPERATOR_IP` 填新笔记本相应网卡的可达 IPv4 地址；
  查看 `ipconfig`，**不要填机载 IP 或 127.0.0.1**。
- `ONBOARD_BRIDGE_HOST` 为 `192.168.5.5`，桥端口默认 50051。
- `ONBOARD_BRIDGE_TOKEN`、`ONBOARD_OBSERVATION_TOKEN`、`EXPECTED_CALIBRATION_ID`
  必须与现有机载配置匹配。由本人从现有私有配置安全复制，部署包不会打包或自动轮换这些值。
- `OPERATOR_CONTROL_TOKEN` 是网页操作令牌，需要使用者设置并在网页 Live 操作时提供。
- `VLA_OBSERVATION_MODE` 示例为 `image_odom`；确认与机载相同。
  Camera/world/body frame 和标定 ID 也必须按真实配置核对，不能因为使用 D435 语义检测
  就随意改掉 VLA 观测流的相机 frame。

在机载的 `~/.config/vla_stack.env` 中，人工把 `VLA_BACKEND_URL` 改为
`http://新笔记本机载网卡IP:8080`，并核对 `VLA_HOST_IP` 是否也需改为该地址。
这是地址修改，**不要覆盖整份已验证机载配置或重新生成全部 token**。
仅在安全落地、未解锁时停止旧链路并按原流程重启加载新配置。本部署工具不远程改机载、不启动飞控。

Windows 网卡应在你信任的“专用网络”。需要机载访问时，在管理员 PowerShell 运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\enable_onboard_firewall.ps1
```

此脚本只开放专用网络下、来自指定机载 IP 的 TCP 8080（或配置端口），不会切换网络类型、
开放公用网络或关闭防火墙。只在本机浏览网页时不需要这条入站规则。

然后重新双击 `start.cmd`，先确认网页、视频、位姿和任务状态。网络检查仅做只读操作：

```powershell
Test-NetConnection 192.168.5.5 -Port 50051
```

机载节点未启动时 50051 不通是正常的。在机载可用时，可用只读请求确认新后端可达：

```bash
curl --fail http://新笔记本IP:8080/api/missions/current
```

## Live 控制仍须显式开启

默认双击安装/启动都保持锁定，即使父进程环境变量设过 CONTROL_OUTPUT_ENABLED。
需要实际任务控制时，先结束默认控制台，再按既有人工安全流程执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\start.ps1 -EnableLiveControl -LiveControlConfirmation I_ACCEPT_REAL_FLIGHT_CONTROL
```

该入口保留令牌检查和机载 NTP 时差检查，失败不会启动 Live 后端。不自动解锁、起飞或下发任务。
启动 Live 只表示允许后续人工请求，不是飞行批准；不要同时从旧、新地面站操作无人机。

## OpenVLA / π0.5

六向移动和板载语义绕飞不需要在笔记本加载 VLA 模型。
如果模型仍在旧电脑/服务器，将 `OPENVLA_URL` 和 `PI05_HOST`/`PI05_PORT`
改为其局域网服务地址；不要继续用指向新笔记本的 127.0.0.1，除非模型确实运行在新笔记本。
只在可信局域网使用，不要把此控制台或推理服务直接暴露到公网。

## 排错

- 首次 pip 失败：检查网络/代理；修好后重跑 deploy，不会覆盖已配置内容。
- Python 不支持：使用 x64 Python 3.11–3.13；可通过 `-PythonExe` 指定。
- 8080 被占用：手动关闭旧控制台，或修改 Port 并同步修改机载上行 URL、防火墙端口。
- 页面正常但无图像：检查机载上行目标是否仍是旧电脑、token、frame、模式和标定 ID 是否匹配。
- 拒绝 Live：检查令牌和时间同步；不要扩大时差阈值绕过检查。
- `TARGET_MISMATCH` 等任务拒绝：属于机载语义任务逻辑，不是部署失败。

重新自检（不会调用机载或发布任务）：

```powershell
.\.venv\Scripts\python.exe .\smoke_test.py
```
