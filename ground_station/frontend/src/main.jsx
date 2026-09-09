import React, { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { api } from "./api.js";
import "./styles.css";

const initialSystem = { backend: "connecting", safety_lock: true };

const atomicTasks = [
  ["takeoff", "起飞", "↑"], ["land", "降落", "↓"], ["hold", "悬停", "■"],
  ["move_forward", "前进", "↑"], ["move_backward", "后退", "↓"],
  ["move_left", "左移", "←"], ["move_right", "右移", "→"],
  ["move_up", "上升", "+Z"], ["move_down", "下降", "−Z"],
  ["yaw_left", "左旋", "↺"], ["yaw_right", "右旋", "↻"],
  ["orbit_world", "定点绕飞", "○"],
];

const onboardTasks = [
  ["semantic_orbit", "双目语义绕飞"],
  ["monocular_semantic_orbit", "左目双位置绕飞"],
  ["semantic_scan_orbit", "扫描椅子并绕飞"],
  ["hybrid_semantic_orbit", "远近融合绕飞"],
];

function StatusDot({ status }) {
  return <span className={`status-dot status-${status}`} aria-hidden="true" />;
}

function Service({ name, service }) {
  return <div className="service-row"><div><span className="service-name"><StatusDot status={service?.status || "offline"} />{name}</span><p>{service?.detail || "无状态"}</p></div></div>;
}

function NumericField({ label, value, setValue, min, max, step, unit }) {
  return <label className="numeric-field"><span>{label}</span><div><input type="number" value={value} min={min} max={max} step={step} onChange={(event) => setValue(Number(event.target.value))} /><small>{unit}</small></div></label>;
}

function App() {
  const [system, setSystem] = useState(initialSystem);
  const [category, setCategory] = useState("atomic");
  const [atomicTask, setAtomicTask] = useState("move_forward");
  const [embodiedTask, setEmbodiedTask] = useState("semantic_orbit");
  const [targetLabel, setTargetLabel] = useState("chair");
  const [mode, setMode] = useState("dry_run");
  const [distance, setDistance] = useState(0.5);
  const takeoffHeight = 0.8;
  const [yawDeg, setYawDeg] = useState(30);
  const [radius, setRadius] = useState(1.5);
  const [laps, setLaps] = useState(1);
  const [orbitDirection, setOrbitDirection] = useState("clockwise");
  const [centerX, setCenterX] = useState(0);
  const [centerY, setCenterY] = useState(0);
  const [centerZ, setCenterZ] = useState(1);
  const [baselineDistance, setBaselineDistance] = useState(0.6);
  const [baselineDirection, setBaselineDirection] = useState("right");
  const [operatorToken, setOperatorToken] = useState("");
  const [liveConfirmation, setLiveConfirmation] = useState("");
  const [busy, setBusy] = useState(false);
  const [stopping, setStopping] = useState(false);
  const [chat, setChat] = useState([{ role: "system", text: "操作台已启动。默认处于 dry-run，任何任务只校验、不下发。", time: new Date() }]);
  const reconnectRef = useRef(null);

  const onboard = system.onboard_observation || {};
  const receiveAge = Number(onboard.receive_age_ms);
  const uplinkFresh = onboard.connected === true && Number.isFinite(receiveAge) && receiveAge <= 3000;
  const liveReady = mode === "live" && !system.safety_lock;
  const scanMissionSelected = embodiedTask === "semantic_scan_orbit";
  const hybridMissionSelected = embodiedTask === "hybrid_semantic_orbit";
  const latestTask = system.task_runtime?.recent_tasks?.[0];

  const appendChat = (role, text) => setChat((current) => [...current, { role, text, time: new Date() }].slice(-30));

  useEffect(() => {
    let socket;
    let cancelled = false;
    const connect = () => {
      const protocol = window.location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(`${protocol}://${window.location.host}/ws/status`);
      socket.onmessage = (event) => { if (!cancelled) setSystem(JSON.parse(event.data)); };
      socket.onopen = () => appendChat("system", "状态通道已连接，开始接收机载图像和 FAST-LIO 位姿。");
      socket.onclose = () => {
        if (!cancelled) {
          setSystem((current) => ({ ...current, backend: "offline" }));
          reconnectRef.current = window.setTimeout(connect, 1500);
        }
      };
    };
    connect();
    return () => { cancelled = true; window.clearTimeout(reconnectRef.current); socket?.close(); };
  }, []);

  const perform = async (operation) => {
    setBusy(true);
    try { await operation(); } catch (error) { appendChat("error", error.message); } finally { setBusy(false); }
  };

  const taskSummary = () => {
    if (category === "atomic") {
      const selected = atomicTasks.find(([name]) => name === atomicTask);
      if (atomicTask === "takeoff") return `${selected?.[1]}（配置高度 ${takeoffHeight} m）`;
      if (["yaw_left", "yaw_right"].includes(atomicTask)) return `${selected?.[1]} ${yawDeg}°`;
      if (["hold", "land"].includes(atomicTask)) return selected?.[1] || atomicTask;
      if (atomicTask === "orbit_world") return `${selected?.[1]}：圆心 (${centerX}, ${centerY}, ${centerZ}) m，半径 ${radius} m，${orbitDirection === "clockwise" ? "顺时针" : "逆时针"} ${laps} 圈`;
      return `${selected?.[1]} ${distance} m`;
    }
    if (embodiedTask === "semantic_scan_orbit") return "沿 world +X 扫描 6 m、向 +Y 展开 5 条；发现 chair 后顺时针绕飞一圈并续扫";
    if (embodiedTask === "hybrid_semantic_orbit") return `机载检测 ${targetLabel}，${baselineDirection === "right" ? "右移" : "左移"} ${baselineDistance} m 粗定位，近距离双目精定位后绕飞`;
    if (embodiedTask === "monocular_semantic_orbit") return `D435 左目双位置检测 ${targetLabel}，${baselineDirection === "right" ? "右移" : "左移"} ${baselineDistance} m 后${orbitDirection === "clockwise" ? "顺时针" : "逆时针"}绕飞`;
    return `机载 YOLO-World 检测 ${targetLabel} 并${orbitDirection === "clockwise" ? "顺时针" : "逆时针"}绕飞`;
  };

  const dispatchTask = () => perform(async () => {
    const summary = taskSummary();
    appendChat("operator", `[${mode === "live" ? "LIVE" : "DRY-RUN"}] ${summary}`);
    const payload = await api("/api/tasks/dispatch", {
      method: "POST",
      headers: operatorToken ? { "X-Operator-Token": operatorToken } : {},
      body: JSON.stringify({
        category,
        atomic_task: category === "atomic" ? atomicTask : null,
        embodied_task: category === "embodied" ? embodiedTask : null,
        instruction: summary, mode, live_confirmation: liveConfirmation,
        parameters: {
          distance_m: distance, takeoff_height_m: takeoffHeight, yaw_deg: yawDeg,
          target_label: scanMissionSelected ? "chair" : targetLabel,
          radius_m: 1.5, laps: 1,
          orbit_direction: (scanMissionSelected || hybridMissionSelected) ? "clockwise" : orbitDirection,
          baseline_distance_m: baselineDistance, baseline_direction: baselineDirection,
          center_x_m: centerX, center_y_m: centerY, center_z_m: centerZ,
        },
      }),
    });
    const detail = payload.delivery?.detail || payload.delivery?.status || "任务已接收";
    appendChat("system", `${payload.mode === "live" ? "实机任务" : "预演任务"} ${payload.task_id.slice(0, 8)}：${detail}`);
  });

  const pose = uplinkFresh ? (onboard.local_state || {}) : {};
  const stopTask = async () => {
    setStopping(true);
    try {
      const payload = await api("/api/tasks/stop", {
        method: "POST",
        headers: operatorToken ? { "X-Operator-Token": operatorToken } : {},
        body: JSON.stringify({ mode, live_confirmation: liveConfirmation }),
      });
      appendChat("system", `[${mode}] 停止请求：${payload.delivery?.detail || payload.delivery?.status}。请核对板载取消状态及实际悬停。`);
    } catch (error) { appendChat("error", `停止未确认：${error.message}；请使用遥控器接管。`); }
    finally { setStopping(false); }
  };
  const position = pose.position || {};
  const linear = pose.linear_velocity || {};
  const frameAgeLabel = Number.isFinite(receiveAge) ? (receiveAge < 10000 ? `${receiveAge.toFixed(0)} ms` : `${(receiveAge / 1000).toFixed(0)} s（过期）`) : "—";

  return <main className="shell">
    <header className="topbar">
      <div className="brand"><span className="brand-mark">UAV</span><div><h1>机载任务控制台</h1><p>双目视觉 · FAST-LIO · Diff-Planner</p></div></div>
      <div className="top-status"><span><StatusDot status={system.backend === "online" ? "online" : "offline"} />地面站</span><span><StatusDot status={uplinkFresh ? "online" : "offline"} />机载上行</span><span className={`lock ${system.safety_lock ? "locked" : "unlocked"}`}>{system.safety_lock ? "控制锁定" : "实机控制已开启"}</span></div>
    </header>

    <section className="metric-strip" aria-label="系统摘要">
      <div><span>机载链路</span><strong>{uplinkFresh ? "在线" : "离线"}</strong></div><div><span>视频上行</span><strong>{uplinkFresh && onboard.receive_fps ? `${onboard.receive_fps} FPS` : "—"}</strong></div><div><span>帧延迟</span><strong>{frameAgeLabel}</strong></div><div><span>控制模式</span><strong>{system.safety_lock ? "LOCKED" : "LIVE"}</strong></div><div><span>地面站地址</span><strong>{system.host_interfaces?.onboard_lan || "127.0.0.1"}</strong></div>
    </section>

    <section className="workspace">
      <div className="primary-column">
        <article className="panel vision-panel">
          <div className="panel-heading"><div><span className="eyebrow">Live observation</span><h2>机载相机视频流</h2></div><span className={`mode-badge ${uplinkFresh ? "live" : "dry_run"}`}>{uplinkFresh ? "LIVE" : "OFFLINE"}</span></div>
          <div className={`camera-stage ${uplinkFresh ? "has-image" : ""}`}>{uplinkFresh ? <img src="/api/onboard/stream.mjpeg" alt="UAV live camera stream" /> : <div className="camera-empty"><span>RGB</span><strong>等待机载视频流</strong><p>{Number.isFinite(receiveAge) ? "最近一次数据已过期，请启动机载 bridge。" : "尚未收到机载观测。"}</p></div>}<div className="camera-overlay top-left">FRAME <b>{uplinkFresh ? onboard.image_sequence : "—"}</b> · VEHICLE <b>{uplinkFresh ? onboard.vehicle_id : "—"}</b></div><div className="camera-overlay bottom-right">{uplinkFresh ? "ONBOARD STREAM" : "NO UPLINK"}</div></div>
        </article>

        <div className="telemetry-grid">
          <article className="panel state-panel"><div className="panel-heading"><div><span className="eyebrow">FAST-LIO / EKF</span><h2>实时位姿</h2></div></div><div className="state-values state-six"><span>x <b>{position.x == null ? "—" : Number(position.x).toFixed(3)}</b> m</span><span>y <b>{position.y == null ? "—" : Number(position.y).toFixed(3)}</b> m</span><span>z <b>{position.z == null ? "—" : Number(position.z).toFixed(3)}</b> m</span><span>yaw <b>{pose.yaw_rad == null ? "—" : Number(pose.yaw_rad).toFixed(3)}</b> rad</span><span>vx <b>{linear.x == null ? "—" : Number(linear.x).toFixed(3)}</b> m/s</span><span>vy <b>{linear.y == null ? "—" : Number(linear.y).toFixed(3)}</b> m/s</span></div></article>
          <article className="panel services-panel"><div className="panel-heading"><div><span className="eyebrow">Runtime</span><h2>机载感知与规划</h2></div></div><Service name="FAST-LIO + RGB uplink" service={{ status: uplinkFresh ? (onboard.calibration_validated ? "online" : "degraded") : "offline", detail: uplinkFresh ? `${onboard.world_frame} → ${onboard.body_frame} · ${onboard.calibration_id}` : "等待同步观测" }} /><Service name="板载任务 bridge" service={{ status: uplinkFresh ? "online" : "offline", detail: uplinkFresh ? "任务状态与观测上行正常" : "未收到新数据" }} /></article>
        </div>
      </div>

      <aside className="side-column">
        <article className="panel command-panel">
          <div className="panel-heading"><div><span className="eyebrow">Operator control</span><h2>任务下发</h2></div><span className={`mode-badge ${mode}`}>{mode}</span></div>
          <div className="chat-window">{chat.map((message, index) => <div className={`chat-message ${message.role}`} key={`${message.time.getTime()}-${index}`}><span>{message.role === "operator" ? "操作员" : message.role === "error" ? "错误" : "系统"}</span><p>{message.text}</p><time>{message.time.toLocaleTimeString("zh-CN", { hour12: false })}</time></div>)}</div>
          <div className="task-tabs"><button className={category === "atomic" ? "active" : ""} onClick={() => setCategory("atomic")}>原子任务</button><button className={category === "embodied" ? "active" : ""} onClick={() => setCategory("embodied")}>板载语义任务</button></div>

          {category === "atomic" ? <>
            <div className="atomic-grid">{atomicTasks.map(([name, label, glyph]) => <button key={name} className={atomicTask === name ? "selected" : ""} onClick={() => setAtomicTask(name)}><b>{glyph}</b><span>{label}</span></button>)}</div>
            {atomicTask === "orbit_world" ? <div className="parameter-grid"><NumericField label="圆心 X" value={centerX} setValue={setCenterX} min="-1000" max="1000" step="0.1" unit="m" /><NumericField label="圆心 Y" value={centerY} setValue={setCenterY} min="-1000" max="1000" step="0.1" unit="m" /><NumericField label="圆心 Z / 绕飞高度" value={centerZ} setValue={setCenterZ} min="-100" max="100" step="0.1" unit="m" /><NumericField label="绕飞半径" value={radius} setValue={setRadius} min="0.5" max="5" step="0.1" unit="m" /><NumericField label="圈数" value={laps} setValue={setLaps} min="0.25" max="3" step="0.25" unit="圈" /><label className="numeric-field"><span>方向</span><select value={orbitDirection} onChange={(event) => setOrbitDirection(event.target.value)}><option value="clockwise">顺时针</option><option value="counterclockwise">逆时针</option></select></label></div> : <div className="parameter-grid"><NumericField label="移动距离" value={distance} setValue={setDistance} min="0.05" max="2" step="0.05" unit="m" /><div className="numeric-field"><span>起飞相对高度（固定）</span><div>0.8 m</div><small>LIVE 起飞会请求 PX4Ctrl 切换 Offboard、解锁并爬升。</small></div><NumericField label="旋转角度" value={yawDeg} setValue={setYawDeg} min="1" max="90" step="1" unit="°" /></div>}
          </> : <>
            <div className="template-row">{onboardTasks.map(([name, label]) => <button key={name} className={embodiedTask === name ? "selected" : ""} onClick={() => { setEmbodiedTask(name); setTargetLabel("chair"); if (["semantic_scan_orbit", "hybrid_semantic_orbit"].includes(name)) setOrbitDirection("clockwise"); }}>{label}</button>)}</div>
            <label className="text-field"><span>{scanMissionSelected ? "固定检测目标" : "YOLO-World 英文目标词"}</span><input value={scanMissionSelected ? "chair" : targetLabel} disabled={scanMissionSelected} onChange={(event) => setTargetLabel(event.target.value)} pattern="[A-Za-z][A-Za-z-]{0,31}" placeholder="例如：chair、person、bottle" /></label>
            <div className="parameter-grid"><div className="numeric-field"><span>板载流水线</span><div>{scanMissionSelected ? "曲线扫描 → YOLO → 双目绕飞 → 断点续扫" : hybridMissionSelected ? "左目粗定位 → 分段靠近 → 双目精定位 → 绕飞" : embodiedTask === "monocular_semantic_orbit" ? "左目 A/B 拍摄 → 实测基线定位 → Diff-Planner → ORBIT" : "YOLO-World → D435 stereo → Diff-Planner → ORBIT"}</div><small>检测、定位和规划均在机载电脑执行，笔记本不进行模型推理。</small></div>{["monocular_semantic_orbit", "hybrid_semantic_orbit"].includes(embodiedTask) && <><NumericField label="第二次横移位移" value={baselineDistance} setValue={setBaselineDistance} min="0.5" max="1" step="0.05" unit="m" /><label className="numeric-field"><span>第二次横移方向</span><select value={baselineDirection} onChange={(event) => setBaselineDirection(event.target.value)}><option value="right">向右横移</option><option value="left">向左横移</option></select></label></>}{!scanMissionSelected && !hybridMissionSelected && <label className="numeric-field"><span>绕飞方向</span><select value={orbitDirection} onChange={(event) => setOrbitDirection(event.target.value)}><option value="clockwise">顺时针</option><option value="counterclockwise">逆时针</option></select></label>}</div>
          </>}

          <div className="dispatch-settings"><label><span>执行位置</span><div>{category === "atomic" ? "板载原子飞行技能" : "板载视觉与 Diff-Planner 流水线"}</div></label><label><span>模式</span><select value={mode} onChange={(event) => setMode(event.target.value)}><option value="dry_run">Dry-run（不下发）</option><option value="live">Live（实机）</option></select></label></div>
          {mode === "live" && <div className="live-gate"><strong>实机双重确认</strong><input type="password" value={operatorToken} onChange={(event) => setOperatorToken(event.target.value)} placeholder="操作令牌" /><input value={liveConfirmation} onChange={(event) => setLiveConfirmation(event.target.value)} placeholder="输入确认短语" /><small>{liveReady ? "主机输出已开启；仍要求机载 bridge 在线且处于 live。" : "主机输出锁尚未开启，本请求会被拒绝。"}</small></div>}
          <button className={`dispatch-button ${mode}`} disabled={busy || (mode === "live" && !uplinkFresh)} onClick={dispatchTask}>{busy ? "处理中……" : mode === "live" ? (uplinkFresh ? "确认并下发实机任务" : "机载 bridge 离线") : "提交 Dry-run 任务"}</button>
          <button className="dispatch-button" disabled={stopping} onClick={stopTask}>{stopping ? "正在请求停止……" : mode === "live" ? "停止所有板载任务并悬停" : "Dry-run：校验停止任务"}</button>
          <small>停止不受任务选择、普通请求忙碌或图像上行状态限制；仍需实机授权。非紧急断电，链路故障请用遥控器接管。</small>
          {(system.task_runtime?.recent_tasks || []).slice(0, 4).map((task) => <div className="mission-id" key={task.task_id}><span>{task.label || "板载任务状态"}</span><code>{task.task_id?.slice(0, 8)} · {task.runtime?.semantic_state || task.runtime?.status || task.delivery?.status}</code><p>{task.runtime?.detail || task.delivery?.detail}</p></div>)}
        </article>
        {onboard.observation_mode === "image_odom" && <article className="safety-note"><span>图像 + 里程计模式</span><p>当前未使用相机安装外参，不支持依赖外参的目标三维定位。FAST-LIO/EKF、时间检查及航点过期剔除仍然有效。</p></article>}
        <article className="safety-note"><span>安全边界</span><p>Live 请求必须同时通过主机输出开关、操作令牌、确认短语和机载发布开关。机载链路离线时，页面禁止下发实机任务。</p></article>
      </aside>
    </section>
  </main>;
}

createRoot(document.getElementById("root")).render(<React.StrictMode><App /></React.StrictMode>);
