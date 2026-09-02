import React, { useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import "./styles.css";

const initialSystem = {
  backend: "connecting",
  safety_lock: true,
  models: {
    openvla: { status: "offline", detail: "Waiting for status" },
    pi05: { status: "offline", detail: "Waiting for status" },
  },
  mission: null,
};

const atomicTasks = [
  ["takeoff", "起飞", "↑"], ["land", "降落", "↓"], ["hold", "悬停", "■"],
  ["move_forward", "前进", "↑"], ["move_backward", "后退", "↓"],
  ["move_left", "左移", "←"], ["move_right", "右移", "→"],
  ["move_up", "上升", "+Z"], ["move_down", "下降", "−Z"],
  ["yaw_left", "左旋", "↺"], ["yaw_right", "右旋", "↻"],
];

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload.detail || `Request failed: ${response.status}`);
  return payload;
}

function StatusDot({ status }) {
  return <span className={`status-dot status-${status}`} aria-hidden="true" />;
}

function Service({ name, service }) {
  return <div className="service-row"><div><span className="service-name"><StatusDot status={service?.status || "offline"} />{name}</span><p>{service?.detail || "No status"}</p></div><span className="latency">{service?.latency_ms ? `${service.latency_ms} ms` : "—"}</span></div>;
}

function NumericField({ label, value, setValue, min, max, step, unit }) {
  return <label className="numeric-field"><span>{label}</span><div><input type="number" value={value} min={min} max={max} step={step} onChange={(event) => setValue(Number(event.target.value))} /><small>{unit}</small></div></label>;
}

function App() {
  const [system, setSystem] = useState(initialSystem);
  const [category, setCategory] = useState("embodied");
  const [atomicTask, setAtomicTask] = useState("move_forward");
  const [embodiedTask, setEmbodiedTask] = useState("freeform");
  const [instruction, setInstruction] = useState("向前飞行，并与障碍物保持安全距离");
  const [targetLabel, setTargetLabel] = useState("电线杆");
  const [policy, setPolicy] = useState("openvla");
  const [mode, setMode] = useState("dry_run");
  const [distance, setDistance] = useState(0.5);
  const [takeoffHeight, setTakeoffHeight] = useState(1.0);
  const [yawDeg, setYawDeg] = useState(30);
  const [radius, setRadius] = useState(1.5);
  const [laps, setLaps] = useState(1);
  const [orbitDirection, setOrbitDirection] = useState("clockwise");
  const [extraDistance, setExtraDistance] = useState(2);
  const [operatorToken, setOperatorToken] = useState("");
  const [liveConfirmation, setLiveConfirmation] = useState("");
  const [testPreviewUrl, setTestPreviewUrl] = useState("");
  const [imageBase64, setImageBase64] = useState("");
  const [action, setAction] = useState(null);
  const [busy, setBusy] = useState(false);
  const [chat, setChat] = useState([{ role: "system", text: "操作台已启动。默认处于 dry-run，任何任务只校验、不下发。", time: new Date() }]);
  const reconnectRef = useRef(null);

  const mission = system.mission;
  const onboard = system.onboard_observation || {};
  const missionState = mission?.state || "IDLE";
  const liveReady = mode === "live" && !system.safety_lock;
  const displayedImage = onboard.connected ? "/api/onboard/stream.mjpeg" : testPreviewUrl;

  const appendChat = (role, text) => setChat((current) => [...current, { role, text, time: new Date() }].slice(-30));

  useEffect(() => {
    let socket;
    let cancelled = false;
    const connect = () => {
      const protocol = window.location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(`${protocol}://${window.location.host}/ws/status`);
      socket.onmessage = (event) => { if (!cancelled) setSystem(JSON.parse(event.data)); };
      socket.onopen = () => appendChat("system", "状态通道已连接，开始接收图像序号和 FAST-LIO 位姿。");
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

  useEffect(() => {
    const predicted = onboard.last_result?.action_local_delta?.[0];
    if (predicted) setAction(predicted);
  }, [onboard.last_result?.preview_sequence]);

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
      return `${selected?.[1]} ${distance} m`;
    }
    if (embodiedTask === "orbit_target") return `以 ${radius} m 半径${orbitDirection === "clockwise" ? "顺时针" : "逆时针"}绕 ${targetLabel} 飞行 ${laps} 圈`;
    if (embodiedTask === "pass_target_forward") return `飞过 ${targetLabel} 后继续前进 ${extraDistance} m`;
    return instruction;
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
        instruction, policy, mode, live_confirmation: liveConfirmation,
        parameters: {
          distance_m: distance, takeoff_height_m: takeoffHeight, yaw_deg: yawDeg,
          target_label: targetLabel, radius_m: radius, laps,
          orbit_direction: orbitDirection, extra_distance_m: extraDistance,
        },
      }),
    });
    const detail = payload.delivery?.detail || payload.delivery?.status || "任务已接收";
    appendChat("system", `${payload.mode === "live" ? "实机任务" : "预演任务"} ${payload.task_id.slice(0, 8)}：${detail}`);
    if (payload.mission) setSystem((current) => ({ ...current, mission: payload.mission }));
  });

  const missionCommand = (command) => perform(async () => {
    if (!mission) throw new Error("当前没有活动任务。");
    const payload = await api(`/api/missions/${mission.mission_id}/${command}`, { method: "POST" });
    setSystem((current) => ({ ...current, mission: payload.mission }));
    appendChat("system", payload.mission.status_message);
  });

  const handleImage = (event) => {
    const file = event.target.files?.[0];
    if (!file || !file.type.startsWith("image/")) return;
    const reader = new FileReader();
    reader.onload = () => {
      const dataUrl = String(reader.result);
      setTestPreviewUrl(dataUrl);
      setImageBase64(dataUrl.split(",")[1] || "");
      setAction(null);
      appendChat("system", `已载入离线测试帧：${file.name}`);
    };
    reader.readAsDataURL(file);
  };

  const inferTestFrame = () => perform(async () => {
    if (!imageBase64) throw new Error("请先载入一张离线测试图像。");
    const pose = onboard.local_state?.position || { x: 0, y: 0, z: 0 };
    const currentYawDeg = Number(onboard.local_state?.yaw_rad || 0) * 180 / Math.PI;
    const payload = await api(`/api/inference/${policy}`, {
      method: "POST",
      body: JSON.stringify({ image_base64: imageBase64, instruction, proprio: [pose.x, pose.y, pose.z, currentYawDeg] }),
    });
    setAction(payload.action_local_delta[0]);
    appendChat("system", `${policy === "openvla" ? "OpenVLA" : "π0.5"} 离线单帧推理完成，结果未下发。`);
  });

  const actionRows = useMemo(() => {
    if (!action) return [];
    return ["dx", "dy", "dz", "d_yaw"].map((label, index) => ({ label, value: Number(action[index]).toFixed(4), unit: index === 3 ? "rad" : "m" }));
  }, [action]);

  const pose = onboard.local_state || {};
  const position = pose.position || {};
  const linear = pose.linear_velocity || {};

  return <main className="shell">
    <header className="topbar">
      <div className="brand"><span className="brand-mark">VLA</span><div><h1>UAV Embodied Ground Station</h1><p>OpenVLA / π0.5 · FAST-LIO · Diff-Planner</p></div></div>
      <div className="top-status"><span><StatusDot status={system.backend === "online" ? "online" : "offline"} />Backend</span><span><StatusDot status={onboard.connected ? "online" : "offline"} />UAV uplink</span><span className={`lock ${system.safety_lock ? "locked" : "unlocked"}`}>{system.safety_lock ? "CONTROL LOCKED" : "LIVE CONTROL ENABLED"}</span></div>
    </header>

    <section className="metric-strip" aria-label="System summary">
      <div><span>Mission</span><strong>{missionState}</strong></div><div><span>Video uplink</span><strong>{onboard.receive_fps ? `${onboard.receive_fps} FPS` : "—"}</strong></div><div><span>Frame age</span><strong>{onboard.receive_age_ms != null ? `${onboard.receive_age_ms} ms` : "—"}</strong></div><div><span>K-frame policy</span><strong>{onboard.k_frames ? `${onboard.frames_until_inference} / ${onboard.k_frames}` : "—"}</strong></div><div><span>Host LAN</span><strong>{system.host_interfaces?.onboard_lan || "127.0.0.1"}</strong></div>
    </section>

    <section className="workspace">
      <div className="primary-column">
        <article className="panel vision-panel">
          <div className="panel-heading"><div><span className="eyebrow">Live observation</span><h2>机载相机视频流</h2></div><label className="file-button">载入离线帧<input type="file" accept="image/*" onChange={handleImage} /></label></div>
          <div className={`camera-stage ${displayedImage ? "has-image" : ""}`}>{displayedImage ? <img src={displayedImage} alt="UAV live camera stream" /> : <div className="camera-empty"><span>RGB</span><strong>等待机载视频流</strong><p>后端将通过 MJPEG 持续显示最近接收的图像。</p></div>}<div className="camera-overlay top-left">FRAME <b>{onboard.image_sequence ?? "—"}</b> · VEHICLE <b>{onboard.vehicle_id || "—"}</b></div><div className="camera-overlay bottom-right">{onboard.last_result?.output_mode === "live_trajectory" ? "LIVE TRAJECTORY" : "PLANNER PREVIEW"}</div></div>
          <div className="action-grid">{actionRows.length ? actionRows.map((item) => <div key={item.label}><span>{item.label}</span><strong>{item.value}</strong><small>{item.unit}</small></div>) : <p className="empty-action">等待 VLA 动作预测。</p>}</div>
        </article>

        <div className="telemetry-grid">
          <article className="panel state-panel"><div className="panel-heading"><div><span className="eyebrow">FAST-LIO / EKF</span><h2>实时位姿</h2></div></div><div className="state-values state-six"><span>x <b>{Number(position.x ?? 0).toFixed(3)}</b> m</span><span>y <b>{Number(position.y ?? 0).toFixed(3)}</b> m</span><span>z <b>{Number(position.z ?? 0).toFixed(3)}</b> m</span><span>yaw <b>{Number(pose.yaw_rad ?? 0).toFixed(3)}</b> rad</span><span>vx <b>{Number(linear.x ?? 0).toFixed(3)}</b> m/s</span><span>vy <b>{Number(linear.y ?? 0).toFixed(3)}</b> m/s</span></div></article>
          <article className="panel services-panel"><div className="panel-heading"><div><span className="eyebrow">Runtime</span><h2>推理与感知服务</h2></div></div><Service name="OpenVLA · real 3ep" service={system.models?.openvla} /><Service name="π0.5 · UAV-Flow 1ep" service={system.models?.pi05} /><Service name="FAST-LIO + RGB uplink" service={{ status: onboard.connected ? (onboard.calibration_validated ? "online" : "degraded") : "offline", detail: onboard.connected ? `${onboard.world_frame} → ${onboard.body_frame} · ${onboard.calibration_id}` : "等待同步观测" }} /></article>
        </div>
      </div>

      <aside className="side-column">
        <article className="panel command-panel">
          <div className="panel-heading"><div><span className="eyebrow">Operator dialog</span><h2>任务对话与控制</h2></div><span className={`mode-badge ${mode}`}>{mode}</span></div>
          <div className="chat-window">{chat.map((message, index) => <div className={`chat-message ${message.role}`} key={`${message.time.getTime()}-${index}`}><span>{message.role === "operator" ? "操作员" : message.role === "error" ? "错误" : "系统"}</span><p>{message.text}</p><time>{message.time.toLocaleTimeString("zh-CN", { hour12: false })}</time></div>)}</div>
          <div className="task-tabs"><button className={category === "atomic" ? "active" : ""} onClick={() => setCategory("atomic")}>原子任务</button><button className={category === "embodied" ? "active" : ""} onClick={() => setCategory("embodied")}>VLA 具身任务</button></div>

          {category === "atomic" ? <>
            <div className="atomic-grid">{atomicTasks.map(([name, label, glyph]) => <button key={name} className={atomicTask === name ? "selected" : ""} onClick={() => setAtomicTask(name)}><b>{glyph}</b><span>{label}</span></button>)}</div>
            <div className="parameter-grid"><NumericField label="移动距离" value={distance} setValue={setDistance} min="0.05" max="1" step="0.05" unit="m" /><NumericField label="起飞配置高度" value={takeoffHeight} setValue={setTakeoffHeight} min="0.3" max="2" step="0.1" unit="m" /><NumericField label="旋转角度" value={yawDeg} setValue={setYawDeg} min="1" max="90" step="1" unit="°" /></div>
          </> : <>
            <div className="template-row"><button className={embodiedTask === "freeform" ? "selected" : ""} onClick={() => setEmbodiedTask("freeform")}>自由指令</button><button className={embodiedTask === "orbit_target" ? "selected" : ""} onClick={() => setEmbodiedTask("orbit_target")}>绕目标飞行</button><button className={embodiedTask === "pass_target_forward" ? "selected" : ""} onClick={() => setEmbodiedTask("pass_target_forward")}>飞过后前进</button></div>
            {embodiedTask === "freeform" ? <textarea className="instruction-box" value={instruction} onChange={(event) => setInstruction(event.target.value)} placeholder="输入要完成的具身目标……" /> : <><label className="text-field"><span>目标名称</span><input value={targetLabel} onChange={(event) => setTargetLabel(event.target.value)} placeholder="例如：椅子、电线杆、红色箱子" /></label>{embodiedTask === "orbit_target" ? <div className="parameter-grid"><NumericField label="绕飞半径" value={radius} setValue={setRadius} min="0.5" max="5" step="0.1" unit="m" /><NumericField label="圈数" value={laps} setValue={setLaps} min="0.25" max="3" step="0.25" unit="圈" /><label className="numeric-field"><span>方向</span><select value={orbitDirection} onChange={(event) => setOrbitDirection(event.target.value)}><option value="clockwise">顺时针</option><option value="counterclockwise">逆时针</option></select></label></div> : <div className="parameter-grid"><NumericField label="通过后继续前进" value={extraDistance} setValue={setExtraDistance} min="0.2" max="5" step="0.1" unit="m" /></div>}</>}
          </>}

          <div className="dispatch-settings"><label><span>策略</span><select value={policy} onChange={(event) => setPolicy(event.target.value)}><option value="openvla">OpenVLA 3ep</option><option value="pi05">π0.5 1ep</option></select></label><label><span>模式</span><select value={mode} onChange={(event) => setMode(event.target.value)}><option value="dry_run">Dry-run（不下发）</option><option value="live">Live（实机）</option></select></label></div>
          {mode === "live" && <div className="live-gate"><strong>实机双重确认</strong><input type="password" value={operatorToken} onChange={(event) => setOperatorToken(event.target.value)} placeholder="操作令牌" /><input value={liveConfirmation} onChange={(event) => setLiveConfirmation(event.target.value)} placeholder="输入主机配置的确认短语" /><small>{liveReady ? "主机输出开关已开启，仍需机载桥开关。" : "主机输出锁尚未开启，本请求会被拒绝。"}</small></div>}
          <button className={`dispatch-button ${mode}`} disabled={busy} onClick={dispatchTask}>{busy ? "处理中……" : mode === "live" ? "确认并下发实机任务" : "提交 Dry-run 任务"}</button>
          <div className="mission-actions"><button disabled={busy || missionState !== "RUNNING"} onClick={() => missionCommand("hold")}>暂停 VLA</button><button disabled={busy || !mission || ["ABORTED", "SUCCEEDED", "FAULT"].includes(missionState)} onClick={() => missionCommand("stop")}>停止任务</button><button disabled={busy || !imageBase64} onClick={inferTestFrame}>离线单帧推理</button></div>
          {mission && <div className="mission-id"><span>当前任务</span><code>{mission.mission_id}</code><p>{mission.status_message}</p></div>}
        </article>
        <article className="safety-note"><span>安全边界</span><p>网页默认只做轨迹预览。Live 请求必须同时通过主机输出开关、操作令牌、确认短语和机载发布开关；本页面不会发送 MAVROS 解锁或飞控模式切换命令。</p></article>
      </aside>
    </section>
  </main>;
}

createRoot(document.getElementById("root")).render(<React.StrictMode><App /></React.StrictMode>);
