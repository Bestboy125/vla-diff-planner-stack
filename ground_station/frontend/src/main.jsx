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
  return (
    <div className="service-row">
      <div>
        <span className="service-name"><StatusDot status={service?.status || "offline"} />{name}</span>
        <p>{service?.detail || "No status"}</p>
      </div>
      <span className="latency">{service?.latency_ms ? `${service.latency_ms} ms` : "—"}</span>
    </div>
  );
}

function App() {
  const [system, setSystem] = useState(initialSystem);
  const [instruction, setInstruction] = useState("向前飞行，并与障碍物保持安全距离");
  const [policy, setPolicy] = useState("openvla");
  const [mode, setMode] = useState("dry_run");
  const [previewUrl, setPreviewUrl] = useState("");
  const [imageBase64, setImageBase64] = useState("");
  const [action, setAction] = useState(null);
  const [busy, setBusy] = useState(false);
  const [events, setEvents] = useState([{ time: new Date(), text: "控制台已启动，等待后端连接。", tone: "info" }]);
  const reconnectRef = useRef(null);

  const mission = system.mission;
  const onboard = system.onboard_observation || {};
  const canCreate = !mission || ["SUCCEEDED", "ABORTED", "FAULT"].includes(mission.state);
  const missionState = mission?.state || "IDLE";

  const appendEvent = (text, tone = "info") => {
    setEvents((current) => [{ time: new Date(), text, tone }, ...current].slice(0, 12));
  };

  useEffect(() => {
    let socket;
    let cancelled = false;
    const connect = () => {
      const protocol = window.location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(`${protocol}://${window.location.host}/ws/status`);
      socket.onmessage = (event) => {
        if (!cancelled) setSystem(JSON.parse(event.data));
      };
      socket.onopen = () => appendEvent("状态通道已连接。", "success");
      socket.onclose = () => {
        if (!cancelled) {
          setSystem((current) => ({ ...current, backend: "offline" }));
          reconnectRef.current = window.setTimeout(connect, 1500);
        }
      };
    };
    connect();
    return () => {
      cancelled = true;
      window.clearTimeout(reconnectRef.current);
      socket?.close();
    };
  }, []);

  useEffect(() => {
    if (onboard.image_sequence === null || onboard.image_sequence === undefined) return;
    setPreviewUrl(`/api/onboard/latest/image?sequence=${onboard.image_sequence}`);
    const predicted = onboard.last_result?.action_local_delta?.[0];
    if (predicted) setAction(predicted);
  }, [onboard.image_sequence, onboard.last_result?.preview_sequence]);

  const handleImage = (event) => {
    const file = event.target.files?.[0];
    if (!file) return;
    if (!file.type.startsWith("image/")) {
      appendEvent("请选择有效的图像文件。", "danger");
      return;
    }
    const reader = new FileReader();
    reader.onload = () => {
      const dataUrl = String(reader.result);
      setPreviewUrl(dataUrl);
      setImageBase64(dataUrl.split(",")[1] || "");
      setAction(null);
      appendEvent(`已载入测试帧：${file.name}`);
    };
    reader.readAsDataURL(file);
  };

  const perform = async (operation) => {
    setBusy(true);
    try {
      await operation();
    } catch (error) {
      appendEvent(error.message, "danger");
    } finally {
      setBusy(false);
    }
  };

  const createMission = () => perform(async () => {
    const payload = await api("/api/missions", {
      method: "POST",
      body: JSON.stringify({ instruction, policy, mode }),
    });
    setSystem((current) => ({ ...current, mission: payload.mission }));
    appendEvent(`任务 ${payload.mission.mission_id.slice(0, 8)} 已建立。`, "success");
  });

  const missionCommand = (command) => perform(async () => {
    const payload = await api(`/api/missions/${mission.mission_id}/${command}`, { method: "POST" });
    setSystem((current) => ({ ...current, mission: payload.mission }));
    appendEvent(payload.mission.status_message, command === "stop" ? "danger" : "success");
  });

  const infer = () => perform(async () => {
    if (!imageBase64) throw new Error("请先加载一张测试图像。");
    const payload = await api(`/api/inference/${policy}`, {
      method: "POST",
      body: JSON.stringify({ image_base64: imageBase64, instruction, proprio: [0, 0, 0, 0] }),
    });
    setAction(payload.action_local_delta[0]);
    appendEvent(`${policy === "openvla" ? "OpenVLA" : "π0.5"} 单帧推理完成。`, "success");
  });

  const actionRows = useMemo(() => {
    if (!action) return [];
    return ["dx", "dy", "dz", "d_yaw"].map((label, index) => ({
      label,
      value: Number(action[index]).toFixed(5),
      unit: index === 3 ? "rad" : "m",
    }));
  }, [action]);

  return (
    <main className="shell">
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark">VA</span>
          <div><h1>VLA Ground Station</h1><p>Semantic intent · Safe local planning</p></div>
        </div>
        <div className="top-status">
          <span><StatusDot status={system.backend === "online" ? "online" : "offline"} />Backend</span>
          <span className={`lock ${system.safety_lock ? "locked" : "unlocked"}`}>
            {system.safety_lock ? "CONTROL LOCKED" : "LIVE CONTROL"}
          </span>
        </div>
      </header>

      <section className="metric-strip" aria-label="System summary">
        <div><span>Mission</span><strong>{missionState}</strong></div>
        <div><span>K-frame policy</span><strong>{onboard.k_frames ? `1 / ${onboard.k_frames}` : "—"}</strong></div>
        <div><span>OpenVLA p95</span><strong>134.8 ms</strong></div>
        <div><span>Command TTL</span><strong>500 ms</strong></div>
        <div><span>Host LAN</span><strong>{system.host_interfaces?.onboard_lan || "127.0.0.1"}</strong></div>
      </section>

      <section className="workspace">
        <div className="primary-column">
          <article className="panel vision-panel">
            <div className="panel-heading">
              <div><span className="eyebrow">Observation</span><h2>Camera workspace</h2></div>
              <label className="file-button">Load test frame<input type="file" accept="image/*" onChange={handleImage} /></label>
            </div>
            <div className={`camera-stage ${previewUrl ? "has-image" : ""}`}>
              {previewUrl ? <img src={previewUrl} alt="Selected UAV camera frame" /> : (
                <div className="camera-empty"><span>RGB</span><strong>No live stream connected</strong><p>Load a frame to validate the inference path.</p></div>
              )}
              <div className="camera-overlay top-left">FRAME <b>{onboard.image_sequence ?? "—"}</b> · NEXT VLA <b>{onboard.frames_until_inference ?? "—"}</b></div>
              <div className="camera-overlay bottom-right">PLANNER PREVIEW ONLY</div>
            </div>
            <div className="action-grid">
              {actionRows.length ? actionRows.map((item) => (
                <div key={item.label}><span>{item.label}</span><strong>{item.value}</strong><small>{item.unit}</small></div>
              )) : <p className="empty-action">No action prediction yet.</p>}
            </div>
          </article>

          <article className="panel event-panel">
            <div className="panel-heading"><div><span className="eyebrow">Audit log</span><h2>Recent events</h2></div></div>
            <div className="events">
              {events.map((event, index) => (
                <div className="event" key={`${event.time.getTime()}-${index}`}>
                  <time>{event.time.toLocaleTimeString("zh-CN", { hour12: false })}</time>
                  <span className={`event-line ${event.tone}`} />
                  <p>{event.text}</p>
                </div>
              ))}
            </div>
          </article>
        </div>

        <aside className="side-column">
          <article className="panel mission-panel">
            <div className="panel-heading"><div><span className="eyebrow">Mission intent</span><h2>Command composer</h2></div></div>
            <label className="field-label" htmlFor="instruction">Natural-language instruction</label>
            <textarea id="instruction" value={instruction} onChange={(event) => setInstruction(event.target.value)} disabled={!canCreate} />
            <div className="field-pair">
              <label><span>Policy</span><select value={policy} onChange={(event) => setPolicy(event.target.value)} disabled={!canCreate}><option value="openvla">OpenVLA 3ep</option><option value="pi05">π0.5 1ep</option></select></label>
              <label><span>Mode</span><select value={mode} onChange={(event) => setMode(event.target.value)} disabled={!canCreate}><option value="dry_run">Dry run</option><option value="live">Live</option></select></label>
            </div>
            <div className="button-stack">
              {canCreate ? <button className="primary" disabled={busy} onClick={createMission}>Create mission</button> : (
                <>
                  {missionState === "ARMED" && <button className="primary" disabled={busy} onClick={() => missionCommand("start")}>Start mission</button>}
                  {missionState === "RUNNING" && <button className="secondary" disabled={busy} onClick={() => missionCommand("hold")}>Hold position</button>}
                  {missionState === "HOLDING" && <button className="primary" disabled={busy} onClick={() => missionCommand("start")}>Resume mission</button>}
                  <button className="danger" disabled={busy} onClick={() => missionCommand("stop")}>Stop mission</button>
                </>
              )}
              <button className="ghost" disabled={busy || !imageBase64} onClick={infer}>Run single-frame inference</button>
            </div>
            {mission && <div className="mission-id"><span>Mission ID</span><code>{mission.mission_id}</code><p>{mission.status_message}</p></div>}
          </article>

          <article className="panel services-panel">
            <div className="panel-heading"><div><span className="eyebrow">Runtime</span><h2>Model services</h2></div></div>
            <Service name="OpenVLA · real" service={system.models?.openvla} />
            <Service name="π0.5 · UAV-Flow" service={system.models?.pi05} />
            <Service name="FAST-LIO + RGB uplink" service={{
              status: onboard.connected ? (onboard.calibration_validated ? "online" : "degraded") : "offline",
              detail: onboard.connected
                ? `${onboard.world_frame} → ${onboard.body_frame} · ${onboard.calibration_id}`
                : "Waiting for synchronized onboard observations",
            }} />
          </article>

          <article className="panel state-panel">
            <div className="panel-heading"><div><span className="eyebrow">FAST-LIO</span><h2>Local state (read-only)</h2></div></div>
            <div className="state-values">
              <span>x <b>{Number(onboard.local_state?.position?.x ?? 0).toFixed(3)}</b> m</span>
              <span>y <b>{Number(onboard.local_state?.position?.y ?? 0).toFixed(3)}</b> m</span>
              <span>z <b>{Number(onboard.local_state?.position?.z ?? 0).toFixed(3)}</b> m</span>
              <span>yaw <b>{Number(onboard.local_state?.yaw_rad ?? 0).toFixed(3)}</b> rad</span>
            </div>
          </article>

          <article className="safety-note">
            <span>Safety boundary</span>
            <p>Flight output, takeoff and arming remain disabled. Only synchronized observations and isolated Diff-Planner preview topics are enabled by this adapter.</p>
          </article>
        </aside>
      </section>
    </main>
  );
}

createRoot(document.getElementById("root")).render(<React.StrictMode><App /></React.StrictMode>);
