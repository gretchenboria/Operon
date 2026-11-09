// Operon phone & IMU controller. Connects to Operon Studio through the WebSocket relay.
const params = new URLSearchParams(location.search);
const ROOM = (params.get("room") || "OPERON").toUpperCase().replace(/[^A-Z0-9]/g, "").slice(0, 8);
const ROOM_OK = /^[A-Z0-9]{3,8}$/.test(ROOM);
const ES = (navigator.language || "").toLowerCase().startsWith("es");
const T = (en, es) => (ES ? es : en);
const $ = (id) => document.getElementById(id);

let myId;
try { myId = localStorage.getItem("df26.phoneId"); if (!myId) { myId = "p" + Math.random().toString(36).slice(2, 10); localStorage.setItem("df26.phoneId", myId); } }
catch { myId = "p" + Math.random().toString(36).slice(2, 10); }

// Friendly nickname so the presenter can pick a pilot by name.
const ADJ = ["Blue", "Swift", "Brave", "Lucky", "Turbo", "Quiet", "Golden", "Cosmic", "Rapid", "Clever", "Mighty", "Sunny"];
const ANI = ["Falcon", "Otter", "Fox", "Panda", "Gecko", "Lynx", "Koala", "Orca", "Heron", "Badger", "Tiger", "Robin"];
let myName;
try { myName = localStorage.getItem("df26.name"); } catch {}
if (!myName) {
  const h = [...myId].reduce((a, c) => (a * 31 + c.charCodeAt(0)) >>> 0, 7);
  myName = `${ADJ[h % ADJ.length]} ${ANI[(h >>> 4) % ANI.length]} ${h % 90 + 10}`;
  try { localStorage.setItem("df26.name", myName); } catch {}
}
let pilotId = null, pilotName = "", lastHintCls = "";

// ---------- i18n of static text ----------
const STATIC = {
  "idle-title": T("Connected to Operon Studio", "Conectado a Operon Studio"),
  "idle-sub": T("Drive the 4-DOF & 6-DOF digital twin and stream live accelerometer + gyroscope IMU telemetry.", "Controla el gemelo digital 4-DOF y 6-DOF y transmite telemetría IMU en vivo."),
  "t-team": T("Your arm", "Tu brazo"),
  "t-pose": T("Tilt to steer the gripper", "Inclina para mover la pinza"),
  "t-pose-sub": T("Phone flat, screen up = gripper pointing down. Tilt forward = pitch, sideways = roll, turn = yaw.", "Teléfono plano = pinza hacia abajo. Inclina al frente = pitch, de lado = roll, gira = yaw."),
  "enable-tilt": T("Enable Watch IMU", "Activar IMU"), zero: T("ZERO HERE (Set Neutral)", "CERO AQUÍ (Centrar)"), manual: T("Manual Angle Sliders", "Usar deslizadores"),
  "t-pad": T("Optional Touch Pad", "Pad táctil opcional"), "t-away": T("▲ away", "▲ lejos"),
  "t-base": T("▼ robot", "▼ robot"), "t-height": T("height", "altura"),
  "t-joints": T("You are a motor", "Eres un motor"),
  "t-joints-sub": T("These are the joints your phone drives. Your teammates hold the others.", "Estas son las articulaciones que mueves. Tu equipo tiene las demás.")
};
for (const [k, v] of Object.entries(STATIC)) if ($(k)) $(k).textContent = v;

// ---------- connection (WebSocket with resilient auto-recovery & HTTP fallback for watchOS WebKit) ----------
let ws = null, retry = 0, useHttp = false, httpQueue = [], httpTimer = null, lastNetOkAt = 0;
const status = $("status");
function setStatus(cls, txt) { status.className = `pill ${cls}`; status.textContent = txt; }

function stopHttpMode() {
  useHttp = false;
  if (httpTimer) { clearInterval(httpTimer); httpTimer = null; }
}

function startHttpMode() {
  if (useHttp) return;
  useHttp = true;
  if (ws) { try { ws.onclose = null; ws.close(); } catch {} ws = null; }
  setStatus("ok", `HTTP ${ROOM}`);
  httpQueue.push({ t: "hello", name: myName });
  let pollBusy = false;
  const poll = async () => {
    if (pollBusy || !useHttp) return;
    pollBusy = true;
    const batch = httpQueue.splice(0, httpQueue.length);
    try {
      const r = await fetch(`/api/room/${ROOM}/poll`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id: myId, msgs: batch }),
        cache: "no-store",
        keepalive: true
      });
      if (r.ok) {
        lastNetOkAt = performance.now();
        const d = await r.json();
        setStatus("ok", `HTTP ${ROOM}`);
        if (Array.isArray(d.msgs)) d.msgs.forEach(handle);
      } else {
        setStatus("warn", T("syncing...", "sincronizando..."));
      }
    } catch {
      setStatus("warn", T("retrying...", "reintentando..."));
    } finally {
      pollBusy = false;
    }
  };
  poll();
  // 100 ms (10 Hz) in HTTP fallback mode prevents watchOS WebKit memory pressure / WebContent jettison
  httpTimer = setInterval(poll, 100);
}

function connect() {
  if (!ROOM_OK) {
    setStatus("no", T("no room", "sin sala"));
    $("idle-title").textContent = T("Scan the QR code on the big screen", "Escanea el código QR de la pantalla");
    $("idle-sub").textContent = T("This link is missing its room code.", "A este enlace le falta el código de sala.");
    return;
  }
  if (ws && (ws.readyState === 0 || ws.readyState === 1)) return;
  stopHttpMode();
  const proto = location.protocol === "https:" ? "wss" : "ws";
  setStatus("warn", T("connecting…", "conectando…"));
  const fallbackTimeout = setTimeout(() => {
    if (!ws || ws.readyState !== 1) startHttpMode();
  }, 2600);
  try {
    ws = new WebSocket(`${proto}://${location.host}/api/room/${ROOM}/ws?role=phone&id=${myId}`);
    ws.onopen = () => {
      clearTimeout(fallbackTimeout);
      stopHttpMode();
      retry = 0;
      lastNetOkAt = performance.now();
      setStatus("ok", `${T("room", "sala")} ${ROOM}`);
      send({ t: "hello", name: myName });
    };
    ws.onmessage = (e) => {
      lastNetOkAt = performance.now();
      let m; try { m = JSON.parse(e.data); } catch { return; }
      handle(m);
    };
    ws.onerror = () => { clearTimeout(fallbackTimeout); if (retry >= 2) startHttpMode(); };
    ws.onclose = () => {
      clearTimeout(fallbackTimeout);
      ws = null;
      if (useHttp) return;
      if (retry >= 2) { startHttpMode(); return; }
      setStatus("warn", T("reconnecting…", "reconectando…"));
      setTimeout(connect, Math.min(2000, 300 * 2 ** retry++));
    };
  } catch {
    clearTimeout(fallbackTimeout);
    startHttpMode();
  }
}

function ensureConnected() {
  if (!ws || ws.readyState > 1) {
    retry = 0;
    connect();
  }
}

const send = (m) => {
  if (useHttp) {
    if (m.t === "pose") {
      const idx = httpQueue.findIndex(x => x.t === "pose" && !x.v?.grabToggle && !x.v?.snap);
      if (idx >= 0 && !m.v?.grabToggle && !m.v?.snap) { httpQueue[idx] = m; return; }
    }
    if (httpQueue.length < 15) httpQueue.push(m);
  } else if (ws && ws.readyState === 1) {
    try { ws.send(JSON.stringify(m)); } catch {}
  }
};
connect();
setInterval(() => { if (!useHttp && ws && ws.readyState === 1) send({ t: "ping" }); }, 15000);

// ---------- state ----------
let slide = "teleop", deckSlide = "idle", mode = "pose", team = 6, assigned = [], vote = null, quizQ = null;
let station = { n: 1, name: T("[1] Pick & place cell", "[1] Tomar y colocar celda") };
const DEFAULT_QUIZ = {
  q: 0,
  text: T("Someone shouts 'stop!' while the arm is moving.", "Alguien grita 'alto!' mientras el brazo se mueve."),
  revealed: false,
  answer: null,
  options: [
    { name: T("Reflex", "Reflejo"), desc: T("deterministic code", "codigo determinista") },
    { name: T("Cloud Model", "Modelo en nube"), desc: T("Layer 2", "Capa 2") },
    { name: T("On-device model", "Modelo local"), desc: T("local inference", "inferencia local") },
    { name: T("Escalate + safe stop", "Escalar + alto seguro"), desc: T("safe halt", "parada segura") },
    { name: T("Refuse · fail closed", "Negar · falla cerrada"), desc: T("zero bytes to motors", "cero bytes a los motores") }
  ]
};
let lastQuizMsg = DEFAULT_QUIZ;
if ($("hint")) $("hint").textContent = T("1. Hold wrist comfortably in front of screen & tap ZERO HERE · 2. Move ±14° to steer & tap GRASP", "1. Pon la muñeca frente a la pantalla y toca CERO AQUÍ · 2. Mueve ±14° y toca AGARRAR");
if ($("tab-teleop")) $("tab-teleop").textContent = T("Robot Teleop", "Teleop Robot");
if ($("tab-quiz")) $("tab-quiz").textContent = T("Live Quiz", "Quiz en Vivo");
if ($("mode-bar")) {
  $("mode-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
    slide = b.dataset.mode;
    if (slide === "quiz") renderQuiz(lastQuizMsg);
    render();
  }));
}
function show(ids) {
  for (const id of ["p-idle", "p-team", "p-pose", "p-joints", "p-quiz", "p-pilot", "p-station"]) $(id).classList.toggle("hide", !ids.includes(id));
}
function render() {
  $("me-name").textContent = `${T("you are", "eres")} ${myName}`;
  $("pilot-who").textContent = slide === "teleop" && pilotId ? `[PILOT] ${pilotId === myId ? T("you", "tu") : pilotName}` : "";
  $("pilot-title").textContent = T("Active Pilot", "Piloto Activo");
  $("pilot-sub").textContent = T("Your device controls the 6-DOF end-effector pose and IMU stream.", "Tu dispositivo controla la pose 6-DOF y telemetria IMU.");
  if ($("mode-bar")) $("mode-bar").querySelectorAll("button").forEach(b => b.classList.toggle("sel", b.dataset.mode === slide));
  if (station) { $("st-n").textContent = `${T("Mission", "Misión")} · ${station.n}/3`; $("st-name").textContent = station.name; }
  if (slide === "teleop") {
    const ids = [mode === "pose" ? "p-pose" : "p-joints"];
    if (station) ids.unshift("p-station");
    if (pilotId === myId && mode === "pose") ids.unshift("p-pilot");
    if (deckSlide === "idle") ids.unshift("p-idle");
    show(ids);
  }
  else if (slide === "quiz") {
    const ids = ["p-quiz"];
    if (deckSlide === "idle") ids.unshift("p-idle");
    show(ids);
  }
  else show(["p-idle", "p-pose"]);
  $("team").querySelectorAll("button").forEach(b => b.classList.toggle("sel", +b.dataset.v === team));
}

function handle(m) {
  if (m.t === "state") {
    deckSlide = m.slide || "idle";
    if (m.mode) mode = m.mode;
    if (deckSlide === "idle") {
      if (m.title) $("idle-title").textContent = m.title;
      if ($("idle-sync")) $("idle-sync").textContent = T("controller ready", "control listo");
    } else {
      slide = deckSlide;
    }
    if (deckSlide === "quiz") { lastQuizMsg = m; renderQuiz(m); }
    if (deckSlide === "teleop") {
      station = m.station || station;
      pilotId = m.pilot?.id || null;
      pilotName = m.pilot?.name || "";
      if (m.closed !== undefined) { pose.g = m.closed ? 1 : 0; paintGrab(); }
    }
    render();
  } else if (m.t === "grip") {
    pose.g = m.g ? 1 : 0;
    paintGrab();
  } else if (m.t === "assign") {
    team = 6; mode = m.mode || mode; assigned = m.joints || [];
    renderJoints(); render();
  } else if (m.t === "pilot") {
    pilotId = m.id || null; pilotName = m.name || ""; if (navigator.vibrate && pilotId === myId) navigator.vibrate([80, 60, 80]); render();
  } else if (m.t === "hint") {
    const h = m.h6 || m.h4;
    if (h) {
      const el = $("hint");
      el.innerHTML = `${(m.station || "")} ${h.msg}${h.dir && h.cls !== "ok" ? `<div style="font:500 15px var(--mono);margin-top:6px">${h.dir}</div>` : ""}`;
      el.style.background = h.cls === "ok" ? "var(--pg)" : h.cls === "warn" ? "var(--py)" : "var(--pr)";
      if (h.cls !== lastHintCls && h.cls !== "no" && navigator.vibrate) navigator.vibrate(h.cls === "ok" ? [60, 40, 60] : 40);
      lastHintCls = h.cls;
    }
  } else if (m.t === "score") {
    for (const k of ["4", "6"]) {
      const e = m["e" + k]; if (!e) continue;
      const ok = e.pos < 12 && e.ori < 10;
      $("sc" + k).className = ok ? "ok" : e.ori > 25 ? "no" : "warn";
      $("sc" + k).innerHTML = `${k}-DOF${+k === team ? " · " + T("you", "tú") : ""}<b>${e.pos} mm · ${e.ori}°</b>`;
    }
  }
}

$("team").querySelectorAll("button").forEach(b => b.addEventListener("click", () => { team = 6; send({ t: "team", team }); render(); }));

// ---------- pose: tilt + pad + live IMU + Watch Wake Lock ----------
const pose = { yaw: 0, pitch: 0, roll: 0, g: 0, ax: 0, ay: 0, az: 9.81, gx: 0, gy: 0, gz: 0, tap: 0 };
let zero = null, tiltOn = false, manualOn = false, lastO = null, touched = false;
let deskGain = 2.4; // Default 2.4x Smooth Mode: comfortable, steady wrist movement without over-sensitivity
let wearMode = "left"; // "left" (Left Wrist Watch) | "right" (Right Wrist Watch) | "phone" (Phone in Hand)
let flipLR = 1, flipUD = 1;
let fusedAtt = { yaw: 0, pitch: 0, roll: 0, lastT: 0 };
let wakeLockSentinel = null;

// Safe W3C Screen Wake Lock (NO <video>.play() or AudioContext which cause watchOS WebKit to show a blank black media screen or jettison WebContent!)
async function acquireWatchWakeLock() {
  try {
    if ("wakeLock" in navigator && navigator.wakeLock?.request && (!wakeLockSentinel || wakeLockSentinel.released)) {
      wakeLockSentinel = await navigator.wakeLock.request("screen");
    }
  } catch {}
}

function onWatchWakeOrInteract() {
  acquireWatchWakeLock();
  ensureConnected();
}

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible") onWatchWakeOrInteract();
});
window.addEventListener("pageshow", onWatchWakeOrInteract);
window.addEventListener("focus", onWatchWakeOrInteract);
window.addEventListener("touchstart", onWatchWakeOrInteract, { passive: true });
window.addEventListener("pointerdown", onWatchWakeOrInteract, { passive: true });

// Apple Watch Digital Crown support: turning the Digital Crown fires `wheel` events, which both resets watchOS's 15s screen dim timer
// AND lets you smoothly raise/lower gripper height (vz) without lifting your wrist!
let crownTimeout = null;
window.addEventListener("wheel", (e) => {
  onWatchWakeOrInteract();
  if (Math.abs(e.deltaY) > 0.5) {
    vz = e.deltaY < 0 ? 0.65 : -0.65;
    if (crownTimeout) clearTimeout(crownTimeout);
    crownTimeout = setTimeout(() => { vz = 0; }, 220);
  }
}, { passive: true });

// Wear Mode selector (Left Wrist Watch vs Right Wrist Watch vs Phone in Hand)
if ($("wrist-side")) {
  $("wrist-side").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
    wearMode = b.dataset.w || "left";
    $("wrist-side").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
    zero = null;
    touched = true;
  }));
}

// Instant 1-tap Axis Flip buttons so user never fights inverted directions
if ($("flip-lr")) {
  $("flip-lr").addEventListener("click", () => {
    flipLR *= -1;
    $("flip-lr").classList.toggle("sel", flipLR < 0);
    $("flip-lr").textContent = flipLR < 0 ? "Left/Right: Flipped" : "Flip Left/Right";
  });
}
if ($("flip-ud")) {
  $("flip-ud").addEventListener("click", () => {
    flipUD *= -1;
    $("flip-ud").classList.toggle("sel", flipUD < 0);
    $("flip-ud").textContent = flipUD < 0 ? "Up/Down: Flipped" : "Flip Up/Down";
  });
}

// Gain selector on Watch / Phone (1.5x Standard, 3.2x Desk Mode, 4.8x Micro)
if ($("gain-bar")) {
  $("gain-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
    deskGain = parseFloat(b.dataset.g) || 3.2;
    $("gain-bar").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
    touched = true;
  }));
}

// Anatomical Forearm / Hand Frame -> Screen-Centric Robot Frame Mapping:
// On a Watch strapped to your Left Wrist (+X_device = toward hand/fingers, +Y_device = toward thumb/anterior, +Z_device = out of watch face):
// - Raising/Lowering forearm or tilting thumb up/down changes gravity along X/Y
// - Supinating/Pronating forearm rotates around X_device (beta)
// - Sweeping forearm Left/Right rotates around Z_device (alpha)
function mapDeviceAnglesToRobot(rawYaw, rawPitch, rawRoll) {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const wrap90 = (v) => {
    let w = ((v + 180) % 360 + 360) % 360 - 180;
    if (w > 90) w = 180 - w;
    if (w < -90) w = -180 - w;
    return w;
  };
  // Soft 2.2-deg jitter deadzone so holding your wrist steady keeps the robot 100% locked
  const dz = (v, d = 2.2) => Math.abs(v) < d ? 0 : (v > 0 ? v - d : v + d);
  const y = dz(wrap90(rawYaw), 2.2), p = dz(wrap90(rawPitch), 2.2), r = dz(wrap90(rawRoll), 2.2);
  let lrDeg, udDeg, twistDeg;
  if (wearMode === "left") {
    // Left Wrist Watch held comfortably facing screen
    lrDeg = (-y * 0.85 + r * 0.45) * flipLR;
    udDeg = (p * 0.88) * flipUD;
    twistDeg = r * 0.85;
  } else if (wearMode === "right") {
    // Right Wrist Watch (+X_device points left toward fingers)
    lrDeg = (-y * 0.85 - r * 0.45) * flipLR;
    udDeg = (-p * 0.88) * flipUD;
    twistDeg = -r * 0.85;
  } else {
    // Phone held in hand pointing at screen
    lrDeg = (-y * 0.78 - r * 0.65) * flipLR;
    udDeg = (p * 0.88) * flipUD;
    twistDeg = r * 0.85;
  }
  const targetYaw = clamp(lrDeg * deskGain, -80, 80);
  const targetPitch = clamp(udDeg * deskGain, -70, 70);
  const targetRoll = clamp(twistDeg * deskGain, -80, 80);
  // Smooth low-pass filter so wrist motion glides steadily without twitchiness
  pose.yaw = Math.round(pose.yaw * 0.55 + targetYaw * 0.45);
  pose.pitch = Math.round(pose.pitch * 0.55 + targetPitch * 0.45);
  pose.roll = Math.round(pose.roll * 0.55 + targetRoll * 0.45);
}

function onMotion(e) {
  const a = e.accelerationIncludingGravity || e.acceleration;
  const r = e.rotationRate;
  const now = performance.now();
  // If watchOS dimmed/slept for > 1.2s and just woke up on wrist motion, immediately restore WebSocket!
  if (fusedAtt.lastT && (now - fusedAtt.lastT) > 1200) {
    ensureConnected();
  }
  const dt = fusedAtt.lastT ? Math.min(0.08, Math.max(0.002, (now - fusedAtt.lastT) / 1000)) : 0.016;
  fusedAtt.lastT = now;

  if (a && (a.x != null || a.y != null || a.z != null)) {
    pose.ax = +(a.x || 0).toFixed(3);
    pose.ay = +(a.y || 0).toFixed(3);
    pose.az = +(a.z || 0).toFixed(3);
    const mag = Math.hypot(pose.ax, pose.ay, pose.az);
    if (mag > 16.5) pose.tap = 1;
  }
  if (r && (r.alpha != null || r.beta != null || r.gamma != null)) {
    const d2r = Math.PI / 180;
    pose.gx = +((r.beta || 0) * d2r).toFixed(3);
    pose.gy = +((r.gamma || 0) * d2r).toFixed(3);
    pose.gz = +((r.alpha || 0) * d2r).toFixed(3);
  }

  // High-rate 6-Axis Complementary Filter (fuses 60Hz Gyro rate with gravity vector so attitude is rock-solid)
  const r2d = 180 / Math.PI;
  const gMag = Math.hypot(pose.gx, pose.gy, pose.gz);
  const aDelta = Math.abs(Math.hypot(pose.ax, pose.ay, pose.az) - 9.80665);
  const isWristStill = gMag < 0.09 && aDelta < 0.45;

  // Use hypot(ax, az) in denominator so watch face verticality (az crossing 0) NEVER flips +/-180 deg!
  const accElev = wearMode === "phone"
    ? Math.atan2(-pose.ay, Math.max(1.5, Math.hypot(pose.ax, pose.az))) * r2d
    : Math.atan2(pose.ax, Math.max(1.5, Math.hypot(pose.ay, pose.az))) * r2d;
  const accTwist = wearMode === "phone"
    ? Math.atan2(pose.ax, Math.max(1.5, Math.hypot(pose.ay, pose.az))) * r2d
    : Math.atan2(pose.ay, Math.max(1.5, Math.hypot(pose.ax, pose.az))) * r2d;

  if (!isWristStill) {
    const pRate = (wearMode === "phone" ? pose.gx : pose.gy) * r2d;
    const rRate = (wearMode === "phone" ? pose.gy : pose.gx) * r2d;
    fusedAtt.pitch = 0.90 * (fusedAtt.pitch + (Math.abs(pRate) > 2.0 ? pRate * dt : 0)) + 0.10 * accElev;
    fusedAtt.roll = 0.90 * (fusedAtt.roll + (Math.abs(rRate) > 2.0 ? rRate * dt : 0)) + 0.10 * accTwist;
    const gzDegS = Math.abs(pose.gz * r2d) > 2.5 ? pose.gz * r2d : 0;
    fusedAtt.yaw = Math.max(-75, Math.min(75, fusedAtt.yaw * 0.995 + gzDegS * dt));
  } else {
    // Stationary wrist lock: gently damp towards gravity reference without integrating gyro drift
    fusedAtt.pitch = 0.96 * fusedAtt.pitch + 0.04 * accElev;
    fusedAtt.roll = 0.96 * fusedAtt.roll + 0.04 * accTwist;
  }

  if (!zero) {
    fusedAtt.pitch = accElev;
    fusedAtt.roll = accTwist;
    zero = { a: lastO ? lastO.a : fusedAtt.yaw, b: accElev, g: accTwist, fPitch: accElev, fRoll: accTwist };
  }
  if (zero.fPitch === undefined) { zero.fPitch = accElev; zero.fRoll = accTwist; }

  let yaw = lastO && (now - (lastO.t || 0) <= 250) ? (lastO.a - zero.a) : (fusedAtt.yaw - zero.a);
  while (yaw > 180) yaw -= 360;
  while (yaw < -180) yaw += 360;
  // Ignore uncalibrated compass jumps if gyro shows the wrist is stationary
  if (isWristStill && Math.abs(pose.gz * r2d) < 2.5 && fusedAtt.lastYaw !== undefined) {
    yaw = fusedAtt.lastYaw;
  } else {
    fusedAtt.lastYaw = yaw;
  }

  const fPitch = fusedAtt.pitch - zero.fPitch;
  const fRoll = fusedAtt.roll - zero.fRoll;
  mapDeviceAnglesToRobot(yaw, fPitch, fRoll);
  tiltOn = true;
  readout();
}

function onOrient(e) {
  if (e.alpha == null && e.beta == null) return;
  const now = performance.now();
  lastO = { a: e.alpha || 0, b: e.beta || 0, g: e.gamma || 0, t: now };
  if (!zero) zero = { ...lastO, fPitch: fusedAtt.pitch, fRoll: fusedAtt.roll };
  // If 60Hz DeviceMotion (onMotion) is active, let onMotion exclusively compute pose so onOrient never fights it with Euler gimbal jumps!
  if (fusedAtt.lastT && (now - fusedAtt.lastT) < 250) return;
  let yaw = lastO.a - zero.a; while (yaw > 180) yaw -= 360; while (yaw < -180) yaw += 360;
  const rawPitch = wearMode === "phone" ? (lastO.b - zero.b) : -(lastO.g - zero.g);
  const rawRoll = wearMode === "phone" ? (lastO.g - zero.g) : (lastO.b - zero.b);
  mapDeviceAnglesToRobot(yaw, rawPitch, rawRoll);
  tiltOn = true;
  readout();
}
function readout() {
  $("r-yaw").textContent = pose.yaw + "°"; $("r-pitch").textContent = pose.pitch + "°"; $("r-roll").textContent = pose.roll + "°";
}

// Auto-bind motion/orientation listeners immediately on devices that don't require explicit permission prompt
try {
  if (typeof DeviceOrientationEvent !== "undefined" && typeof DeviceOrientationEvent.requestPermission !== "function") {
    window.addEventListener("deviceorientation", onOrient);
  }
  if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission !== "function") {
    window.addEventListener("devicemotion", onMotion);
  }
} catch {}

async function activateMotionSensorsFromUserGesture() {
  try {
    if (!window.isSecureContext) throw new Error("needs https");
    // CRITICAL FOR iOS / watchOS WebKit:
    // DeviceOrientationEvent.requestPermission() and DeviceMotionEvent.requestPermission()
    // MUST be invoked synchronously on the very first line of the click/touchend handler
    // BEFORE any other `await` (like wakeLock), otherwise Safari drops the transient user-activation token!
    const permPromises = [];
    if (typeof DeviceOrientationEvent !== "undefined" && typeof DeviceOrientationEvent.requestPermission === "function") {
      permPromises.push(DeviceOrientationEvent.requestPermission().catch(() => "denied"));
    }
    if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
      permPromises.push(DeviceMotionEvent.requestPermission().catch(() => "denied"));
    }
    const results = permPromises.length ? await Promise.all(permPromises) : ["granted"];
    if (results.every(r => r !== "granted")) {
      throw new Error("permission denied");
    }

    window.addEventListener("deviceorientation", onOrient);
    window.addEventListener("devicemotion", onMotion);
    tiltOn = true; zero = null; fusedAtt = { yaw: 0, pitch: 0, roll: 0, lastT: 0 };
    aim = "tilt";
    $("aim")?.querySelectorAll("button").forEach(x => x.classList.toggle("sel", x.dataset.v === "tilt"));
    $("tilt-box")?.classList.remove("hide");
    $("enable-tilt").textContent = T("Wrist IMU + Screen Wake Lock Active ✓", "IMU + Pantalla Activa ✓");
    $("enable-tilt").classList.add("light");

    // Acquire Screen Wake Lock & WebAudio keep-alive AFTER sensor permission is granted
    acquireWatchWakeLock();
  } catch (err) {
    $("enable-tilt").textContent = T(`Tap here to allow Wrist Motion (${err.message})`, `Toca aquí para activar sensor (${err.message})`);
    openManual();
  }
}

$("enable-tilt").addEventListener("click", activateMotionSensorsFromUserGesture);

// Also trigger sensor permission on the first tap of Zero, Align, or Grasp if not yet active
$("zero").addEventListener("click", () => {
  if (!tiltOn) activateMotionSensorsFromUserGesture();
  else acquireWatchWakeLock();
  zero = lastO
    ? { ...lastO, fPitch: fusedAtt.pitch, fRoll: fusedAtt.roll }
    : { a: fusedAtt.yaw, b: fusedAtt.pitch, g: fusedAtt.roll, fPitch: fusedAtt.pitch, fRoll: fusedAtt.roll };
  pose.yaw = pose.pitch = pose.roll = 0;
  readout();
  send({ t: "pose", v: { vx: 0, vy: 0, vz: 0, g: pose.g, aim, gain: deskGain, yaw: 0, pitch: 0, roll: 0, recenter: 1 } });
});
function openManual() { manualOn = true; $("manual-box").classList.remove("hide"); }
$("manual").addEventListener("click", () => { if (manualOn) { manualOn = false; $("manual-box").classList.add("hide"); } else openManual(); });
for (const k of ["yaw", "pitch", "roll"]) $("m-" + k).addEventListener("input", (e) => { pose[k] = +e.target.value; $("mv-" + k).textContent = pose[k] + "°"; readout(); });

function paintGrab() {
  $("grab").textContent = pose.g ? T("RELEASE", "SOLTAR") : T("GRASP", "AGARRAR");
  $("grab").style.background = pose.g ? "var(--pr)" : "var(--pg)";
}
$("grab").addEventListener("click", () => {
  touched = true;
  pose.g = pose.g ? 0 : 1;
  paintGrab();
  send({ t: "pose", v: { vx: -jy, vy: -jx, vz, g: pose.g, grabToggle: 1, aim, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll } });
  if (navigator.vibrate) navigator.vibrate(30);
});
if ($("snap")) {
  $("snap").textContent = T("ALIGN", "ALINEAR");
  $("snap").addEventListener("click", () => {
    touched = true;
    send({ t: "pose", v: { vx: 0, vy: 0, vz: 0, g: pose.g, aim, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, snap: 1 } });
    if (navigator.vibrate) navigator.vibrate([30, 30, 50]);
  });
}
paintGrab();

// Joystick (springs back to center) = hand velocity in the top view; ▲/▼ = height velocity.
const stick = $("stick"), knob = $("knob");
let jx = 0, jy = 0, vz = 0, aim = "tilt";
function stickMove(e) {
  const r = stick.getBoundingClientRect(), cx = r.left + r.width / 2, cy = r.top + r.height / 2, R = r.width * 0.36;
  let dx = (e.clientX - cx) / R, dy = (e.clientY - cy) / R; const m = Math.hypot(dx, dy); if (m > 1) { dx /= m; dy /= m; }
  jx = dx; jy = dy; knob.style.left = `${33 + dx * 33}%`; knob.style.top = `${33 + dy * 33}%`;
}
const stickEnd = () => { jx = jy = 0; knob.style.left = knob.style.top = "33%"; };
stick.addEventListener("pointerdown", (e) => { touched = true; acquireWatchWakeLock(); stick.setPointerCapture(e.pointerId); stickMove(e); });
stick.addEventListener("pointermove", (e) => { if (e.buttons || e.pointerType === "touch") stickMove(e); });
stick.addEventListener("pointerup", stickEnd); stick.addEventListener("pointercancel", stickEnd);
for (const [id, v] of [["up", 1], ["down", -1]]) {
  const b = $(id);
  b.addEventListener("pointerdown", (e) => { touched = true; acquireWatchWakeLock(); b.setPointerCapture(e.pointerId); vz = v; });
  for (const ev of ["pointerup", "pointercancel", "pointerleave"]) b.addEventListener(ev, () => { vz = 0; });
}
$("aim").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
  aim = b.dataset.v; $("aim").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  $("tilt-box").classList.toggle("hide", aim !== "tilt"); touched = true;
}));
// Stream continuously at 20 Hz even without screen touches so Apple Watch IMU drives the arms hands-free
setInterval(() => {
  if (slide === "teleop" && mode === "pose") {
    const tMs = Date.now();
    send({ t: "pose", v: { vx: -jy, vy: -jx, vz, g: pose.g, aim, gain: deskGain, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, ax: pose.ax, ay: pose.ay, az: pose.az, gx: pose.gx, gy: pose.gy, gz: pose.gz, tap: pose.tap, utc_ms: tMs, utc_iso: new Date(tMs).toISOString() } });
    pose.tap = 0;
  }
}, 50);

// ---------- joints ----------
const jointVals = {};
function renderJoints() {
  const box = $("joints");
  if (!assigned.length) { box.innerHTML = `<p class="sub">${T("Waiting for an assignment…", "Esperando asignación…")}</p>`; return; }
  box.innerHTML = assigned.map(j => `
    <div class="js"><div class="row"><span class="lbl">${team}-DOF · J${j.i + 1} · ${j.n}</span><span class="v" id="jv-${j.i}">${j.v}°</span></div>
    <input type="range" data-j="${j.i}" min="${j.lim[0]}" max="${j.lim[1]}" value="${j.v}"></div>`).join("");
  box.querySelectorAll("input").forEach(inp => inp.addEventListener("input", () => {
    jointVals[inp.dataset.j] = +inp.value; $("jv-" + inp.dataset.j).textContent = inp.value + "°";
  }));
  assigned.forEach(j => { jointVals[j.i] = j.v; }); // the deck is authoritative on (re)assignment
}
setInterval(() => {
  if (slide === "teleop" && mode === "joints" && assigned.length) {
    const v = {}; assigned.forEach(j => v[j.i] = jointVals[j.i]); send({ t: "joints", v });
  }
}, 120);

// ---------- quiz ----------
function renderQuiz(m) {
  if (quizQ !== m.q) { vote = null; quizQ = m.q; }
  $("q-n").textContent = `${T("Question", "Pregunta")} ${m.q + 1}`;
  $("q-text").textContent = m.text || "";
  const box = $("choices");
  const right = m.revealed && Number.isInteger(m.answer) ? m.answer : null;
  box.innerHTML = (m.options || []).map((o, i) => `<button class="choice ${vote === i ? "sel" : ""}" data-i="${i}" style="${right === i ? "background:var(--pg)" : right !== null && vote === i ? "background:var(--pr)" : ""}">${o.name}${right === i ? " ✓" : ""}<small>${o.desc || ""}</small></button>`).join("");
  box.querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
    if (m.revealed) return;
    vote = +b.dataset.i; send({ t: "vote", q: m.q, choice: vote });
    box.querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
    $("q-note").textContent = T("Vote sent. You can change it until the reveal.", "Voto enviado. Puedes cambiarlo antes de revelar.");
  }));
  $("q-note").textContent = m.revealed ? (vote === null ? T("Answer revealed on the big screen.", "Respuesta revelada en la pantalla.") : vote === right ? T("Correct.", "Correcto.") : T("Not this time. See why on the big screen.", "Esta vez no. Mira por que en la pantalla.")) : vote === null ? T("Tap your answer.", "Toca tu respuesta.") : T("Vote sent.", "Voto enviado.");
}

render();
