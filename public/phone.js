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
  "enable-tilt": T("Enable tilt", "Activar inclinación"), zero: T("Zero", "Cero"), manual: T("Sliders instead", "Usar deslizadores"),
  "t-pad": T("Move the hand", "Mueve la mano"), "t-away": T("▲ away", "▲ lejos"),
  "t-base": T("▼ robot", "▼ robot"), "t-height": T("height", "altura"),
  "t-joints": T("You are a motor", "Eres un motor"),
  "t-joints-sub": T("These are the joints your phone drives. Your teammates hold the others.", "Estas son las articulaciones que mueves. Tu equipo tiene las demás.")
};
for (const [k, v] of Object.entries(STATIC)) if ($(k)) $(k).textContent = v;

// ---------- connection (WebSocket with instant HTTP fallback for watchOS WebKit) ----------
let ws = null, retry = 0, useHttp = false, httpQueue = [], httpTimer = null;
const status = $("status");
function setStatus(cls, txt) { status.className = `pill ${cls}`; status.textContent = txt; }

function startHttpMode() {
  if (useHttp) return;
  useHttp = true;
  if (ws) { try { ws.onclose = null; ws.close(); } catch {} ws = null; }
  setStatus("ok", `HTTP ${ROOM}`);
  httpQueue.push({ t: "hello", name: myName });
  const poll = async () => {
    const batch = httpQueue.splice(0, httpQueue.length);
    try {
      const r = await fetch(`/api/room/${ROOM}/poll`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id: myId, msgs: batch }),
        cache: "no-store"
      });
      if (r.ok) {
        const d = await r.json();
        setStatus("ok", `HTTP ${ROOM}`);
        if (Array.isArray(d.msgs)) d.msgs.forEach(handle);
      } else {
        setStatus("warn", T("syncing...", "sincronizando..."));
      }
    } catch {
      setStatus("warn", T("retrying...", "reintentando..."));
    }
  };
  poll();
  httpTimer = setInterval(poll, 220);
}

function connect() {
  if (!ROOM_OK) {
    setStatus("no", T("no room", "sin sala"));
    $("idle-title").textContent = T("Scan the QR code on the big screen", "Escanea el código QR de la pantalla");
    $("idle-sub").textContent = T("This link is missing its room code.", "A este enlace le falta el código de sala.");
    return;
  }
  if (useHttp) return;
  const proto = location.protocol === "https:" ? "wss" : "ws";
  setStatus("warn", T("connecting…", "conectando…"));
  const fallbackTimeout = setTimeout(() => {
    if (!ws || ws.readyState !== 1) startHttpMode();
  }, 1200);
  try {
    ws = new WebSocket(`${proto}://${location.host}/api/room/${ROOM}/ws?role=phone&id=${myId}`);
    ws.onopen = () => {
      clearTimeout(fallbackTimeout);
      retry = 0;
      setStatus("ok", `${T("room", "sala")} ${ROOM}`);
      send({ t: "hello", name: myName });
    };
    ws.onmessage = (e) => { let m; try { m = JSON.parse(e.data); } catch { return; } handle(m); };
    ws.onerror = () => { clearTimeout(fallbackTimeout); startHttpMode(); };
    ws.onclose = () => {
      clearTimeout(fallbackTimeout);
      if (useHttp) return;
      if (retry >= 1) { startHttpMode(); return; }
      setStatus("no", T("reconnecting…", "reconectando…"));
      setTimeout(connect, Math.min(3000, 400 * 2 ** retry++));
    };
  } catch {
    clearTimeout(fallbackTimeout);
    startHttpMode();
  }
}
const send = (m) => {
  if (useHttp) {
    if (m.t === "pose") {
      const idx = httpQueue.findIndex(x => x.t === "pose" && !x.v?.grabToggle && !x.v?.snap);
      if (idx >= 0 && !m.v?.grabToggle && !m.v?.snap) { httpQueue[idx] = m; return; }
    }
    httpQueue.push(m);
  } else if (ws && ws.readyState === 1) {
    ws.send(JSON.stringify(m));
  }
};
connect();
setInterval(() => { if (!useHttp) send({ t: "ping" }); }, 25000);

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
if ($("hint")) $("hint").textContent = T("Drag joystick to steer · tap ALIGN or GRASP", "Arrastra el joystick · toca ALINEAR o AGARRAR");
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
    const ids = ["p-team", mode === "pose" ? "p-pose" : "p-joints"];
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
  else show(["p-idle", "p-team", "p-pose"]);
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
    team = m.team; mode = m.mode || mode; assigned = m.joints || [];
    renderJoints(); render();
  } else if (m.t === "pilot") {
    pilotId = m.id || null; pilotName = m.name || ""; if (navigator.vibrate && pilotId === myId) navigator.vibrate([80, 60, 80]); render();
  } else if (m.t === "hint") {
    const h = team === 4 ? m.h4 : m.h6;
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

$("team").querySelectorAll("button").forEach(b => b.addEventListener("click", () => { team = +b.dataset.v; send({ t: "team", team }); render(); }));

// ---------- pose: tilt + pad + live IMU + Wrist Clench / Flick Grip Detector + Watch Wake Lock ----------
const pose = { yaw: 0, pitch: 0, roll: 0, g: 0, ax: 0, ay: 0, az: 9.81, gx: 0, gy: 0, gz: 0, tap: 0 };
let zero = null, tiltOn = false, manualOn = false, lastO = null, touched = false, lastGestureAt = 0;
let deskGain = 2.5; // Default 2.5x Desk Mode: +/-22 deg comfortable wrist movement in front of screen spans full robot workspace
let fusedAtt = { yaw: 0, pitch: 0, roll: 0, lastT: 0 };
let wakeLockSentinel = null, audioKeepAliveCtx = null;

// Prevent Apple Watch / iOS Safari from sleeping, dimming, or freezing JS timers while wrist is held in front of screen
async function acquireWatchWakeLock() {
  // 1. Standard W3C Screen Wake Lock API (supported in modern iOS / watchOS WebKit)
  try {
    if ("wakeLock" in navigator && navigator.wakeLock?.request) {
      wakeLockSentinel = await navigator.wakeLock.request("screen");
    }
  } catch {}
  // 2. WebAudio silent oscillator keep-alive (prevents watchOS WebKit from suspending the JS event loop when wrist lowers slightly)
  try {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (AC && !audioKeepAliveCtx) {
      audioKeepAliveCtx = new AC();
      const osc = audioKeepAliveCtx.createOscillator();
      const gain = audioKeepAliveCtx.createGain();
      gain.gain.value = 0.00001; // inaudible
      osc.connect(gain);
      gain.connect(audioKeepAliveCtx.destination);
      osc.start();
    } else if (audioKeepAliveCtx && audioKeepAliveCtx.state === "suspended") {
      await audioKeepAliveCtx.resume();
    }
  } catch {}
  // 3. Tiny 1x1 muted canvas stream video loop (prevents mobile Safari screen dim timeout)
  try {
    const vid = $("wake-video");
    if (vid && !vid.srcObject) {
      const cv = document.createElement("canvas");
      cv.width = cv.height = 2;
      const ctx = cv.getContext("2d");
      setInterval(() => { if (ctx) { ctx.fillStyle = "#000"; ctx.fillRect(0, 0, 2, 2); } }, 1000);
      if (cv.captureStream) {
        vid.srcObject = cv.captureStream(1);
        vid.play().catch(() => {});
      }
    }
  } catch {}
}

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible") {
    acquireWatchWakeLock();
    if (!useHttp && (!ws || ws.readyState !== 1)) connect();
  }
});

// Gain selector on Watch / Phone (1x Full Arm, 2.5x Desk Mode, 4x Micro)
if ($("gain-bar")) {
  $("gain-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
    deskGain = parseFloat(b.dataset.g) || 2.5;
    $("gain-bar").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
    touched = true;
  }));
}

function onMotion(e) {
  const a = e.accelerationIncludingGravity || e.acceleration;
  const r = e.rotationRate;
  const now = performance.now();
  const dt = fusedAtt.lastT ? Math.min(0.1, Math.max(0.002, (now - fusedAtt.lastT) / 1000)) : 0.02;
  fusedAtt.lastT = now;

  if (a && (a.x != null || a.y != null || a.z != null)) {
    pose.ax = +(a.x || 0).toFixed(3);
    pose.ay = +(a.y || 0).toFixed(3);
    pose.az = +(a.z || 0).toFixed(3);
    const mag = Math.hypot(pose.ax, pose.ay, pose.az);
    if (mag > 16.5) pose.tap = 1; // sharp tap fiducial like abel_sim2real compare.py
  }
  if (r && (r.alpha != null || r.beta != null || r.gamma != null)) {
    const d2r = Math.PI / 180;
    pose.gx = +((r.beta || 0) * d2r).toFixed(3);
    pose.gy = +((r.gamma || 0) * d2r).toFixed(3);
    pose.gz = +((r.alpha || 0) * d2r).toFixed(3);
  }

  // Fallback Attitude Integration from 6-Axis Accelerometer + Gyroscope when watchOS omits DeviceOrientationEvent:
  // If DeviceOrientation hasn't fired recently, compute pitch/roll from gravity vector + gyro integration and yaw from gyro Z
  if (!lastO || (now - (lastO.t || 0) > 250)) {
    const r2d = 180 / Math.PI;
    const accPitch = Math.atan2(-pose.ay, Math.hypot(pose.ax, pose.az)) * r2d;
    const accRoll = Math.atan2(pose.ax, pose.az) * r2d;
    fusedAtt.pitch = 0.92 * (fusedAtt.pitch + pose.gx * r2d * dt) + 0.08 * accPitch;
    fusedAtt.roll = 0.92 * (fusedAtt.roll + pose.gy * r2d * dt) + 0.08 * accRoll;
    fusedAtt.yaw = Math.max(-90, Math.min(90, fusedAtt.yaw + pose.gz * r2d * dt));
    if (!zero) zero = { a: fusedAtt.yaw, b: fusedAtt.pitch, g: fusedAtt.roll };
    const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
    pose.yaw = Math.round(clamp((fusedAtt.yaw - zero.a) * deskGain, -180, 180));
    pose.pitch = Math.round(clamp((fusedAtt.pitch - zero.b) * deskGain, -90, 90));
    pose.roll = Math.round(clamp((fusedAtt.roll - zero.g) * deskGain, -90, 90));
    tiltOn = true;
    readout();
  }

  // Push immediate pose + IMU packet on significant motion so watch never feels laggy or frozen between intervals
  if (now - (onMotion.lastSent || 0) >= 45) {
    onMotion.lastSent = now;
    const tMs = Date.now();
    send({ t: "pose", v: { vx: -jy, vy: -jx, vz, g: pose.g, aim, gain: deskGain, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, ax: pose.ax, ay: pose.ay, az: pose.az, gx: pose.gx, gy: pose.gy, gz: pose.gz, tap: pose.tap, utc_ms: tMs, utc_iso: new Date(tMs).toISOString() } });
    pose.tap = 0;
  }

  // Hands-free Wrist Clench / Double-Tap Impulse or Pronation Roll Flick -> Toggle Grip/Release
  const aDelta = Math.abs(Math.hypot(pose.ax, pose.ay, pose.az) - 9.80665);
  const gMag = Math.hypot(pose.gx, pose.gy, pose.gz);
  if (now - lastGestureAt > 850 && ((aDelta > 3.6 && gMag < 1.9) || Math.abs(pose.gx) > 2.5)) {
    lastGestureAt = now;
    touched = true;
    pose.g = pose.g ? 0 : 1;
    paintGrab();
    const gesture = Math.abs(pose.gx) > 2.5
      ? (pose.g ? "Wrist Roll Flick -> GRASP" : "Wrist Roll Flick -> RELEASE")
      : (pose.g ? "Clench Impulse -> GRASP" : "Tendon Impulse -> RELEASE");
    send({ t: "pose", v: { vx: -jy, vy: -jx, vz, g: pose.g, grabToggle: 1, gesture, aim, gain: deskGain, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, ax: pose.ax, ay: pose.ay, az: pose.az, gx: pose.gx, gy: pose.gy, gz: pose.gz, tap: 1 } });
    if (navigator.vibrate) navigator.vibrate([40, 30, 50]);
  }
}

function onOrient(e) {
  if (e.alpha == null && e.beta == null) return;
  lastO = { a: e.alpha || 0, b: e.beta || 0, g: e.gamma || 0, t: performance.now() };
  if (!zero) zero = { ...lastO };
  // Apply Desk Motion Gain (default 2.5x) so small comfortable wrist rotations right in front of your screen cover the full workspace
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  let yaw = lastO.a - zero.a; while (yaw > 180) yaw -= 360; while (yaw < -180) yaw += 360;
  pose.yaw = Math.round(clamp(yaw * deskGain, -180, 180));
  pose.pitch = Math.round(clamp((lastO.b - zero.b) * deskGain, -90, 90));
  pose.roll = Math.round(clamp((lastO.g - zero.g) * deskGain, -90, 90));
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
  zero = lastO ? { ...lastO } : { a: fusedAtt.yaw, b: fusedAtt.pitch, g: fusedAtt.roll };
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
