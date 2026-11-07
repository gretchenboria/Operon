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
  setStatus("ok", `⌚ ${ROOM}`);
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
        setStatus("ok", `⌚ ${ROOM}`);
        if (Array.isArray(d.msgs)) d.msgs.forEach(handle);
      } else {
        setStatus("warn", T("syncing…", "sincronizando…"));
      }
    } catch {
      setStatus("warn", T("retrying…", "reintentando…"));
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
let station = { n: 1, name: T("🔋 Pick the battery", "🔋 Toma la batería") };
const DEFAULT_QUIZ = {
  q: 0,
  text: T("Someone shouts “stop!” while the arm is moving.", "Alguien grita “¡alto!” mientras el brazo se mueve."),
  revealed: false,
  answer: null,
  options: [
    { name: T("Reflex", "Reflejo"), desc: T("deterministic code", "código determinista") },
    { name: T("Cloud Gemini", "Gemini en la nube"), desc: T("Layer 2", "Capa 2") },
    { name: T("On-device model", "Modelo local"), desc: T("YOLO · Gemma · fleet memory", "YOLO · Gemma · memoria") },
    { name: T("Escalate + safe stop", "Escalar + alto seguro"), desc: T("frontier model, never cached", "modelo grande, sin caché") },
    { name: T("Refuse · fail closed", "Negar · falla cerrada"), desc: T("no bytes to the motors", "cero bytes a los motores") }
  ]
};
let lastQuizMsg = DEFAULT_QUIZ;
if ($("hint")) $("hint").textContent = T("🎯 Drag joystick to steer · tap SNAP or GRAB", "🎯 Arrastra el joystick · toca ALINEAR o AGARRAR");
if ($("tab-teleop")) $("tab-teleop").textContent = T("🦾 Robot Teleop", "🦾 Teleop Robot");
if ($("tab-quiz")) $("tab-quiz").textContent = T("🗳️ Live Quiz", "🗳️ Quiz en Vivo");
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
  $("pilot-who").textContent = slide === "teleop" && pilotId ? `🎮 ${pilotId === myId ? T("you", "tú") : pilotName}` : "";
  $("pilot-title").textContent = T("You're driving!", "¡Tú manejas!");
  $("pilot-sub").textContent = T("Your phone alone controls both arms now. Everyone's watching!", "Solo tu teléfono controla ambos brazos. ¡Todos te miran!");
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

// ---------- pose: tilt + pad + live IMU (for Sim-vs-Watch/Phone IMU comparison) ----------
const pose = { yaw: 0, pitch: 0, roll: 0, g: 0, ax: 0, ay: 0, az: 9.81, gx: 0, gy: 0, gz: 0, tap: 0 };
let zero = null, tiltOn = false, manualOn = false, lastO = null, touched = false;

function onMotion(e) {
  const a = e.accelerationIncludingGravity || e.acceleration;
  const r = e.rotationRate;
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
}

function onOrient(e) {
  if (e.alpha == null && e.beta == null) return;
  lastO = { a: e.alpha || 0, b: e.beta || 0, g: e.gamma || 0 };
  if (!zero) zero = { ...lastO };
  let yaw = -(lastO.a - zero.a); while (yaw > 180) yaw -= 360; while (yaw < -180) yaw += 360;
  pose.yaw = Math.round(yaw);
  pose.pitch = Math.round(Math.max(-90, Math.min(90, -(lastO.b - zero.b))));
  pose.roll = Math.round(Math.max(-90, Math.min(90, lastO.g - zero.g)));
  readout();
}
function readout() {
  $("r-yaw").textContent = pose.yaw + "°"; $("r-pitch").textContent = pose.pitch + "°"; $("r-roll").textContent = pose.roll + "°";
}
$("enable-tilt").addEventListener("click", async () => {
  try {
    if (typeof DeviceOrientationEvent !== "undefined" && typeof DeviceOrientationEvent.requestPermission === "function") {
      const r = await DeviceOrientationEvent.requestPermission();
      if (r !== "granted") throw new Error("denied");
    }
    if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
      try { await DeviceMotionEvent.requestPermission(); } catch {}
    }
    if (!window.isSecureContext) throw new Error("needs https");
    window.addEventListener("deviceorientation", onOrient);
    window.addEventListener("devicemotion", onMotion);
    tiltOn = true; zero = null;
    $("enable-tilt").textContent = T("Tilt on ✓", "Inclinación activa ✓");
    $("enable-tilt").classList.add("light");
    setTimeout(() => { if (!lastO) { $("enable-tilt").textContent = T("No tilt sensor: use sliders", "Sin sensor: usa deslizadores"); openManual(); } }, 1500);
  } catch (err) {
    $("enable-tilt").textContent = T(`Tilt unavailable (${err.message}): use sliders`, `Sin inclinación (${err.message}): usa deslizadores`);
    openManual();
  }
});
$("zero").addEventListener("click", () => { zero = lastO ? { ...lastO } : null; pose.yaw = pose.pitch = pose.roll = 0; readout(); });
function openManual() { manualOn = true; $("manual-box").classList.remove("hide"); }
$("manual").addEventListener("click", () => { if (manualOn) { manualOn = false; $("manual-box").classList.add("hide"); } else openManual(); });
for (const k of ["yaw", "pitch", "roll"]) $("m-" + k).addEventListener("input", (e) => { pose[k] = +e.target.value; $("mv-" + k).textContent = pose[k] + "°"; readout(); });

function paintGrab() {
  $("grab").textContent = pose.g ? T("✋ RELEASE", "✋ SOLTAR") : T("✊ GRAB", "✊ AGARRAR");
  $("grab").style.background = pose.g ? "var(--pr)" : "var(--pg)";
}
$("grab").addEventListener("click", () => {
  touched = true;
  pose.g = pose.g ? 0 : 1;
  paintGrab();
  send({ t: "pose", v: { vx: -jy, vy: jx, vz, g: pose.g, grabToggle: 1, aim, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll } });
  if (navigator.vibrate) navigator.vibrate(30);
});
if ($("snap")) {
  $("snap").textContent = T("🎯 SNAP", "🎯 ALINEAR");
  $("snap").addEventListener("click", () => {
    touched = true;
    send({ t: "pose", v: { vx: 0, vy: 0, vz: 0, g: pose.g, aim, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, snap: 1 } });
    if (navigator.vibrate) navigator.vibrate([30, 30, 50]);
  });
}
paintGrab();

// Joystick (springs back to center) = hand velocity in the top view; ▲/▼ = height velocity.
const stick = $("stick"), knob = $("knob");
let jx = 0, jy = 0, vz = 0, aim = "auto";
function stickMove(e) {
  const r = stick.getBoundingClientRect(), cx = r.left + r.width / 2, cy = r.top + r.height / 2, R = r.width * 0.36;
  let dx = (e.clientX - cx) / R, dy = (e.clientY - cy) / R; const m = Math.hypot(dx, dy); if (m > 1) { dx /= m; dy /= m; }
  jx = dx; jy = dy; knob.style.left = `${33 + dx * 33}%`; knob.style.top = `${33 + dy * 33}%`;
}
const stickEnd = () => { jx = jy = 0; knob.style.left = knob.style.top = "33%"; };
stick.addEventListener("pointerdown", (e) => { touched = true; stick.setPointerCapture(e.pointerId); stickMove(e); });
stick.addEventListener("pointermove", (e) => { if (e.buttons || e.pointerType === "touch") stickMove(e); });
stick.addEventListener("pointerup", stickEnd); stick.addEventListener("pointercancel", stickEnd);
for (const [id, v] of [["up", 1], ["down", -1]]) {
  const b = $(id);
  b.addEventListener("pointerdown", (e) => { touched = true; b.setPointerCapture(e.pointerId); vz = v; });
  for (const ev of ["pointerup", "pointercancel", "pointerleave"]) b.addEventListener(ev, () => { vz = 0; });
}
$("aim").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
  aim = b.dataset.v; $("aim").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  $("tilt-box").classList.toggle("hide", aim !== "tilt"); touched = true;
}));
setInterval(() => {
  if (slide === "teleop" && mode === "pose" && (tiltOn || manualOn || touched)) {
    send({ t: "pose", v: { vx: -jy, vy: jx, vz, g: pose.g, aim, yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, ax: pose.ax, ay: pose.ay, az: pose.az, gx: pose.gx, gy: pose.gy, gz: pose.gz, tap: pose.tap } });
    pose.tap = 0;
  }
}, 90);

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
  $("q-note").textContent = m.revealed ? (vote === null ? T("Answer revealed on the big screen.", "Respuesta revelada en la pantalla.") : vote === right ? T("You got it! 🎉", "¡Acertaste! 🎉") : T("Not this time. See why on the big screen.", "Esta vez no. Mira por qué en la pantalla.")) : vote === null ? T("Tap your answer.", "Toca tu respuesta.") : T("Vote sent.", "Voto enviado.");
}

render();
