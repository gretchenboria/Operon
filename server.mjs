// Cloud Run / Node.js server: serves the deck (static assets in ./public) and
// relays audience phones to the presenter's deck per room code over WebSockets.
//
//   phone  ──ws──►  Room  ──ws──►  deck      (teleop commands, quiz votes)
//   deck   ──ws──►  Room  ──ws──►  phones    (current slide state, per-phone assignments)

import http from "node:http";
import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC_DIR = path.join(__dirname, "public");
const TRAJ_DIR = path.join(__dirname, "config", "trajectories");
const PORT = parseInt(process.env.PORT || "8787", 10);

// Deterministic parser for config/trajectories/*.yaml (shared firmware contract)
function parseTrajectoryYaml(raw, fileStem = "trajectory") {
  const out = { name: fileStem, description: "", joint_order: ["base", "shoulder", "elbow", "gripper"], home: [90, 90, 90, 90], waypoints: [] };
  for (const line of raw.split(/\r?\n/)) {
    const clean = line.replace(/#.*$/, "").trim();
    if (!clean) continue;
    const mName = clean.match(/^name:\s*(.+)$/);
    if (mName) { out.name = mName[1].trim(); continue; }
    const mDesc = clean.match(/^description:\s*(.+)$/);
    if (mDesc) { out.description = mDesc[1].trim().replace(/^["']|["']$/g, ""); continue; }
    const mOrder = clean.match(/^joint_order:\s*\[([^\]]+)\]/);
    if (mOrder) { out.joint_order = mOrder[1].split(",").map(s => s.trim()); continue; }
    const mHome = clean.match(/^home:\s*\[([^\]]+)\]/);
    if (mHome) { out.home = mHome[1].split(",").map(Number); continue; }
    const mWp = clean.match(/^-\s*\{(.+)\}/);
    if (mWp) {
      const body = mWp[1];
      const mTgt = body.match(/targets:\s*\[([^\]]+)\]/);
      const mDur = body.match(/duration_ms:\s*([0-9.]+)/);
      const mSet = body.match(/settle_ms:\s*([0-9.]+)/);
      if (mTgt) {
        const commentMatch = line.match(/#\s*(.+)$/);
        out.waypoints.push({
          targets: mTgt[1].split(",").map(Number),
          duration_ms: mDur ? Number(mDur[1]) : 1000,
          settle_ms: mSet ? Number(mSet[1]) : 0,
          label: commentMatch ? commentMatch[1].trim() : ""
        });
      }
    }
  }
  return out;
}

const ROOM_RE = /^[A-Z0-9]{3,8}$/;
const MAX_PHONES = 400;
const MAX_MSG = 2048;

const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".webp": "image/webp",
  ".gif": "image/gif",
  ".ico": "image/x-icon",
  ".glb": "model/gltf-binary",
  ".stl": "model/stl",
  ".woff": "font/woff",
  ".woff2": "font/woff2"
};

class Room {
  constructor(code) {
    this.code = code;
    this.sockets = new Set(); // each ws has ws.meta = { role, id }
    this.state = null;
    this.httpPhones = new Map(); // id -> { lastSeen, queue: [] }
    this.lastGestureMs = 0;
    this.watchGrip = 0;
  }

  touchHttpPhone(id) {
    let p = this.httpPhones.get(id);
    const isNew = !p;
    if (!p) {
      p = { lastSeen: Date.now(), queue: [] };
      this.httpPhones.set(id, p);
    } else {
      p.lastSeen = Date.now();
    }
    return isNew;
  }

  pruneHttpPhones() {
    const now = Date.now();
    let changed = false;
    for (const [id, p] of this.httpPhones) {
      if (now - p.lastSeen > 12000) {
        this.httpPhones.delete(id);
        this.toDecks({ t: "leave", from: id });
        changed = true;
      }
    }
    if (changed) this.toDecks({ t: "presence", phones: this.phoneIds() });
  }

  getWebSockets(tag) {
    const out = [];
    for (const ws of this.sockets) {
      if (ws.readyState !== WebSocket.OPEN) continue;
      if (!tag || ws.meta.role === tag || `id:${ws.meta.id}` === tag) out.push(ws);
    }
    return out;
  }

  phoneIds() {
    this.pruneHttpPhones();
    const wsIds = this.getWebSockets("phone").map(ws => ws.meta.id).filter(Boolean);
    return [...new Set([...wsIds, ...this.httpPhones.keys()])];
  }

  toDecks(msg) {
    const s = JSON.stringify(msg);
    for (const ws of this.getWebSockets("deck")) {
      try { ws.send(s); } catch {}
    }
  }

  onConnect(ws, role, id) {
    ws.meta = { role, id };
    this.sockets.add(ws);

    if (role === "deck") {
      try {
        ws.send(JSON.stringify({ t: "presence", phones: this.phoneIds() }));
        ws.send(JSON.stringify({ t: "isaac_status", connected: this.getWebSockets("isaac").length > 0 }));
      } catch {}
    } else if (role === "isaac") {
      this.toDecks({ t: "isaac_status", connected: true, from: id });
    } else {
      if (this.state) {
        try { ws.send(JSON.stringify(this.state)); } catch {}
      }
      this.toDecks({ t: "join", from: id });
      this.toDecks({ t: "presence", phones: this.phoneIds() });
    }

    ws.on("message", (data, isBinary) => {
      if (isBinary) return;
      const raw = typeof data === "string" ? data : data.toString("utf8");
      this.onMessage(ws, raw);
    });

    ws.on("close", () => this.onClose(ws));
    ws.on("error", () => this.onClose(ws));
  }

  onMessage(ws, raw) {
    const me = ws.meta || {};
    const maxLen = me.role === "isaac" ? 262144 : me.role === "deck" ? 8192 : MAX_MSG;
    if (typeof raw !== "string" || raw.length > maxLen) return;
    let msg;
    try { msg = JSON.parse(raw); } catch { return; }
    if (msg.t === "ping") {
      try { ws.send('{"t":"pong"}'); } catch {}
      return;
    }

    if (me.role === "isaac") {
      msg.from = me.id;
      this.toDecks(msg);
      return;
    }

    if (me.role === "deck") {
      if (msg.t === "isaac_cmd") {
        for (const sim of this.getWebSockets("isaac")) {
          try { sim.send(raw); } catch {}
        }
        return;
      }
      if (msg.to) {
        for (const p of this.getWebSockets(`id:${msg.to}`)) {
          try { p.send(raw); } catch {}
        }
        const hp = this.httpPhones.get(msg.to);
        if (hp) {
          hp.queue.push(msg);
          if (hp.queue.length > 25) hp.queue.shift();
        }
        return;
      }
      if (msg.t === "state") {
        if (msg.slide === "idle" && msg.hidden && this.state && this.state.slide !== "idle" && this.getWebSockets("deck").length > 1) {
          return;
        }
        this.state = msg;
      }
      for (const p of this.getWebSockets("phone")) {
        try { p.send(raw); } catch {}
      }
      for (const hp of this.httpPhones.values()) {
        hp.queue.push(msg);
        if (hp.queue.length > 25) hp.queue.shift();
      }
      return;
    }

    // phone -> decks; server stamps sender id so phones can't spoof each other
    msg.from = me.id;
    this.toDecks(msg);
  }

  onClose(ws) {
    if (!this.sockets.has(ws)) return;
    this.sockets.delete(ws);
    const me = ws.meta || {};
    try { ws.close(); } catch {}

    const others = (tag) => [...this.sockets].filter(
      s => s !== ws && s.readyState !== WebSocket.CLOSED && s.readyState !== WebSocket.CLOSING &&
        (!tag || s.meta.role === tag || `id:${s.meta.id}` === tag)
    );

    if (me.role === "isaac") {
      this.toDecks({ t: "isaac_status", connected: others("isaac").length > 0 });
    } else if (me.role === "phone") {
      if (!others(`id:${me.id}`).length) this.toDecks({ t: "leave", from: me.id });
      this.toDecks({
        t: "presence",
        phones: [...new Set(others("phone").map(s => s.meta.id).filter(Boolean))]
      });
    } else if (me.role === "deck" && !others("deck").length) {
      const idle = { t: "state", slide: "idle", title: "Waiting for the presenter…" };
      this.state = idle;
      const s = JSON.stringify(idle);
      for (const p of others("phone")) {
        try { p.send(s); } catch {}
      }
    }
  }
}

const rooms = new Map();
function getRoom(code) {
  let r = rooms.get(code);
  if (!r) {
    r = new Room(code);
    rooms.set(code, r);
  }
  return r;
}

const wss = new WebSocketServer({ noServer: true });

const server = http.createServer((req, res) => {
  const url = new URL(req.url || "/", `http://${req.headers.host || "localhost"}`);

  if (url.pathname === "/api/health") {
    const nowMs = Date.now();
    const body = JSON.stringify({
      ok: true,
      relay: true,
      utc_iso: new Date(nowMs).toISOString(),
      epoch_utc_ms: nowMs
    });
    res.writeHead(200, {
      "Content-Type": "application/json; charset=utf-8",
      "Cache-Control": "no-store"
    });
    res.end(body);
    return;
  }

  if (url.pathname === "/api/trajectories" && req.method === "GET") {
    try {
      const files = fs.readdirSync(TRAJ_DIR).filter(f => f.endsWith(".yaml"));
      const trajectories = {};
      for (const f of files) {
        const stem = path.basename(f, ".yaml");
        const raw = fs.readFileSync(path.join(TRAJ_DIR, f), "utf8");
        trajectories[stem] = { ...parseTrajectoryYaml(raw, stem), raw_yaml: raw };
      }
      res.writeHead(200, { "Content-Type": "application/json; charset=utf-8", "Cache-Control": "no-store" });
      res.end(JSON.stringify({ ok: true, trajectories }));
    } catch (e) {
      res.writeHead(500, { "Content-Type": "application/json; charset=utf-8" });
      res.end(JSON.stringify({ ok: false, error: e.message }));
    }
    return;
  }

  const mTraj = url.pathname.match(/^\/api\/trajectories\/([a-zA-Z0-9_-]+)$/);
  if (mTraj && req.method === "GET") {
    const stem = mTraj[1];
    const p = path.join(TRAJ_DIR, `${stem}.yaml`);
    if (!fs.existsSync(p)) {
      res.writeHead(404, { "Content-Type": "application/json; charset=utf-8" });
      res.end(JSON.stringify({ ok: false, error: "Trajectory YAML not found" }));
      return;
    }
    const raw = fs.readFileSync(p, "utf8");
    res.writeHead(200, { "Content-Type": "application/json; charset=utf-8", "Cache-Control": "no-store" });
    res.end(JSON.stringify({ ok: true, trajectory: parseTrajectoryYaml(raw, stem), raw_yaml: raw }));
    return;
  }

  const m = url.pathname.match(/^\/api\/room\/([^/]+)\/ws$/);
  if (m) {
    const code = m[1].toUpperCase();
    if (!ROOM_RE.test(code)) {
      res.writeHead(400, { "Content-Type": "text/plain; charset=utf-8" });
      res.end("bad room");
      return;
    }
    res.writeHead(426, { "Content-Type": "text/plain; charset=utf-8" });
    res.end("expected websocket");
    return;
  }

  // HTTP polling & command fallback for Apple Watch WebKit (where watchOS blocks WebSockets)
  const mPoll = url.pathname.match(/^\/api\/room\/([^/]+)\/poll$/);
  if (mPoll && (req.method === "POST" || req.method === "OPTIONS")) {
    res.setHeader("Access-Control-Allow-Origin", "*");
    res.setHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type");
    if (req.method === "OPTIONS") { res.writeHead(204); res.end(); return; }
    const code = mPoll[1].toUpperCase();
    if (!ROOM_RE.test(code)) { res.writeHead(400); res.end("bad room"); return; }
    let raw = "";
    req.setEncoding("utf8");
    req.on("data", chunk => { if (raw.length < 128 * 1024) raw += chunk; });
    req.on("end", () => {
      try {
        const body = JSON.parse(raw || "{}");
        const id = String(body.id || "watch-1").replace(/[^a-zA-Z0-9-]/g, "").slice(0, 40) || "watch-1";
        const room = getRoom(code);
        const isNew = room.touchHttpPhone(id);
        const hp = room.httpPhones.get(id);
        if (isNew) {
          if (room.state) hp.queue.push(room.state);
          room.toDecks({ t: "join", from: id });
          room.toDecks({ t: "presence", phones: room.phoneIds() });
        }
        if (Array.isArray(body.msgs)) {
          for (const msg of body.msgs) {
            if (msg && typeof msg === "object" && msg.t !== "ping") {
              msg.from = id;
              room.toDecks(msg);
            }
          }
        }
        const out = hp ? hp.queue.splice(0, hp.queue.length) : [];
        res.writeHead(200, { "Content-Type": "application/json; charset=utf-8", "Cache-Control": "no-store" });
        res.end(JSON.stringify({ ok: true, msgs: out, state: room.state }));
      } catch {
        res.writeHead(400, { "Content-Type": "application/json" });
        res.end('{"ok":false}');
      }
    });
    return;
  }

  // HTTP POST ingest for Apple Watch Sensor Logger / SensorLog / watchOS Shortcuts live push
  const mImu = url.pathname.match(/^\/api\/room\/([^/]+)\/imu$/);
  if (mImu && (req.method === "POST" || req.method === "OPTIONS")) {
    res.setHeader("Access-Control-Allow-Origin", "*");
    res.setHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type");
    if (req.method === "OPTIONS") { res.writeHead(204); res.end(); return; }
    const code = mImu[1].toUpperCase();
    if (!ROOM_RE.test(code)) { res.writeHead(400); res.end("bad room"); return; }
    let raw = "";
    req.setEncoding("utf8");
    req.on("data", chunk => { if (raw.length < 512 * 1024) raw += chunk; });
    req.on("end", () => {
      try {
        const body = JSON.parse(raw || "{}");
        const room = getRoom(code);
        const wid = "watch-http";
        let ax = 0, ay = 0, az = 9.81, gx = 0, gy = 0, gz = 0;
        // Support Sensor Logger payload array (`payload: [{ name: "accelerometer"|"wristMotion", values: {...} }]`) or flat JSON
        if (Array.isArray(body.payload)) {
          for (const s of body.payload) {
            const v = s.values || {};
            if (s.name?.includes("accel") || s.name?.includes("motion")) {
              ax = +(v.x ?? v.accelerationX ?? ax);
              ay = +(v.y ?? v.accelerationY ?? ay);
              az = +(v.z ?? v.accelerationZ ?? az);
              if (Math.hypot(ax, ay, az) < 3.5) { ax *= 9.80665; ay *= 9.80665; az *= 9.80665; }
            }
            if (s.name?.includes("gyro") || s.name?.includes("rotation") || s.name?.includes("motion")) {
              gx = +(v.rotationRateX ?? v.x ?? gx);
              gy = +(v.rotationRateY ?? v.y ?? gy);
              gz = +(v.rotationRateZ ?? v.z ?? gz);
            }
          }
        } else {
          ax = +(body.ax ?? body.accelerometerAccelerationX ?? 0);
          ay = +(body.ay ?? body.accelerometerAccelerationY ?? 0);
          az = +(body.az ?? body.accelerometerAccelerationZ ?? 9.81);
          if (Math.hypot(ax, ay, az) < 3.5) { ax *= 9.80665; ay *= 9.80665; az *= 9.80665; }
          gx = +(body.gx ?? body.gyroRotationX ?? 0);
          gy = +(body.gy ?? body.gyroRotationY ?? 0);
          gz = +(body.gz ?? body.gyroRotationZ ?? 0);
        }
        const aMag = Math.hypot(ax, ay, az);
        const gMag = Math.hypot(gx, gy, gz);
        const tap = aMag > 16.5 ? 1 : 0;
        const nowMs = Date.now();
        let grabToggle = 0, gesture = "";
        // Detect Wrist Clench / Double-Tap Tendon Impulse (MMG |a - g| > 3.2 m/s² with low translation) or Pronation Flick (|gx| > 2.3 rad/s)
        if (nowMs - room.lastGestureMs > 850) {
          if (Math.abs(aMag - 9.80665) > 3.2 && gMag < 1.8) {
            room.lastGestureMs = nowMs;
            room.watchGrip = room.watchGrip ? 0 : 1;
            grabToggle = 1;
            gesture = room.watchGrip ? "Clench Impulse -> GRASP" : "Tendon Impulse -> RELEASE";
          } else if (Math.abs(gx) > 2.3) {
            room.lastGestureMs = nowMs;
            room.watchGrip = room.watchGrip ? 0 : 1;
            grabToggle = 1;
            gesture = room.watchGrip ? "Pronation Flick -> GRASP" : "Supination Flick -> RELEASE";
          }
        }
        room.touchHttpPhone(wid);
        const dtS = room.lastImuMs ? Math.min(0.08, Math.max(0.004, (nowMs - room.lastImuMs) / 1000)) : 0.02;
        room.lastImuMs = nowMs;
        const r2d = 180 / Math.PI;
        const isStill = gMag < 0.09 && Math.abs(aMag - 9.80665) < 0.45;
        const accElev = Math.atan2(ax, Math.max(1.5, Math.hypot(ay, az))) * r2d;
        const accTwist = Math.atan2(ay, Math.max(1.5, Math.hypot(ax, az))) * r2d;
        if (!isStill) {
          room.fusedPitch = 0.88 * ((room.fusedPitch ?? accElev) + gy * r2d * dtS) + 0.12 * accElev;
          room.fusedRoll = 0.88 * ((room.fusedRoll ?? accTwist) + gx * r2d * dtS) + 0.12 * accTwist;
          const gzDegS = Math.abs(gz * r2d) > 2.5 ? gz * r2d : 0;
          room.fusedYaw = Math.max(-75, Math.min(75, (room.fusedYaw || 0) * 0.995 + gzDegS * dtS));
        } else {
          room.fusedPitch = 0.96 * (room.fusedPitch ?? accElev) + 0.04 * accElev;
          room.fusedRoll = 0.96 * (room.fusedRoll ?? accTwist) + 0.04 * accTwist;
        }
        if (room.zeroPitch === undefined) { room.zeroPitch = room.fusedPitch; room.zeroRoll = room.fusedRoll; }
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const dz = (v, d = 1.8) => Math.abs(v) < d ? 0 : (v > 0 ? v - d : v + d);
        const dPitch = dz(room.fusedPitch - room.zeroPitch);
        const dRoll = dz(room.fusedRoll - room.zeroRoll);
        const yawDeg = Math.round(clamp((-dz(room.fusedYaw || 0) * 1.15 + dRoll * 0.65) * 3.2, -85, 85));
        const pitchDeg = Math.round(clamp(dPitch * 1.20 * 3.2, -75, 75));
        const rollDeg = Math.round(clamp(dRoll * 1.10 * 3.2, -85, 85));
        room.toDecks({ t: "pose", from: wid, v: { vx: 0, vy: 0, vz: 0, aim: "tilt", gain: 3.2, yaw: yawDeg, pitch: pitchDeg, roll: rollDeg, g: room.watchGrip, grabToggle, gesture, ax, ay, az, gx, gy, gz, tap } });
        res.writeHead(200, { "Content-Type": "application/json" });
        res.end(JSON.stringify({ ok: true, grip: room.watchGrip, gesture }));
      } catch {
        res.writeHead(400, { "Content-Type": "application/json" });
        res.end('{"ok":false}');
      }
    });
    return;
  }

  // Serve static files from ./public
  let relPath = decodeURIComponent(url.pathname);
  if (relPath === "/" || relPath === "") relPath = "/index.html";
  if (relPath === "/teleop-studio" || relPath === "/teleop-studio/" || relPath === "/telemetry" || relPath === "/telemetry/") {
    relPath = "/teleop-studio.html";
  }
  const filePath = path.resolve(PUBLIC_DIR, "." + relPath);
  if (!filePath.startsWith(PUBLIC_DIR + path.sep) && filePath !== PUBLIC_DIR) {
    res.writeHead(403);
    res.end("Forbidden");
    return;
  }

  fs.stat(filePath, (err, stat) => {
    if (err || !stat.isFile()) {
      res.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
      res.end("Not found");
      return;
    }
    const ext = path.extname(filePath).toLowerCase();
    const contentType = MIME[ext] || "application/octet-stream";
    res.writeHead(200, {
      "Content-Type": contentType,
      "Cache-Control": "no-cache"
    });
    fs.createReadStream(filePath).pipe(res);
  });
});

function rejectUpgrade(socket, status, text) {
  const msg = Buffer.from(text, "utf8");
  socket.write(
    `HTTP/1.1 ${status}\r\n` +
    `Content-Type: text/plain; charset=utf-8\r\n` +
    `Content-Length: ${msg.length}\r\n` +
    `Connection: close\r\n\r\n`
  );
  socket.write(msg);
  socket.destroy();
}

server.on("upgrade", (req, socket, head) => {
  const url = new URL(req.url || "/", `http://${req.headers.host || "localhost"}`);
  const m = url.pathname.match(/^\/api\/room\/([^/]+)\/ws$/);
  if (!m) {
    rejectUpgrade(socket, "404 Not Found", "not found");
    return;
  }
  const code = m[1].toUpperCase();
  if (!ROOM_RE.test(code)) {
    rejectUpgrade(socket, "400 Bad Request", "bad room");
    return;
  }

  const rawRole = url.searchParams.get("role");
  const role = rawRole === "deck" ? "deck" : rawRole === "isaac" ? "isaac" : "phone";

  const room = getRoom(code);
  if (role === "phone" && room.getWebSockets("phone").length >= MAX_PHONES) {
    rejectUpgrade(socket, "503 Service Unavailable", "room full");
    return;
  }

  const id = (url.searchParams.get("id") || crypto.randomUUID()).replace(/[^a-zA-Z0-9-]/g, "").slice(0, 40) || crypto.randomUUID();

  wss.handleUpgrade(req, socket, head, (ws) => {
    room.onConnect(ws, role, id);
  });
});

server.listen(PORT, "0.0.0.0", () => {
  console.log(`Operon Studio + Telemetry Relay listening on http://0.0.0.0:${PORT}`);
});
