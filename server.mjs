// Cloud Run / Node.js server: serves the deck (static assets in ./public) and
// relays audience phones to the presenter's deck per room code over WebSockets.
//
//   phone  ──ws──►  Room  ──ws──►  deck      (teleop commands, quiz votes)
//   deck   ──ws──►  Room  ──ws──►  phones    (current slide state, per-phone assignments)

import http from "node:http";
import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { execFile } from "node:child_process";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC_DIR = path.join(__dirname, "public");
const PORT = parseInt(process.env.PORT || "8787", 10);
const PRESENTER_TOKEN = (process.env.PRESENTER_TOKEN || "").trim();
const GCP_PROJECT = process.env.GCP_PROJECT || "gen-lang-client-0221638641";

let cachedToken = { token: "", exp: 0 };
async function getVertexToken() {
  if (cachedToken.token && Date.now() < cachedToken.exp) return cachedToken.token;
  try {
    const r = await fetch("http://metadata.google.internal/computeMetadata/v1/instance/service-accounts/default/token", {
      headers: { "Metadata-Flavor": "Google" },
      signal: AbortSignal.timeout(1500)
    });
    if (r.ok) {
      const d = await r.json();
      cachedToken = { token: d.access_token, exp: Date.now() + Math.max(60, (d.expires_in || 1800) - 60) * 1000 };
      return cachedToken.token;
    }
  } catch {}
  return new Promise((resolve) => {
    execFile("gcloud", ["auth", "print-access-token"], { timeout: 4000 }, (err, stdout) => {
      const tok = (stdout || "").trim();
      if (!err && tok) {
        cachedToken = { token: tok, exp: Date.now() + 40 * 60 * 1000 };
        resolve(tok);
      } else resolve("");
    });
  });
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
  ".woff": "font/woff",
  ".woff2": "font/woff2"
};

class Room {
  constructor(code) {
    this.code = code;
    this.sockets = new Set(); // each ws has ws.meta = { role, id }
    this.state = null;
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
    return [...new Set(this.getWebSockets("phone").map(ws => ws.meta.id).filter(Boolean))];
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
      try { ws.send(JSON.stringify({ t: "presence", phones: this.phoneIds() })); } catch {}
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
    if (typeof raw !== "string" || raw.length > MAX_MSG) return;
    let msg;
    try { msg = JSON.parse(raw); } catch { return; }
    const me = ws.meta || {};
    if (msg.t === "ping") {
      try { ws.send('{"t":"pong"}'); } catch {}
      return;
    }

    if (me.role === "deck") {
      if (msg.to) {
        for (const p of this.getWebSockets(`id:${msg.to}`)) {
          try { p.send(raw); } catch {}
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

    if (me.role === "phone") {
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
    const body = JSON.stringify({ ok: true, relay: true, vertex: true, model: "gemini-3.8-flash" });
    res.writeHead(200, {
      "Content-Type": "application/json; charset=utf-8",
      "Cache-Control": "no-store"
    });
    res.end(body);
    return;
  }

  if (url.pathname === "/api/gemini" && req.method === "POST") {
    let raw = "";
    req.setEncoding("utf8");
    req.on("data", chunk => { if (raw.length < 4 * 1024 * 1024) raw += chunk; });
    req.on("end", async () => {
      try {
        const payload = JSON.parse(raw || "{}");
        const requestedModel = /^[a-z0-9.-]+$/i.test(payload.model || "") ? payload.model : "gemini-3.8-flash";
        const vertexModel = requestedModel === "gemini-3.8-flash" ? "gemini-3-flash-preview" : requestedModel;
        const tok = await getVertexToken();
        if (!tok) {
          res.writeHead(503, { "Content-Type": "application/json" });
          res.end(JSON.stringify({ error: { message: "No Vertex AI credentials available" } }));
          return;
        }
        const contents = (payload.contents || []).map(c => ({ role: c.role || "user", parts: c.parts || [] }));
        const vBody = JSON.stringify({ contents, generationConfig: payload.generationConfig || { maxOutputTokens: 1200, temperature: 0.4 } });
        const callVertex = (m) => fetch(`https://aiplatform.googleapis.com/v1/projects/${GCP_PROJECT}/locations/global/publishers/google/models/${encodeURIComponent(m)}:generateContent`, {
          method: "POST",
          headers: { "Authorization": `Bearer ${tok}`, "Content-Type": "application/json" },
          body: vBody,
          signal: AbortSignal.timeout(22000)
        });
        let vr = await callVertex(vertexModel);
        if (vr.status === 404 && vertexModel !== "gemini-3-flash-preview") {
          vr = await callVertex("gemini-3-flash-preview");
        }
        const data = await vr.json().catch(() => ({}));
        data._model = requestedModel;
        res.writeHead(vr.status, { "Content-Type": "application/json; charset=utf-8", "Cache-Control": "no-store" });
        res.end(JSON.stringify(data));
      } catch (e) {
        res.writeHead(500, { "Content-Type": "application/json" });
        res.end(JSON.stringify({ error: { message: e.message } }));
      }
    });
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

  const role = url.searchParams.get("role") === "deck" ? "deck" : "phone";

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
