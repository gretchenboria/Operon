// Cloudflare Worker: serves the deck (static assets) and relays audience phones
// to the presenter's deck through one Durable Object per room code.
//
//   phone  ──ws──►  Room DO  ──ws──►  deck      (teleop commands, quiz votes)
//   deck   ──ws──►  Room DO  ──ws──►  phones    (current slide state, per-phone assignments)

const ROOM_RE = /^[A-Z0-9]{3,8}$/;

export default {
  async fetch(req, env) {
    const url = new URL(req.url);
    if (url.pathname === "/api/health") {
      return Response.json({ ok: true, relay: true });
    }
    const m = url.pathname.match(/^\/api\/room\/([^/]+)\/ws$/);
    if (m) {
      const code = m[1].toUpperCase();
      if (!ROOM_RE.test(code)) return new Response("bad room", { status: 400 });
      if (req.headers.get("Upgrade") !== "websocket") return new Response("expected websocket", { status: 426 });
      const stub = env.ROOMS.get(env.ROOMS.idFromName(code));
      return stub.fetch(req);
    }
    if (url.pathname === "/teleop-studio" || url.pathname === "/teleop-studio/" || url.pathname === "/telemetry" || url.pathname === "/telemetry/") {
      const rewritten = new URL("/teleop-studio.html", req.url);
      return env.ASSETS.fetch(new Request(rewritten, req));
    }
    return env.ASSETS.fetch(req);
  }
};

const MAX_PHONES = 400;
const MAX_MSG = 2048;

export class Room {
  constructor(ctx, env) {
    this.ctx = ctx;
    this.env = env;
  }

  async fetch(req) {
    const url = new URL(req.url);
    const role = url.searchParams.get("role") === "deck" ? "deck" : "phone";
    // Only the presenter's deck (with the key) may drive the room. Phones need no key.
    if (role === "deck" && this.env.PRESENTER_TOKEN && url.searchParams.get("k") !== this.env.PRESENTER_TOKEN) {
      return new Response("presenter key required", { status: 403 });
    }
    const id = (url.searchParams.get("id") || crypto.randomUUID()).replace(/[^a-zA-Z0-9-]/g, "").slice(0, 40) || crypto.randomUUID();
    if (role === "phone" && this.ctx.getWebSockets("phone").length >= MAX_PHONES) {
      return new Response("room full", { status: 503 });
    }
    const pair = new WebSocketPair();
    const [client, server] = Object.values(pair);
    this.ctx.acceptWebSocket(server, [role, `id:${id}`]);
    server.serializeAttachment({ role, id });
    if (role === "deck") {
      // Newest deck gets the latest state cache; tell it who is here.
      server.send(JSON.stringify({ t: "presence", phones: this.phoneIds() }));
    } else {
      const state = await this.ctx.storage.get("state");
      if (state) server.send(JSON.stringify(state));
      this.toDecks({ t: "join", from: id });
      this.toDecks({ t: "presence", phones: this.phoneIds() });
    }
    return new Response(null, { status: 101, webSocket: client });
  }

  phoneIds() {
    return [...new Set(this.ctx.getWebSockets("phone").filter(ws => ws.readyState === WebSocket.OPEN).map(ws => ws.deserializeAttachment()?.id).filter(Boolean))];
  }

  toDecks(msg) {
    const s = JSON.stringify(msg);
    for (const ws of this.ctx.getWebSockets("deck")) { try { ws.send(s); } catch {} }
  }

  async webSocketMessage(ws, raw) {
    if (typeof raw !== "string" || raw.length > MAX_MSG) return;
    let msg;
    try { msg = JSON.parse(raw); } catch { return; }
    const me = ws.deserializeAttachment() || {};
    if (msg.t === "ping") { ws.send('{"t":"pong"}'); return; }

    if (me.role === "deck") {
      if (msg.to) {
        for (const p of this.ctx.getWebSockets(`id:${msg.to}`)) { try { p.send(raw); } catch {} }
        return;
      }
      if (msg.t === "state") await this.ctx.storage.put("state", msg);
      for (const p of this.ctx.getWebSockets("phone")) { try { p.send(raw); } catch {} }
      return;
    }
    // phone -> decks; the server stamps the sender id so phones can't spoof each other
    msg.from = me.id;
    this.toDecks(msg);
  }

  async webSocketClose(ws) {
    const me = ws.deserializeAttachment() || {};
    try { ws.close(); } catch {}
    const others = (tag) => this.ctx.getWebSockets(tag).filter(s => s !== ws && s.readyState !== WebSocket.CLOSED && s.readyState !== WebSocket.CLOSING);
    if (me.role === "phone") {
      // A phone that reconnected keeps its id: only report it gone when its last socket closes.
      if (!others(`id:${me.id}`).length) this.toDecks({ t: "leave", from: me.id });
      this.toDecks({ t: "presence", phones: [...new Set(others("phone").map(s => s.deserializeAttachment()?.id).filter(Boolean))] });
    } else if (me.role === "deck" && !others("deck").length) {
      // Presenter gone: don't leave phones (or late joiners) on a stale slide.
      const idle = { t: "state", slide: "idle", title: "Waiting for the presenter…" };
      await this.ctx.storage.put("state", idle);
      const s = JSON.stringify(idle);
      for (const p of others("phone")) { try { p.send(s); } catch {} }
    }
  }

  async webSocketError(ws) { await this.webSocketClose(ws); }
}
