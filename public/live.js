// Deck-side connection to the audience WebSocket relay (Cloud Run server.mjs or Worker).
// Works when served by server.mjs (npm run dev or the deployed Cloud Run URL).
// On a plain static server it reports "offline" and slides fall back to on-screen controls.
import { qrcode } from "./vendor/qrcode.mjs";

const listeners = new Set();
let ws = null, status = "connecting", phones = [], retry = 0, lastState = null, checked = null;

// The room is pinned so the QR code never changes during the talk. ?room= overrides (rehearsals, tests).
function roomCode() {
  const fromUrl = new URLSearchParams(location.search).get("room");
  return fromUrl && /^[A-Z0-9]{3,8}$/i.test(fromUrl) ? fromUrl.toUpperCase() : "OPERON";
}
// Presenter key: only a deck opened with ?k=<key> may drive the room.
function presenterKey() {
  const k = new URLSearchParams(location.search).get("k");
  // remembered on this laptop, so the presenter only opens the special link once
  try { if (k) localStorage.setItem("df26.k", k); return k || localStorage.getItem("df26.k") || ""; } catch { return k || ""; }
}
const KEY = presenterKey();
export const ROOM = roomCode();

export function phoneUrl() {
  const base = location.origin + location.pathname.replace(/(?:\/teleop-studio|\/telemetry)?\/?[^/]*$/, "/");
  return `${base}phone.html?room=${ROOM}`;
}

export function qrSVG(text = phoneUrl(), cell = 6) {
  const q = qrcode(0, "M"); q.addData(text); q.make();
  return q.createSvgTag({ cellSize: cell, margin: 2, scalable: true });
}

function emit(ev) { listeners.forEach(fn => { try { fn(ev); } catch (e) { console.error(e); } }); }

export function onLive(fn) { listeners.add(fn); fn({ t: "status", status, phones }); return () => listeners.delete(fn); }
export function liveStatus() { return { status, phones: phones.slice(), room: ROOM }; }

export function send(msg) {
  if (msg.t === "state") lastState = msg;
  if (ws && ws.readyState === 1) ws.send(JSON.stringify(msg));
}

async function hasRelay() {
  if (checked !== null) return checked;
  if (location.protocol === "file:") return (checked = false);
  try {
    const r = await fetch("/api/health", { cache: "no-store" });
    checked = r.ok && (await r.json()).relay === true;
  } catch { checked = false; }
  return checked;
}

async function connect() {
  if (!(await hasRelay())) { status = "offline"; emit({ t: "status", status, phones }); return; }
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/api/room/${ROOM}/ws?role=deck&k=${encodeURIComponent(KEY)}&id=deck-${Math.random().toString(36).slice(2, 8)}`);
  let opened = false;
  ws.onopen = () => {
    opened = true; status = "live"; retry = 0; emit({ t: "status", status, phones });
    if (lastState) ws.send(JSON.stringify(lastState));
  };
  ws.onmessage = (e) => {
    let m; try { m = JSON.parse(e.data); } catch { return; }
    if (m.t === "presence") { phones = m.phones || []; emit({ t: "status", status, phones }); return; }
    if (m.t === "join" && lastState && ws && ws.readyState === 1) {
      ws.send(JSON.stringify(lastState));
    }
    emit(m);
  };
  ws.onclose = (e) => {
    if (e.code === 1008 || (!opened && retry > 2)) { status = "nokey"; emit({ t: "status", status, phones }); return; }
    status = "reconnecting"; emit({ t: "status", status, phones });
    setTimeout(connect, Math.min(8000, 500 * 2 ** retry++));
  };
}
connect();
setInterval(() => { if (ws && ws.readyState === 1) ws.send('{"t":"ping"}'); }, 25000);

// Small reusable "join from your phone" panel.
export function joinPanelHTML(L, compact = false) {
  return `
    <div class="qr-box" data-live="qr" style="${compact ? "width:150px" : "width:200px"}"></div>
    <div class="col" style="gap:4px;min-width:0">
      <span class="label-sm">${L("Join from your phone", "Únete desde tu teléfono")}</span>
      <a class="mono" data-live="url" target="_blank" rel="noopener" style="font-size:13px;word-break:break-all;color:var(--ink);text-decoration:underline"></a>
      <span class="pill" data-live="status" style="align-self:flex-start">…</span><span class="mono" style="font-size:14px">${L("room", "sala")} <b>${ROOM}</b></span>
    </div>`;
}

export function wireJoinPanel(root, L) {
  const qr = root.querySelector('[data-live="qr"]');
  const url = root.querySelector('[data-live="url"]');
  const pill = root.querySelector('[data-live="status"]');
  if (qr) qr.innerHTML = qrSVG();
  if (url) {
    url.href = phoneUrl();
    url.textContent = phoneUrl().replace(/^https?:\/\//, "");
  }
  return onLive((ev) => {
    if (ev.t !== "status" || !pill) return;
    const n = ev.phones.length;
    pill.className = `pill ${ev.status === "live" ? "ok" : ev.status === "offline" ? "idle" : "warn"}`;
    pill.textContent = ev.status === "live" ? `${n} ${L(n === 1 ? "phone" : "phones", n === 1 ? "teléfono" : "teléfonos")}`
      : ev.status === "offline" ? L("relay offline · local controls", "relay apagado · controles locales")
      : ev.status === "nokey" ? L("view only · open your presenter link", "solo vista · abre tu enlace de presentador") : L("connecting…", "conectando…");
  });
}
