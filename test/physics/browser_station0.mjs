import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
// Drive the real studio page in Chrome over DevTools Protocol:
// boot physics, run the scripted pick & place, and read the physics panel.
import WebSocket from "ws";
import { spawn } from "node:child_process";
import { rmSync } from "node:fs";
import fs from "node:fs";

const PORT = process.env.APP_PORT || "8841";
const CDP = 9347;
const URL = (process.env.OPERON_URL || `http://127.0.0.1:${PORT}`) + "/teleop-studio.html";
const CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";

const PROF = "/tmp/cr-prof-" + process.pid;
const chrome = spawn(CHROME, [
  "--headless=new", "--disable-gpu", "--no-sandbox", "--no-first-run",
  "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
  `--remote-debugging-port=${CDP}`, "--remote-allow-origins=*",
  "--window-size=1600,1000", `--user-data-dir=${PROF}`,
  "about:blank",
], { stdio: ["ignore", "ignore", "pipe"] });

let chromeErr = "";
chrome.stderr.on("data", (d) => { chromeErr += d.toString(); });
const cleanup = () => { try { chrome.kill("SIGKILL"); } catch {} try { rmSync(PROF, { recursive: true, force: true }); } catch {} };
process.on("exit", cleanup);

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// wait for the CDP endpoint
let target = null;
for (let i = 0; i < 60; i++) {
  try {
    const r = await fetch(`http://127.0.0.1:${CDP}/json/list`);
    const list = await r.json();
    target = list.find((t) => t.type === "page");
    if (target?.webSocketDebuggerUrl) break;
  } catch {}
  await sleep(250);
}
if (!target) { console.error("could not reach CDP:", chromeErr.slice(-500)); process.exit(1); }

const ws = new WebSocket(target.webSocketDebuggerUrl, { perMessageDeflate: false });
await new Promise((res, rej) => { ws.once("open", res); ws.once("error", rej); });

let msgId = 0;
const pending = new Map();
const consoleMsgs = [];
const pageErrors = [];
ws.on("message", (raw) => {
  const m = JSON.parse(raw.toString());
  if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); return; }
  if (m.method === "Runtime.consoleAPICalled") {
    const txt = (m.params.args || []).map((a) => a.value ?? a.description ?? "").join(" ");
    consoleMsgs.push(`${m.params.type}: ${txt}`);
  }
  if (m.method === "Runtime.exceptionThrown") {
    const d = m.params.exceptionDetails;
    pageErrors.push(d.exception?.description || d.text || JSON.stringify(d));
  }
  if (m.method === "Log.entryAdded" && m.params.entry.level === "error") {
    pageErrors.push(`${m.params.entry.source}: ${m.params.entry.text}`);
  }
});
const send = (method, params = {}) => new Promise((res) => {
  const id = ++msgId;
  pending.set(id, (m) => res(m.result ?? m.error));
  ws.send(JSON.stringify({ id, method, params }));
});
const evaluate = async (expr) => {
  const r = await send("Runtime.evaluate", { expression: expr, returnByValue: true, awaitPromise: true });
  if (r?.exceptionDetails) return { __err: r.exceptionDetails.exception?.description || r.exceptionDetails.text };
  return r?.result?.value;
};

await send("Runtime.enable");
await send("Log.enable");
await send("Page.enable");
await send("Page.navigate", { url: URL });

let fail = 0;
const ok = (c, m) => { console.log(`${c ? "  PASS" : "  FAIL"}  ${m}`); if (!c) fail++; };

// ── wait for the physics badge to leave "booting" ──
let badge = "";
for (let i = 0; i < 100; i++) {
  await sleep(400);
  badge = await evaluate(`document.getElementById("phys-badge")?.textContent || ""`);
  if (badge && !/booting/i.test(badge)) break;
}
console.log(`\nphysics badge: "${badge}"`);
ok(/MuJoCo/.test(badge), `Contact Physics badge reports MuJoCo (got "${badge}")`);

const readPanel = () => evaluate(`(() => {
  const g = (id) => document.getElementById(id)?.textContent || "";
  return { gap: g("phys-gap"), grip: g("phys-grip"), pads: g("phys-pads"), ncon: g("phys-ncon"), cost: g("phys-cost") };
})()`);

const idle = await readPanel();
console.log(`idle:     gap=${idle.gap} grip=${idle.grip} pads=${idle.pads} ncon=${idle.ncon} cost=${idle.cost}`);
ok(/mm/.test(idle.gap), "jaw gap is being reported live from the solver");
ok(/ms/.test(idle.cost), "solver cost/substeps reported");
const idleGap = parseFloat(idle.gap);
ok(idleGap > 40, `jaws idle wide open (pad gap ${idleGap.toFixed(1)} mm)`);

// ── run the scripted physical pick & place ──
const clicked = await evaluate(`(() => {
  const b = document.getElementById("btn-pick-place");
  if (!b) return "no button";
  b.click();
  return "clicked";
})()`);
console.log(`\nbtn-pick-place: ${clicked}`);
ok(clicked === "clicked", "scripted pick & place triggered");

// sample through the cycle
const samples = [];
for (let i = 0; i < 24; i++) {
  await sleep(350);
  const s = await readPanel();
  samples.push({ t: ((i + 1) * 0.35).toFixed(2), ...s });
}
console.log("\n  t(s)   jaw gap    grip force          pad load L/R   ncon");
for (const s of samples) {
  console.log(`  ${String(s.t).padStart(5)}  ${s.gap.padStart(9)}  ${s.grip.padStart(18)}  ${s.pads.padStart(14)}  ${s.ncon.padStart(4)}`);
}
const gaps = samples.map((s) => parseFloat(s.gap)).filter(Number.isFinite);
const grips = samples.map((s) => parseFloat(s.grip)).filter(Number.isFinite);
const minGap = Math.min(...gaps), maxGrip = Math.max(...grips);
const everHeld = samples.some((s) => /holding/.test(s.grip));
console.log(`\nminimum jaw gap ${minGap.toFixed(1)} mm, peak grip force ${maxGrip.toFixed(1)} N, reported holding: ${everHeld}`);
ok(minGap < idleGap - 15, "jaws closed substantially during the cycle");
ok(minGap > 20 && minGap < 27, `jaws stopped at the 24 mm cell's own width (${minGap.toFixed(1)} mm pad gap)`);
ok(minGap > 8, "jaws did not close flat (pads can reach a 3.5 mm gap on nothing)");
ok(maxGrip > 1, "a real grip force developed");
ok(everHeld, "panel reported 'holding' from pad load during the carry");

// ── screenshot ──
const shot = await send("Page.captureScreenshot", { format: "png" });
if (shot?.data) {
  fs.writeFileSync("/tmp/operon_physics.png", Buffer.from(shot.data, "base64"));
  console.log(`\nscreenshot written: /tmp/operon_physics.png (${(Buffer.from(shot.data, "base64").length / 1024).toFixed(0)} KB)`);
}

// ── errors ──
const realErrors = pageErrors.filter((e) => !/favicon|ReadPixels|GL Driver/i.test(e));
console.log(`\nconsole messages mentioning operon/mujoco:`);
for (const m of consoleMsgs.filter((m) => /operon|mujoco/i.test(m))) console.log(`  ${m}`);
console.log(`page errors: ${realErrors.length}`);
for (const e of realErrors.slice(0, 10)) console.log(`  ! ${e.split("\n")[0]}`);
ok(realErrors.length === 0, "no uncaught page errors");

console.log(`\n${fail === 0 ? "ALL BROWSER CHECKS PASSED" : `${fail} CHECK(S) FAILED`}`);
ws.close();
cleanup();
process.exit(fail === 0 ? 0 : 1);
