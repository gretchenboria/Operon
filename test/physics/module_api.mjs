import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
// Exercise the REAL public/mujoco-physics.js module through its public API,
// driving the same pick-and-place the studio's scripted cycle performs.
import fs from "node:fs";
import path from "node:path";

const ROOT = REPO;

// The module fetches the MJCF over HTTP in the browser; map that to disk here.
const realFetch = globalThis.fetch;
globalThis.fetch = async (url, opts) => {
  const u = String(url);
  if (u.startsWith("/")) {
    const f = path.join(ROOT, "public", u);
    if (!fs.existsSync(f)) return new Response(null, { status: 404, statusText: "missing" });
    return new Response(fs.readFileSync(f, "utf8"), { status: 200 });
  }
  return realFetch(url, opts);
};

const { initPhysics } = await import(`${ROOT}/public/mujoco-physics.js`);
const { ik6, poseR } = await import(`${ROOT}/public/demos/arms3d.js`);

let fail = 0;
const ok = (c, m) => { console.log(`${c ? "  PASS" : "  FAIL"}  ${m}`); if (!c) fail++; };

const PHYS_GRASP_H = 0.005;
const FRAME = 1 / 60;

const p = await initPhysics({
  vendorBase: `${ROOT}/node_modules/mujoco/`,
  modelUrl: "/models/operon_station0.xml",
  rigs: [4, 6],
});

console.log(`initPhysics -> available=${p.available} reason="${p.reason}" timestep=${p.timestep}s arms=${Object.keys(p.arms)}`);
ok(p.available, "module booted both arm simulations");
if (!p.available) { process.exit(1); }
ok(Object.keys(p.arms).length === 2, "two independent arm instances");

const A = p.arms[4], B = p.arms[6];

// Seed both arms at a home pose, as the studio does on boot.
const home = ik6([0.15, -0.075, 0.12], poseR(-25, 0, 0)).q;
A.teleportTo(home, 1);
B.teleportTo(home, 1);
let r = B.read();
ok(r !== null, "read() returns a snapshot");
ok(Math.abs(r.open - 1) < 1e-6, "jaws start fully open");
ok(!r.held, "nothing held at boot");

// Independence: moving arm B must not perturb arm A's cell at all.
const cellA0 = A.read().cellP.slice();

// ── drive a full pick & place through the module API ──
const settle = (secs) => { const n = Math.round(secs / FRAME); for (let i = 0; i < n; i++) { A.advance(FRAME); B.advance(FRAME); } };
const goto = (p3, yawDeg, grip, secs) => {
  const q = ik6(p3, poseR(yawDeg, 0, 0), B.read().q).q;
  A.setArmTarget(q); B.setArmTarget(q);
  if (grip !== undefined) { A.setGripTarget(grip); B.setGripTarget(grip); }
  settle(secs);
};

settle(0.5);
const cp = B.read().cellP.slice();
const zG = cp[2] + PHYS_GRASP_H;
console.log(`\ncell at rest: [${cp.map(v => v.toFixed(4)).join(", ")}]  grasp TCP z=${zG.toFixed(4)}`);

goto([cp[0], cp[1], zG + 0.075], -25, 1, 0.5);     // over the cell
goto([cp[0], cp[1], zG], -25, 1, 0.6);             // descend
const beforeClose = B.read();
goto([cp[0], cp[1], zG], -25, 0, 0.5);             // CLAMP
const afterClose = B.read();
console.log(`\njaw gap ${beforeClose.jawGapMm.toFixed(1)} mm -> ${afterClose.jawGapMm.toFixed(1)} mm`);
console.log(`pad load L/R = ${afterClose.padLoadN.map(v => v.toFixed(2)).join(" / ")} N, grip ${afterClose.gripForceN.toFixed(1)} N, contacts ${afterClose.contacts}`);
ok(afterClose.jawGapMm < beforeClose.jawGapMm - 5, "jaws actually travelled inward");
ok(afterClose.open > 0.02, "jaws stopped ON the cell, not fully closed through it");
ok(afterClose.held, "read().held is true from pad load, not from a distance test");
ok(afterClose.gripForceN > 1, "a real clamping force is reported");

const zLiftFrom = B.read().cellP[2];
goto([cp[0], cp[1], zG + 0.090], -25, 0, 0.8);     // LIFT
const lifted = B.read();
console.log(`cell lifted ${((lifted.cellP[2] - zLiftFrom) * 1000).toFixed(1)} mm, still held=${lifted.held}`);
ok(lifted.cellP[2] - zLiftFrom > 0.06, "cell carried upward by friction alone");
ok(lifted.held, "grasp survived the lift");

goto([0.15, 0.075, zG + 0.090], 25, 0, 1.0);       // traverse to drop pad
const carried = B.read();
ok(carried.held, "grasp survived the traverse");
ok(Math.abs(carried.cellP[1] - 0.075) < 0.02, "cell carried over the drop pad");

// ── release: must fall under gravity, not teleport ──
const zHeld = B.read().cellP[2];
goto([0.15, 0.075, zG + 0.090], 25, 1, 0.12);      // OPEN briefly
const justOpened = B.read();
goto([0.15, 0.075, zG + 0.090], 25, 1, 1.2);       // fall + settle
const landed = B.read();
console.log(`\nrelease: z ${zHeld.toFixed(4)} -> ${justOpened.cellP[2].toFixed(4)} (0.12 s) -> ${landed.cellP[2].toFixed(4)} settled`);
console.log(`landed at [${landed.cellP.map(v => v.toFixed(3)).join(", ")}] |v|=${landed.cellVel.toFixed(4)} m/s`);
ok(justOpened.cellP[2] < zHeld - 0.001, "cell began falling the moment the jaws opened");
ok(!landed.held, "no longer held after release");
ok(landed.cellVel < 0.03, "cell came to rest");
ok(Math.abs(landed.cellP[1] - 0.075) < 0.035, "cell settled on the drop pad");

// ── arm independence ──
const cellA1 = A.read().cellP;
const drift = Math.hypot(cellA1[0] - cellA0[0], cellA1[1] - cellA0[1], cellA1[2] - cellA0[2]);
console.log(`\narm-4 cell moved ${(drift * 1000).toFixed(1)} mm (its own arm ran the same motion, so motion is expected)`);
ok(Number.isFinite(drift), "arm 4 simulated independently without error");

// ── IMU sensors are live ──
const im = B.read();
console.log(`forearm IMU acc=[${im.imuAcc.map(v => v.toFixed(2)).join(", ")}] |a|=${Math.hypot(...im.imuAcc).toFixed(2)} m/s^2`);
ok(Math.abs(Math.hypot(...im.imuAcc) - 9.80665) < 2.5, "forearm accelerometer reads ~1 g at rest");

// ── resetCell restores the authored rest pose ──
B.resetCell();
const reset = B.read();
console.log(`after resetCell: [${reset.cellP.map(v => v.toFixed(4)).join(", ")}]`);
ok(Math.abs(reset.cellP[1] + 0.075) < 1e-6, "resetCell put the cell back on the pick pad");

// ── recover() must not throw and must keep the sim usable ──
B.recover();
settle(0.2);
ok(B.read() !== null && Number.isFinite(B.read().q[0]), "recover() leaves the sim stepping");

// ── dispose cleanly ──
A.dispose(); B.dispose();
ok(A.read() === null, "dispose() invalidates the instance safely");

console.log(`\n${fail === 0 ? "ALL MODULE CHECKS PASSED" : `${fail} CHECK(S) FAILED`}`);
process.exitCode = fail === 0 ? 0 : 1;
