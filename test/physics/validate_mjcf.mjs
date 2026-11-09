import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
// Validate public/models/operon_station0.xml against public/demos/arms3d.js
import fs from "node:fs";
import load from "mujoco";
import { fk6, ik6, poseR } from "../../public/demos/arms3d.js";

const XML = fs.readFileSync(`${REPO}/public/models/operon_station0.xml`, "utf8");
const mujoco = await load();

const ARM = ["yaw", "shoulder", "elbow", "wrist_roll", "wrist_pitch", "tool_roll"];
let spec, model, data;
let fail = 0;
const ok = (c, msg) => { console.log(`${c ? "  PASS" : "  FAIL"}  ${msg}`); if (!c) fail++; };

try {
  spec = mujoco.parseXMLString(XML);
  model = mujoco.mj_compile(spec);
  data = new mujoco.MjData(model);
  console.log(`compiled: nq=${model.nq} nv=${model.nv} nu=${model.nu} nbody=${model.nbody} nsensordata=${model.nsensordata}`);
  // Mirror ArmSim.setStation(0): park the Station 1/2 props below the table.
  // Runtime always does this (the renderer hides them), and leaving them solid
  // makes the arm clip the charger dock during the pick-and-place traverse.
  for (const n of ["charger", "charger_mount", "valve", "valve_mount"]) {
    const id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, n);
    if (id >= 0) model.body_pos[id * 3 + 2] -= 5.0;
  }
  mujoco.mj_forward(model, data);

  const jnt = {}; for (const n of [...ARM, "finger_l", "finger_r"]) jnt[n] = data.jnt(n);
  const hand = data.body("hand");
  const cell = data.body("cell");
  const cellFree = data.jnt("cell_free");

  const setArm = (q) => { ARM.forEach((n, i) => { jnt[n].qpos[0] = q[i]; }); };
  const setGrip = (travel) => { jnt.finger_r.qpos[0] = travel; jnt.finger_l.qpos[0] = -travel; };

  // ── 1. forward-kinematics agreement with fk6() ──
  console.log("\n[1] MJCF chain vs fk6() — hand origin must equal fk6(q).tip, hand frame must equal fk6(q).R");
  const cases = [
    [0, 0, 0, 0, 0, 0],
    [0.3, 0.5, -0.4, 0.2, 0.6, -0.3],
    [-1.1, 0.9, 0.7, -1.4, -0.5, 2.0],
    [2.4, -0.6, 1.2, 0.8, 1.1, -1.9],
  ];
  let maxP = 0, maxR = 0;
  for (const q of cases) {
    setArm(q); setGrip(0.022);
    mujoco.mj_forward(model, data);
    const f = fk6(q);
    const p = [hand.xpos[0], hand.xpos[1], hand.xpos[2]];
    const dp = Math.hypot(p[0] - f.tip[0], p[1] - f.tip[1], p[2] - f.tip[2]);
    // mjData.xmat is row-major 3x3, same convention as arms3d M
    let dr = 0; for (let i = 0; i < 9; i++) dr = Math.max(dr, Math.abs(hand.xmat[i] - f.R[i]));
    maxP = Math.max(maxP, dp); maxR = Math.max(maxR, dr);
    console.log(`    q=[${q.map(v => v.toFixed(2)).join(",")}]  |dp|=${(dp * 1e6).toFixed(3)} um  max|dR|=${dr.toExponential(2)}`);
  }
  ok(maxP < 1e-9, `tip position matches fk6 to <1 nm (worst ${(maxP * 1e9).toFixed(2)} nm)`);
  ok(maxR < 1e-9, `tip orientation matches fk6 to <1e-9 (worst ${maxR.toExponential(2)})`);

  // ── 2. 4-DOF constraint representability ──
  console.log("\n[2] 4-DOF arm as the same chain with roll joints zeroed");
  const q4 = [0.4, 0.55, -0.35, 0.22];
  setArm([q4[0], q4[1], q4[2], 0, q4[3], 0]);
  mujoco.mj_forward(model, data);
  const { fk4 } = await import(`${REPO}/public/demos/arms3d.js`);
  const f4 = fk4(q4);
  const d4 = Math.hypot(hand.xpos[0] - f4.tip[0], hand.xpos[1] - f4.tip[1], hand.xpos[2] - f4.tip[2]);
  ok(d4 < 1e-9, `fk4 tip reproduced with wrist_roll=tool_roll=0 (${(d4 * 1e9).toFixed(2)} nm)`);

  // ── 3. gravity: arm must sag under load, not float ──
  console.log("\n[3] gravity response (position servos holding against real link mass)");
  mujoco.mj_resetData(model, data);
  const home = ik6([0.15, -0.075, 0.12], poseR(-25, 0, 0)).q;
  setArm(home); setGrip(0.022);
  ARM.forEach((n, i) => { data.ctrl[i] = home[i]; });
  data.ctrl[6] = 0.022;
  mujoco.mj_forward(model, data);
  for (let i = 0; i < 2000; i++) mujoco.mj_step(model, data);   // 1.0 s settle
  const sag = ARM.map((n, i) => Math.abs(jnt[n].qpos[0] - home[i]));
  const maxSag = Math.max(...sag);
  console.log(`    steady-state tracking error per joint (deg): ${sag.map(v => (v * 180 / Math.PI).toFixed(2)).join(" ")}`);
  ok(maxSag > 1e-5, `arm has real compliance (non-zero sag), not kinematic snapping`);
  ok(maxSag < 0.12, `sag stays under 7 deg so the servo can still hold pose (max ${(maxSag * 180 / Math.PI).toFixed(2)} deg)`);

  // ── 4. the cell must rest stably on the pick pad ──
  console.log("\n[4] cell at rest on the pick pad");
  const z0 = cell.xpos[2];
  console.log(`    cell z after settle = ${z0.toFixed(5)} m (expect ~0.0135: 12 mm radius + 1 mm pad, lying on its side)`);
  ok(Math.abs(z0 - 0.0135) < 0.004, `cell rests on its side on the pad at the right height`);
  ok(Math.hypot(cellFree.qvel[0], cellFree.qvel[1], cellFree.qvel[2]) < 1e-3, `cell is at rest (not jittering)`);

  // ── 5. THE GRASP: close on the cell, lift, and see if friction holds it ──
  console.log("\n[5] friction grasp — descend, close, lift. No welds, no teleport.");
  const cellP = [cell.xpos[0], cell.xpos[1], cell.xpos[2]];
  const graspR = poseR(-25, 0, 0);
  const GRASP_H = 0.005;   // grip the cell's upper half; fingertips stay clear of the pad

  const driveTo = (p, R, ms, grip) => {
    const q = ik6(p, R, ARM.map((n) => jnt[n].qpos[0])).q;
    ARM.forEach((n, i) => { data.ctrl[i] = q[i]; });
    if (grip !== undefined) data.ctrl[6] = grip;
    const n = Math.round(ms / 1000 / model.opt.timestep);
    for (let i = 0; i < n; i++) mujoco.mj_step(model, data);
  };

  driveTo([cellP[0], cellP[1], cellP[2] + 0.075], graspR, 600, 0.022);   // approach above
  driveTo([cellP[0], cellP[1], cellP[2] + GRASP_H], graspR, 700, 0.022);   // descend around cell
  const padGapOpen = jnt.finger_r.qpos[0];
  driveTo([cellP[0], cellP[1], cellP[2] + GRASP_H], graspR, 500, 0.0);     // CLOSE on the cell
  const padGapClosed = jnt.finger_r.qpos[0];
  const touchL = data.sensor("pad_l_touch").data[0];
  const touchR = data.sensor("pad_r_touch").data[0];
  const gripFrc = Math.abs(data.sensor("grip_frc").data[0]);
  console.log(`    jaw travel: open=${(padGapOpen * 1000).toFixed(2)} mm -> closed=${(padGapClosed * 1000).toFixed(2)} mm`);
  console.log(`    pad normal load  L=${touchL.toFixed(2)} N  R=${touchR.toFixed(2)} N   grip actuator force=${gripFrc.toFixed(1)} N`);
  ok(padGapClosed > 0.0015, `jaws STOPPED on the cell instead of closing through it (${(padGapClosed * 1000).toFixed(2)} mm > 0 commanded)`);
  ok(touchL > 0.5 && touchR > 0.5, `both pads are loaded (real two-sided contact)`);

  const zBefore = cell.xpos[2];
  driveTo([cellP[0], cellP[1], cellP[2] + 0.090], graspR, 900, 0.0);       // LIFT
  const zAfter = cell.xpos[2];
  const lifted = zAfter - zBefore;
  const tipToCell = Math.hypot(cell.xpos[0] - hand.xpos[0], cell.xpos[1] - hand.xpos[1], cell.xpos[2] - hand.xpos[2]);
  console.log(`    cell rose ${(lifted * 1000).toFixed(1)} mm; cell-to-TCP distance ${(tipToCell * 1000).toFixed(1)} mm`);
  ok(lifted > 0.060, `cell was actually lifted by friction (${(lifted * 1000).toFixed(1)} mm)`);
  ok(tipToCell < 0.030, `cell stayed in the jaws during the lift`);

  // ── 6. RELEASE: open the jaws, the cell must fall under gravity ──
  console.log("\n[6] release — the cell must fall and settle, not teleport");
  driveTo([cellP[0], 0.075, cellP[2] + 0.090], graspR, 900, 0.0);        // traverse to drop pad
  const zHeld = cell.xpos[2];
  driveTo([cellP[0], 0.075, cellP[2] + 0.090], graspR, 200, 0.022);      // OPEN
  const zJustAfter = cell.xpos[2];
  driveTo([cellP[0], 0.075, cellP[2] + 0.090], graspR, 900, 0.022);      // let it fall & settle
  const zLanded = cell.xpos[2];
  const vLanded = Math.hypot(cellFree.qvel[0], cellFree.qvel[1], cellFree.qvel[2]);
  console.log(`    cell z: held=${zHeld.toFixed(4)} -> 0.2 s after open=${zJustAfter.toFixed(4)} -> settled=${zLanded.toFixed(4)}`);
  console.log(`    landed over drop pad at x=${cell.xpos[0].toFixed(3)} y=${cell.xpos[1].toFixed(3)}, |v|=${vLanded.toFixed(4)} m/s`);
  ok(zJustAfter < zHeld - 0.002, `cell began falling immediately on release (gravity, not a teleport)`);
  ok(Math.abs(zLanded - 0.0135) < 0.006, `cell settled back onto the pad surface`);
  ok(vLanded < 0.02, `cell came to rest`);
  ok(Math.abs(cell.xpos[1] - 0.075) < 0.030, `cell landed on the drop pad`);

  console.log(`\n${fail === 0 ? "ALL CHECKS PASSED" : `${fail} CHECK(S) FAILED`}`);
  process.exitCode = fail === 0 ? 0 : 1;
} catch (e) {
  console.error("ERROR:", e && e.message ? e.message : e);
  process.exitCode = 1;
} finally {
  data?.delete?.(); model?.delete?.(); spec?.delete?.();
}
