// ─────────────────────────────────────────────────────────────────────────────
// Operon — MuJoCo 3.x rigid-body contact physics for the browser studio.
//
// Wraps the official Google DeepMind MuJoCo WASM bindings (npm "mujoco",
// Apache-2.0) and exposes one independent simulation per arm rig.
//
// Why one model instance per arm: the 4-DOF and 6-DOF rigs are two separate
// experiments that happen to be drawn side by side. Giving each its own
// mjModel/mjData makes cross-rig contact physically impossible and keeps each
// rig's cell state independent, which is what the sim-vs-real comparison in
// teleop-studio.js assumes.
//
// The arm is driven through POSITION ACTUATORS, not by writing qpos. Commands
// are targets; what you read back is what the solver actually achieved under
// gravity, link inertia, joint damping and contact. That is the whole point:
// the cell is held by friction between two nitrile pads, so it can be grasped,
// carried, dropped, and knocked over, and none of it is scripted.
//
// Everything here degrades safely: if the WASM or the model fails to load the
// studio keeps running on its analytic kinematic path (see `available`).
// ─────────────────────────────────────────────────────────────────────────────

const ARM_JOINTS = ["yaw", "shoulder", "elbow", "wrist_roll", "wrist_pitch", "tool_roll"];
const GRIP_TRAVEL = 0.022;        // m, matches the finger slide range in the MJCF
const N_ARM = ARM_JOINTS.length;

// Realtime budget. The MJCF runs at 0.5 ms for stable elliptic-cone contact;
// at 60 fps that is ~33 substeps/arm/frame and costs ~1.5 ms for both arms.
const MAX_CATCHUP_S = 0.05;       // never try to replay more than 50 ms of lag
const MAX_SUBSTEPS = 80;          // hard ceiling per advance() call

let mujoco = null;                // the loaded WASM module (shared by all arms)
let loadPromise = null;

/** Load the MuJoCo WASM module once. */
function loadModule(vendorBase) {
  if (loadPromise) return loadPromise;
  loadPromise = (async () => {
    const factory = (await import(`${vendorBase}mujoco.js`)).default;
    return factory({
      locateFile: (file) => `${vendorBase}${file}`,
      // Keep MuJoCo's own stdout/stderr out of the console unless it matters.
      print: () => {},
      printErr: (m) => { if (String(m).trim()) console.warn("[mujoco]", m); },
    });
  })();
  return loadPromise;
}

/**
 * One fully independent physical arm + cell + table.
 * All poses are in the arm's own base frame, identical to fk6()/ik6().
 */
class ArmSim {
  constructor(xml, label) {
    this.label = label;
    this.ok = false;
    this.err = "";
    this._acc = 0;
    this._model = null;
    this._data = null;

    let spec = null;
    try {
      spec = mujoco.parseXMLString(xml);
      this._model = mujoco.mj_compile(spec);
      this._data = new mujoco.MjData(this._model);
    } catch (e) {
      this.err = (e && e.message) || String(e);
      try { spec?.delete?.(); } catch {}
      return;
    }
    try { spec.delete(); } catch {}

    const d = this._data;
    // Cache every Embind accessor handle ONCE. These are C++ handles on the
    // WASM heap; creating them per frame would leak without .delete().
    this._jnt = ARM_JOINTS.map((n) => d.jnt(n));
    this._fingerR = d.jnt("finger_r");
    this._cellFree = d.jnt("cell_free");
    this._valveJnt = d.jnt("valve_hinge");
    this._valveActIdx = mujoco.mj_name2id(this._model, mujoco.mjtObj.mjOBJ_ACTUATOR.value, "valve_act");
    this._hand = d.body("hand");
    this._cell = d.body("cell");
    this._sens = {};
    for (const n of ["forearm_acc", "forearm_gyro", "pad_l_touch", "pad_r_touch",
                     "grip_frc", "tcp_force"]) {
      this._sens[n] = d.sensor(n);
    }

    this.timestep = this._model.opt.timestep;
    this.nq = this._model.nq;
    this.nv = this._model.nv;

    // Remember the cell's authored rest pose so Reset can restore it exactly.
    this._cellRest = Array.from(this._cellFree.qpos).slice(0, 7);

    this._prevQ = new Array(N_ARM).fill(0);

    // Station props are PARKED below the floor when their station is not
    // selected, because the renderer hides them. If they stayed solid the arm
    // would collide with invisible geometry mid-traverse — which is exactly
    // what happens: a naive joint-space move from the pick pad to the drop pad
    // clips the charger dock. Parked bodies keep physics and pixels agreeing.
    this._props = {};
    for (const [station, names] of [[1, ["charger", "charger_mount"]], [2, ["valve", "valve_mount"]]]) {
      for (const n of names) {
        const id = mujoco.mj_name2id(this._model, mujoco.mjtObj.mjOBJ_BODY.value, n);
        if (id >= 0) {
          const bp = this._model.body_pos;
          this._props[n] = { id, station, home: [bp[id * 3], bp[id * 3 + 1], bp[id * 3 + 2]] };
        }
      }
    }
    this._station = null;
    this.setStation(0);

    this.ok = true;
  }

  /**
   * Show only the selected station's props as solid bodies; park the rest far
   * below the table where the arm cannot reach them.
   */
  setStation(idx) {
    if (this._station === idx) return;
    this._station = idx;
    const bp = this._model.body_pos;
    for (const p of Object.values(this._props)) {
      const active = p.station === idx;
      const t = active ? p.home : [p.home[0], p.home[1], p.home[2] - 5.0];
      bp[p.id * 3] = t[0]; bp[p.id * 3 + 1] = t[1]; bp[p.id * 3 + 2] = t[2];
    }
    try { mujoco.mj_forward(this._model, this._data); } catch {}
  }

  /** Valve handwheel angle in radians (Station 2). */
  valveAngle() {
    if (!this.ok || !this._valveJnt) return 0;
    return this._valveJnt.qpos[0];
  }

  /**
   * Command the handwheel angle (radians). Servoed, not teleported, so that
   * contacts with the tool resolve smoothly instead of exploding when the
   * gripper happens to be touching the rim.
   */
  setValveTarget(rad) {
    if (!this.ok || this._valveActIdx < 0) return;
    this._data.ctrl[this._valveActIdx] = rad;
  }

  /** Put the arm at q (6 rad) with the given jaw opening, at rest. Instant. */
  teleportTo(q, openFrac = 1) {
    if (!this.ok) return;
    const travel = Math.max(0, Math.min(1, openFrac)) * GRIP_TRAVEL;
    for (let i = 0; i < N_ARM; i++) {
      this._jnt[i].qpos[0] = q[i];
      this._jnt[i].qvel[0] = 0;
      this._data.ctrl[i] = q[i];
    }
    this._fingerR.qpos[0] = travel;
    this._data.ctrl[N_ARM] = travel;
    mujoco.mj_forward(this._model, this._data);
    this._prevQ = Array.from(q);
    this._acc = 0;
  }

  /** Command the arm's joint targets (radians). These are servo setpoints. */
  setArmTarget(q) {
    if (!this.ok) return;
    const m = this._model;
    for (let i = 0; i < N_ARM; i++) {
      const lo = m.actuator_ctrlrange[i * 2], hi = m.actuator_ctrlrange[i * 2 + 1];
      let v = q[i];
      if (!Number.isFinite(v)) v = this._jnt[i].qpos[0];
      this._data.ctrl[i] = Math.max(lo, Math.min(hi, v));
    }
  }

  /** Command jaw opening: 1 = fully open, 0 = fully closed (commanded). */
  setGripTarget(openFrac) {
    if (!this.ok) return;
    const v = Math.max(0, Math.min(1, openFrac)) * GRIP_TRAVEL;
    this._data.ctrl[N_ARM] = v;
  }

  /**
   * Advance physics by real elapsed wall time, in fixed substeps.
   * Lag is capped so a stalled tab can never trigger a catch-up death spiral.
   */
  advance(dtSeconds) {
    if (!this.ok) return 0;
    this._acc = Math.min(this._acc + Math.max(0, dtSeconds), MAX_CATCHUP_S);
    const h = this.timestep;
    let n = Math.floor(this._acc / h);
    if (n > MAX_SUBSTEPS) n = MAX_SUBSTEPS;
    if (n <= 0) return 0;
    this._acc -= n * h;
    for (let i = 0; i < N_ARM; i++) this._prevQ[i] = this._jnt[i].qpos[0];
    try {
      for (let s = 0; s < n; s++) mujoco.mj_step(this._model, this._data);
    } catch (e) {
      // A diverged solver must not take the whole studio down.
      this.err = (e && e.message) || String(e);
      this.recover();
      return 0;
    }
    if (!Number.isFinite(this._jnt[0].qpos[0])) this.recover();
    return n;
  }

  /** Reset dynamics but keep the current servo targets. */
  recover() {
    if (!this.ok) return;
    const ctrl = Array.from(this._data.ctrl);
    mujoco.mj_resetData(this._model, this._data);
    for (let i = 0; i < ctrl.length; i++) this._data.ctrl[i] = ctrl[i];
    for (let i = 0; i < N_ARM; i++) this._jnt[i].qpos[0] = ctrl[i];
    this._fingerR.qpos[0] = ctrl[N_ARM] ?? GRIP_TRAVEL;
    mujoco.mj_forward(this._model, this._data);
    this._acc = 0;
  }

  /**
   * Drop the cell at (x, y) lying on its side, rotated `yawDeg` about Z.
   * Orientation is Rz(yaw) * Rx(90 deg) as a quaternion, which lays the
   * cylinder's local +Z axis flat and spins it to the requested yaw — the same
   * convention as the authored rest pose. Used by domain randomisation.
   */
  placeCell(x, y, yawDeg, z = 0.0135) {
    if (!this.ok) return;
    const h = (yawDeg * Math.PI) / 360;            // half angle in radians
    const cz = Math.cos(h), sz = Math.sin(h), s = Math.SQRT1_2;
    const q = this._cellFree.qpos, v = this._cellFree.qvel;
    q[0] = x; q[1] = y; q[2] = z;
    q[3] = cz * s; q[4] = cz * s; q[5] = sz * s; q[6] = sz * s;
    for (let i = 0; i < 6; i++) v[i] = 0;
    mujoco.mj_forward(this._model, this._data);
  }

  /** Restore the cell to its authored rest pose on the pick pad. */
  resetCell() {
    if (!this.ok) return;
    const q = this._cellFree.qpos, v = this._cellFree.qvel;
    for (let i = 0; i < 7; i++) q[i] = this._cellRest[i];
    for (let i = 0; i < 6; i++) v[i] = 0;
    mujoco.mj_forward(this._model, this._data);
  }

  /**
   * Read the achieved physical state. Returns plain JS values (no handles), so
   * the caller never has to worry about WASM lifetimes.
   */
  read(dt = 1 / 60) {
    if (!this.ok) return null;
    const d = this._data;
    const q = new Array(N_ARM), dq = new Array(N_ARM);
    for (let i = 0; i < N_ARM; i++) {
      q[i] = this._jnt[i].qpos[0];
      dq[i] = this._jnt[i].qvel[0];
    }

    const hp = this._hand.xpos, hm = this._hand.xmat;
    const cp = this._cell.xpos, cm = this._cell.xmat;

    const travel = this._fingerR.qpos[0];
    const padL = this._sens.pad_l_touch.data[0] || 0;
    const padR = this._sens.pad_r_touch.data[0] || 0;
    const gripFrc = Math.abs(this._sens.grip_frc.data[0] || 0);

    const cellToTcp = Math.hypot(cp[0] - hp[0], cp[1] - hp[1], cp[2] - hp[2]);
    // Ground truth for "am I holding it": the pads are carrying load and the
    // cell is actually between the jaws. Sensor-based, no magnet and no pure
    // distance test -- but the TOTAL pad load rather than both pads
    // individually: as the tool re-orients, gravity shifts the cell onto one
    // pad and momentarily unloads the other, which is a secure grasp being
    // misreported as a drop.
    const held = (padL + padR) > 0.4 && cellToTcp < 0.045;

    const acc = this._sens.forearm_acc.data;
    const gyr = this._sens.forearm_gyro.data;
    const ft = this._sens.tcp_force.data;

    return {
      q, dq,
      tip: [hp[0], hp[1], hp[2]],
      R: [hm[0], hm[1], hm[2], hm[3], hm[4], hm[5], hm[6], hm[7], hm[8]],
      cellP: [cp[0], cp[1], cp[2]],
      cellR: [cm[0], cm[1], cm[2], cm[3], cm[4], cm[5], cm[6], cm[7], cm[8]],
      cellVel: Math.hypot(this._cellFree.qvel[0], this._cellFree.qvel[1], this._cellFree.qvel[2]),
      // Station 2: the handwheel's real angle. It is turned by the gripper
      // doing work against stem friction, and holds where it is left.
      valveRad: this._valveJnt ? this._valveJnt.qpos[0] : 0,
      // Visual jaw aperture 0..1, straight from the contact-stopped joint.
      open: Math.max(0, Math.min(1, travel / GRIP_TRAVEL)),
      jawTravelMm: travel * 1000,
      // Clear pad-to-pad opening: the inner face of each nitrile pad sits at
      // (0.00175 + travel) from the tool centreline, so the jaws close to a
      // 3.5 mm gap, not the 18 mm finger-body separation.
      jawGapMm: (0.00175 + travel) * 2000,
      gripForceN: gripFrc,
      padLoadN: [padL, padR],
      held,
      contacts: d.ncon,
      // Real PhysX-grade forearm IMU straight out of the solver.
      imuAcc: [acc[0] || 0, acc[1] || 0, acc[2] || 0],
      imuGyro: [gyr[0] || 0, gyr[1] || 0, gyr[2] || 0],
      tcpForceN: Math.hypot(ft[0] || 0, ft[1] || 0, ft[2] || 0),
      simTime: d.time,
    };
  }

  dispose() {
    try { this._jnt?.forEach((h) => h.delete()); } catch {}
    for (const h of [this._fingerR, this._cellFree, this._valveJnt, this._hand, this._cell]) {
      try { h?.delete(); } catch {}
    }
    try { Object.values(this._sens || {}).forEach((h) => h.delete()); } catch {}
    try { this._data?.delete(); } catch {}
    try { this._model?.delete(); } catch {}
    this.ok = false;
  }
}

/**
 * Boot MuJoCo and build one ArmSim per requested rig.
 * Resolves to { available, reason, arms, version } and never throws.
 */
export async function initPhysics({
  vendorBase = "/vendor/mujoco/",
  modelUrl = "/models/operon_station0.xml",
  rigs = [4, 6],
} = {}) {
  const out = { available: false, reason: "", arms: {}, timestep: 0, GRIP_TRAVEL };
  try {
    const [mod, xml] = await Promise.all([
      loadModule(vendorBase),
      fetch(modelUrl, { cache: "force-cache" }).then((r) => {
        if (!r.ok) throw new Error(`model ${r.status} ${r.statusText}`);
        return r.text();
      }),
    ]);
    mujoco = mod;

    for (const dof of rigs) {
      const sim = new ArmSim(xml, `arm${dof}`);
      if (!sim.ok) {
        for (const s of Object.values(out.arms)) s.dispose();
        out.reason = `MJCF compile failed: ${sim.err}`;
        return out;
      }
      out.arms[dof] = sim;
    }
    out.timestep = out.arms[rigs[0]].timestep;
    out.available = true;
    return out;
  } catch (e) {
    out.reason = (e && e.message) || String(e);
    return out;
  }
}

export { GRIP_TRAVEL, ARM_JOINTS };
