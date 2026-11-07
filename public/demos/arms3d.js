// 4-DOF vs 6-DOF arm kinematics + a tiny canvas 3D renderer. Z is up, meters.
//   4-DOF: yaw · shoulder pitch · elbow pitch · wrist pitch      (typical desktop arm)
//   6-DOF: yaw · shoulder pitch · elbow pitch · wrist roll · wrist pitch · tool roll (spherical wrist)
// Link lengths match Abel-Arm (L1 = L2 = 0.100 m, shoulder at z = 0.090 m) plus a 0.06 m tool.

export const G = { h0: 0.090, L1: 0.100, L2: 0.100, L3: 0.060 };
const D2R = Math.PI / 180;

// ---- tiny 3x3 matrix helpers (row-major arrays of 9) ----
export const M = {
  mul(a, b) {
    const r = new Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
      r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return r;
  },
  T(a) { return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]]; },
  col(a, j) { return [a[j], a[3 + j], a[6 + j]]; },
  Rx(t) { const c = Math.cos(t), s = Math.sin(t); return [1, 0, 0, 0, c, -s, 0, s, c]; },
  Ry(t) { const c = Math.cos(t), s = Math.sin(t); return [c, 0, s, 0, 1, 0, -s, 0, c]; },
  Rz(t) { const c = Math.cos(t), s = Math.sin(t); return [c, -s, 0, s, c, 0, 0, 0, 1]; },
  apply(a, v) { return [a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2], a[6] * v[0] + a[7] * v[1] + a[8] * v[2]]; }
};
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];

// Tool frame: z = approach direction. "Down" = top-down grasp.
export const R_DOWN = [1, 0, 0, 0, -1, 0, 0, 0, -1];

// Command pose from yaw/pitch/roll (degrees) relative to a top-down grasp.
export function poseR(yaw, pitch, roll) {
  return M.mul(M.Rz(yaw * D2R), M.mul(M.Ry(pitch * D2R), M.mul(M.Rx(roll * D2R), R_DOWN)));
}

// Angle between two rotations, treating the parallel gripper as symmetric under a 180° spin.
export function oriError(Ra, Rt) {
  const ang = (A) => { const R = M.mul(M.T(A), Rt); const tr = R[0] + R[4] + R[8]; return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))); };
  return Math.min(ang(Ra), ang(M.mul(Ra, M.Rz(Math.PI)))) / D2R;
}

// Planar 2-link solve: wrist center at radial r, height dz above the shoulder.
function planar(r, dz) {
  const { L1, L2 } = G;
  let d = Math.hypot(r, dz), reach = true;
  if (d > L1 + L2 - 1e-6) { const k = (L1 + L2 - 1e-6) / d; r *= k; dz *= k; d = L1 + L2 - 1e-6; reach = false; }
  if (d < 0.02) { reach = false; }
  const c = Math.max(-1, Math.min(1, (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)));
  const t3 = Math.acos(c);                                         // elbow bends "forward"
  const psi = Math.atan2(r, dz);
  const t2 = psi - Math.atan2(L2 * Math.sin(t3), L1 + L2 * Math.cos(t3));
  return { t2, t3, reach };
}

// ---------------- 6-DOF ----------------
export function fk6(q) {
  const [t1, t2, t3, t4, t5, t6] = q;
  const Y = M.Rz(t1);
  const sh = [0, 0, G.h0];
  const el = add(sh, M.apply(Y, [G.L1 * Math.sin(t2), 0, G.L1 * Math.cos(t2)]));
  const R03 = M.mul(Y, M.Ry(t2 + t3));
  const wr = add(el, scale(M.col(R03, 2), G.L2));
  const R = M.mul(R03, M.mul(M.Rz(t4), M.mul(M.Ry(t5), M.Rz(t6))));
  const tip = add(wr, scale(M.col(R, 2), G.L3));
  const yAx = M.col(Y, 1), fore = M.col(R03, 2), R04 = M.mul(R03, M.Rz(t4));
  const joints = [
    { p: [0, 0, 0.02], axis: [0, 0, 1], kind: "yaw" },
    { p: sh, axis: yAx, kind: "pitch" },
    { p: el, axis: yAx, kind: "pitch" },
    { p: add(el, scale(fore, G.L2 * 0.55)), axis: fore, kind: "roll" },
    { p: wr, axis: M.col(R04, 1), kind: "pitch" },
    { p: add(wr, scale(M.col(R, 2), G.L3 * 0.35)), axis: M.col(R, 2), kind: "roll" }
  ];
  return { pts: [[0, 0, 0], sh, el, wr, tip], R, tip, joints, yaw: t1 };
}

const wrapPi = (a) => {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
};
const angDist = (a, b) => Math.abs(wrapPi(a - b));

export function ik6(p, R, prevQ = null) {
  const z = M.col(R, 2);
  const w = [p[0] - G.L3 * z[0], p[1] - G.L3 * z[1], p[2] - G.L3 * z[2]];
  const t1 = Math.atan2(w[1], w[0]);
  const { t2, t3, reach } = planar(Math.hypot(w[0], w[1]), w[2] - G.h0);
  const R03 = M.mul(M.Rz(t1), M.Ry(t2 + t3));
  const R36 = M.mul(M.T(R03), R);
  // ZYZ decomposition of R36 = Rz(t4) Ry(t5) Rz(t6) with dual-branch wrist singularity resolution
  const r13 = R36[2], r23 = R36[5], r33 = R36[8], r31 = R36[6], r32 = R36[7];
  const s5 = Math.hypot(r13, r23);
  let t4, t5, t6;
  if (s5 < 1e-4) {
    t4 = prevQ ? wrapPi(prevQ[3]) : 0;
    t5 = r33 >= 0 ? 0 : Math.PI;
    const totalRoll = r33 >= 0 ? Math.atan2(R36[3], R36[0]) : Math.atan2(-R36[3], -R36[0]);
    t6 = wrapPi(totalRoll - (r33 >= 0 ? t4 : -t4));
  } else {
    // Branch A: t5 >= 0
    const t5a = Math.atan2(s5, r33);
    const t4a = Math.atan2(r23, r13);
    const t6a = Math.atan2(r32, -r31);
    // Branch B (equivalent orientation, flipped wrist): (-t5a, t4a + PI, t6a + PI)
    const t5b = -t5a;
    const t4b = wrapPi(t4a + Math.PI);
    const t6b = wrapPi(t6a + Math.PI);
    if (prevQ) {
      const costA = angDist(t4a, prevQ[3]) + angDist(t5a, prevQ[4]) + angDist(t6a, prevQ[5]);
      const costB = angDist(t4b, prevQ[3]) + angDist(t5b, prevQ[4]) + angDist(t6b, prevQ[5]);
      if (costB < costA) { t4 = t4b; t5 = t5b; t6 = t6b; }
      else { t4 = t4a; t5 = t5a; t6 = t6a; }
    } else {
      t4 = t4a; t5 = t5a; t6 = t6a;
    }
  }
  return { q: [wrapPi(t1), wrapPi(t2), wrapPi(t3), wrapPi(t4), wrapPi(t5), wrapPi(t6)], reach };
}

// ---------------- 4-DOF ----------------
export function fk4(q) {
  const [t1, t2, t3, t4] = q;
  const Y = M.Rz(t1);
  const sh = [0, 0, G.h0];
  const el = add(sh, M.apply(Y, [G.L1 * Math.sin(t2), 0, G.L1 * Math.cos(t2)]));
  const wr = add(el, M.apply(Y, [G.L2 * Math.sin(t2 + t3), 0, G.L2 * Math.cos(t2 + t3)]));
  const R = M.mul(Y, M.Ry(t2 + t3 + t4));
  const tip = add(wr, scale(M.col(R, 2), G.L3));
  const yAx = M.col(Y, 1);
  const joints = [
    { p: [0, 0, 0.02], axis: [0, 0, 1], kind: "yaw" },
    { p: sh, axis: yAx, kind: "pitch" },
    { p: el, axis: yAx, kind: "pitch" },
    { p: wr, axis: yAx, kind: "pitch" }
  ];
  return { pts: [[0, 0, 0], sh, el, wr, tip], R, tip, joints, yaw: t1 };
}

// Best effort: hit the position exactly; keep only the part of the approach
// direction that lies in the arm's vertical plane. Roll and off-plane yaw are lost.
export function ik4(p, R) {
  const t1 = Math.atan2(p[1], p[0]);
  const zt = M.apply(M.Rz(-t1), M.col(R, 2));       // approach dir in the yaw frame
  const phi = Math.atan2(zt[0], zt[2]);               // pitch of approach within the plane
  const w = [p[0] - G.L3 * Math.cos(t1) * Math.sin(phi), p[1] - G.L3 * Math.sin(t1) * Math.sin(phi), p[2] - G.L3 * Math.cos(phi)];
  const { t2, t3, reach } = planar(Math.hypot(w[0], w[1]), w[2] - G.h0);
  return { q: [t1, t2, t3, phi - t2 - t3], reach };
}

export const JOINTS = {
  4: [{ n: "yaw", lim: [-180, 180] }, { n: "shoulder", lim: [-120, 120] }, { n: "elbow", lim: [-150, 150] }, { n: "wrist pitch", lim: [-150, 150] }],
  6: [{ n: "yaw", lim: [-180, 180] }, { n: "shoulder", lim: [-120, 120] }, { n: "elbow", lim: [-150, 150] }, { n: "wrist roll", lim: [-180, 180] }, { n: "wrist pitch", lim: [-150, 150] }, { n: "tool roll", lim: [-180, 180] }]
};

// ---------------- renderer ----------------
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

export function makeView(canvas) {
  const ctx = canvas.getContext("2d");
  const cam = { yaw: -0.75, pitch: 0.36, dist: 0.5, fov: 1.0, target: [0.09, 0.01, 0.1] };

  function project(p, vp) {
    const [cx, cy, w, h] = vp;
    const cyaw = Math.cos(cam.yaw), syaw = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
    const x = p[0] - cam.target[0], y = p[1] - cam.target[1], z = p[2] - cam.target[2];
    const x1 = cyaw * x - syaw * y, y1 = syaw * x + cyaw * y;
    const depth = cp * y1 + sp * z, up = -sp * y1 + cp * z;
    const dz = cam.dist + depth;
    const f = (h / 2) / Math.tan(cam.fov / 2);
    return [cx + w / 2 + (x1 * f) / dz, cy + h / 2 - (up * f) / dz, dz];
  }
  function line(a, b, vp, color, width, dash) {
    const A = project(a, vp), B = project(b, vp);
    ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = "round";
    if (dash) ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(A[0], A[1]); ctx.lineTo(B[0], B[1]); ctx.stroke(); ctx.restore();
  }
  function poly(pts, vp, fill, stroke) {
    ctx.beginPath(); pts.forEach((p, i) => { const A = project(p, vp); i ? ctx.lineTo(A[0], A[1]) : ctx.moveTo(A[0], A[1]); }); ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); } if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1.5; ctx.stroke(); }
  }
  function dot(p, vp, r, fill, label) {
    const A = project(p, vp);
    ctx.fillStyle = fill; ctx.strokeStyle = "#1e1e1e"; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(A[0], A[1], r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    if (label) { ctx.fillStyle = "#1e1e1e"; ctx.font = "bold 11px 'Roboto Mono', monospace"; ctx.textAlign = "center"; ctx.fillText(label, A[0], A[1] + 4); ctx.textAlign = "left"; }
  }
  function ring(c, axis, r, vp, color, width = 4) {
    const a = norm(axis), t = Math.abs(a[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const u = norm(cross(a, t)), v = cross(a, u);
    ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    for (let k = 0; k <= 36; k++) { const th = k / 36 * Math.PI * 2; const p = add(c, add(scale(u, Math.cos(th) * r), scale(v, Math.sin(th) * r))); const A = project(p, vp); k ? ctx.lineTo(A[0], A[1]) : ctx.moveTo(A[0], A[1]); }
    ctx.stroke(); ctx.restore();
  }
  function arrow(from, dir, len, vp, color, width, dash) {
    const to = add(from, scale(dir, len));
    line(from, to, vp, color, width, dash);
    const A = project(from, vp), B = project(to, vp), ang = Math.atan2(B[1] - A[1], B[0] - A[0]);
    ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(B[0], B[1]);
    ctx.lineTo(B[0] - 13 * Math.cos(ang - .45), B[1] - 13 * Math.sin(ang - .45)); ctx.lineTo(B[0] - 13 * Math.cos(ang + .45), B[1] - 13 * Math.sin(ang + .45)); ctx.fill();
  }
  function gripper(tip, R, vp, alpha = 1, color = "#1e1e1e", w = 6) {
    const x = M.col(R, 0), z = M.col(R, 2);
    const base = add(tip, scale(z, -0.035));
    const f1 = add(base, scale(x, 0.028)), f2 = add(base, scale(x, -0.028));
    ctx.globalAlpha = alpha;
    line(f1, f2, vp, color, w);
    line(f1, add(f1, scale(z, 0.035)), vp, color, w);
    line(f2, add(f2, scale(z, 0.035)), vp, color, w);
    line(add(base, scale(z, -0.012)), base, vp, color, w);
    ctx.globalAlpha = 1;
  }
  function floor(vp) {
    for (let i = -3; i <= 6; i++) line([i * 0.05, -0.2, 0], [i * 0.05, 0.2, 0], vp, "#e6e6e6", 1);
    for (let j = -4; j <= 4; j++) line([-0.15, j * 0.05, 0], [0.3, j * 0.05, 0], vp, "#e6e6e6", 1);
  }

  // dof: 4 or 6. opts: {targetP, targetR, color, err}
  function arm(fk, vp, opts) {
    const { targetP, targetR, color, dof } = opts;
    floor(vp);
    if (dof === 4) {
      // the only plane a 4-DOF arm can point its gripper in: vertical, through the base, along its yaw
      const d = [Math.cos(fk.yaw), Math.sin(fk.yaw), 0];
      poly([scale(d, -0.06), scale(d, 0.3), add(scale(d, 0.3), [0, 0, 0.3]), add(scale(d, -0.06), [0, 0, 0.3])], vp, "rgba(251,188,4,.16)", "rgba(249,171,0,.55)");
      const lp = project(add(scale(d, 0.28), [0, 0, 0.29]), vp);
      ctx.fillStyle = "#b06000"; ctx.font = "13px 'Roboto Mono', monospace"; ctx.fillText("approach plane", lp[0] - 120, lp[1]);
    } else {
      // 6-DOF: a sphere of approach directions around the wrist
      ring(fk.pts[3], [0, 0, 1], 0.07, vp, "rgba(66,133,244,.25)", 2); ring(fk.pts[3], [1, 0, 0], 0.07, vp, "rgba(66,133,244,.25)", 2); ring(fk.pts[3], [0, 1, 0], 0.07, vp, "rgba(66,133,244,.25)", 2);
    }
    if (targetP) { gripper(targetP, targetR, vp, 0.35, "#ea4335", 6); }
    const pts = fk.pts;
    line(pts[0], pts[1], vp, "#1e1e1e", 16); line(pts[0], pts[1], vp, "#c3ecf6", 11);
    for (let i = 1; i < pts.length - 1; i++) { line(pts[i], pts[i + 1], vp, "#1e1e1e", 13); line(pts[i], pts[i + 1], vp, color, 8); }
    fk.joints.forEach((j, i) => {
      if (j.kind === "roll") { ring(j.p, j.axis, 0.018, vp, "#1e1e1e", 6); ring(j.p, j.axis, 0.018, vp, "#4285f4", 3); }
      dot(j.p, vp, 9, j.kind === "roll" ? "#c3ecf6" : "#fff", String(i + 1));
    });
    gripper(fk.tip, fk.R, vp, 1, "#1e1e1e", 7);
    if (targetR) {
      arrow(fk.tip, M.col(targetR, 2), 0.07, vp, "#ea4335", 3, [6, 4]);
    }
    arrow(fk.tip, M.col(fk.R, 2), 0.07, vp, "#1e1e1e", 3.5);
  }
  return { ctx, cam, project, arm, line, dot, ring };
}
