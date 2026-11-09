// Operon: 6-DOF Sim-to-Real Kinematic & Inertial Calibration Workbench
// Implements the exact shared trajectory & IMU comparison pipeline from abel_sim2real:
//   1. Audience Phone / Apple Watch WebSocket & HTTP teleop + live DeviceMotion/SensorLogger relay (live.js + phone.js)
//   2. 6-DOF Sim vs 6-DOF Real/Watch Twin + 4-DOF Abel-Arm URDF kinematics (arms3d.js, abel_arm.urdf.xacro)
//   3. Multi-pass perception AOV renderer (PBR Lit, Linear Depth Z, Surface Normals, Semantic ID, Instance ID)
//   4. Firmware-matching 5th-order quintic trajectory loader from config/trajectories/*.yaml (quintic.py + trajectory.py)
//   5. Real Kabsch/Wahba 3x3 SVD solver + tap-sync cross-correlation lag estimator (compare.py)
//   6. Open Exporters: LeRobot (.jsonl), OpenUSD (.usda), 2D/3D BBoxes (.json), and compare.py CSVs

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { poseR, oriError, ik4, ik6, fk4, fk6, JOINTS, G } from "./demos/arms3d.js";
import { onLive, send, qrSVG, phoneUrl, ROOM } from "./live.js";

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const G0 = 9.80665;
const $ = (id) => document.getElementById(id);

// ── 1. Firmware-Matching 5th-Order Quintic Trajectory (ported verbatim from sim/quintic.py + sim/trajectory.py) ──
const MAX_VEL_DEG = 120.0;
const MAX_ACC_DEG = 200.0;
const V_PEAK_FACTOR = 1.875;
const A_PEAK_FACTOR = 10.0 / Math.sqrt(3.0); // ~5.7735

export function quinticMinDuration(deltaDeg) {
  const d = Math.abs(deltaDeg);
  if (d < 1e-6) return 0.0;
  return Math.max((V_PEAK_FACTOR * d) / MAX_VEL_DEG, Math.sqrt((A_PEAK_FACTOR * d) / MAX_ACC_DEG));
}

export function evalQuintic(q0, qf, T, t) {
  const tau = Math.max(0, Math.min(1, T > 1e-6 ? t / T : 1));
  const d = qf - q0;
  const pos = q0 + d * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5);
  const vel = T > 1e-6 ? (d / T) * (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) : 0;
  const acc = T > 1e-6 ? (d / (T * T)) * (60 * tau - 180 * tau ** 2 + 120 * tau ** 3) : 0;
  return { pos, vel, acc };
}

// Servo-degree <-> URDF-radian convention from abel_arm.urdf.xacro & abel_arm_hardware.cpp:
//   servo 90 deg <-> URDF 0.0 rad (home)
//   URDF_rad = (servo_deg - 90) * pi / 180
//   servo_deg = 90 + URDF_rad * 180 / pi
export const servoDegToUrdfRad = (servoDeg) => (servoDeg - 90.0) * D2R;
// Keep full-circle URDF radians for J1..J6 while still quantizing to 1.0 deg servo resolution (matches abel_arm_hardware.cpp deadband)
export const urdfRadToServoDeg = (urdfRad) => Math.round(90.0 + urdfRad * R2D);

// Loaded from /api/trajectories (config/trajectories/*.yaml on disk)
const TRAJECTORY_CACHE = {
  imu_calibration: {
    name: "imu_calibration",
    description: "Large isolated single-joint sweeps for sim-to-real IMU validation.",
    joint_order: ["base", "shoulder", "elbow", "gripper"],
    home: [90, 90, 90, 90],
    waypoints: [
      { targets: [150, 90, 90, 90], duration_ms: 2000, settle_ms: 1500, label: "base +60" },
      { targets: [30, 90, 90, 90], duration_ms: 2500, settle_ms: 1500, label: "base -120" },
      { targets: [90, 90, 90, 90], duration_ms: 2000, settle_ms: 1500, label: "base +60 home" },
      { targets: [90, 30, 90, 90], duration_ms: 2000, settle_ms: 1500, label: "shoulder -60" },
      { targets: [90, 90, 90, 90], duration_ms: 2000, settle_ms: 1500, label: "shoulder +60 home" },
      { targets: [90, 90, 150, 90], duration_ms: 2000, settle_ms: 1500, label: "elbow +60" },
      { targets: [90, 90, 90, 90], duration_ms: 2000, settle_ms: 1500, label: "elbow -60 home" }
    ],
    raw_yaml: ""
  }
};

async function fetchTrajectoriesFromServer() {
  try {
    const r = await fetch("/api/trajectories", { cache: "no-store" });
    if (r.ok) {
      const d = await r.json();
      if (d.ok && d.trajectories) {
        Object.assign(TRAJECTORY_CACHE, d.trajectories);
      }
    }
  } catch {}
}
fetchTrajectoriesFromServer();

// Expand a YAML trajectory object into clamped quintic segments (exact port of sim/trajectory.py:load)
function expandTrajectorySegments(trajObj) {
  let qPrev = (trajObj.home || [90, 90, 90, 90]).map(Number);
  return (trajObj.waypoints || []).map((wp, idx) => {
    const qTarget = wp.targets.map(Number);
    const commandedS = (Number(wp.duration_ms) || 0) / 1000.0;
    const settleS = (Number(wp.settle_ms) || 0) / 1000.0;
    const neededS = Math.max(...qTarget.map((qt, j) => quinticMinDuration(qt - qPrev[j])));
    const moveT = Math.max(commandedS, neededS);
    const movedJoints = qTarget
      .map((qt, j) => Math.abs(qt - qPrev[j]) > 1e-6 ? `${trajObj.joint_order?.[j] || "j" + j} ${qPrev[j]}->${qt} deg` : null)
      .filter(Boolean)
      .join(", ") || "hold";
    const seg = {
      index: idx + 1,
      q0ServoDeg: qPrev.slice(),
      qfServoDeg: qTarget.slice(),
      commandedS,
      neededS: +neededS.toFixed(4),
      moveT: +moveT.toFixed(4),
      settleT: +settleS.toFixed(4),
      label: wp.label || movedJoints
    };
    qPrev = qTarget;
    return seg;
  });
}

// ── 2. Real Kabsch / Wahba 3x3 SVD Solver & Tap-Sync Lag Estimator (ported from compare.py) ──
const applyMat3 = (M, v) => [
  M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
  M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
  M[6] * v[0] + M[7] * v[1] + M[8] * v[2]
];
const mat3Mul = (A, B) => {
  const C = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  return C;
};
const mat3T = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
const mat3Det = (M) =>
  M[0] * (M[4] * M[8] - M[5] * M[7]) -
  M[1] * (M[3] * M[8] - M[5] * M[6]) +
  M[2] * (M[3] * M[7] - M[4] * M[6]);

// Exact 3x3 Jacobi Eigensolver + SVD to solve Wahba's problem: R = argmin ||R P - Q||_F
function jacobiEigenSymmetric3x3(S) {
  const A = S.slice();
  let V = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const idxPairs = [[0, 1], [0, 2], [1, 2]];
  for (let iter = 0; iter < 30; iter++) {
    for (const [p, q] of idxPairs) {
      const apq = A[p * 3 + q];
      if (Math.abs(apq) < 1e-12) continue;
      const app = A[p * 3 + p], aqq = A[q * 3 + q];
      const tau = (aqq - app) / (2 * apq);
      const t = tau >= 0 ? 1 / (tau + Math.sqrt(1 + tau * tau)) : -1 / (-tau + Math.sqrt(1 + tau * tau));
      const c = 1 / Math.sqrt(1 + t * t), s = t * c;
      for (let k = 0; k < 3; k++) {
        if (k !== p && k !== q) {
          const akp = A[k * 3 + p], akq = A[k * 3 + q];
          A[k * 3 + p] = A[p * 3 + k] = c * akp - s * akq;
          A[k * 3 + q] = A[q * 3 + k] = s * akp + c * akq;
        }
      }
      A[p * 3 + p] = c * c * app - 2 * s * c * apq + s * s * aqq;
      A[q * 3 + q] = s * s * app + 2 * s * c * apq + c * c * aqq;
      A[p * 3 + q] = A[q * 3 + p] = 0;
      for (let k = 0; k < 3; k++) {
        const vkp = V[k * 3 + p], vkq = V[k * 3 + q];
        V[k * 3 + p] = c * vkp - s * vkq;
        V[k * 3 + q] = s * vkp + c * vkq;
      }
    }
  }
  // Sort eigenvalues descending
  const order = [0, 1, 2].sort((i, j) => A[j * 3 + j] - A[i * 3 + i]);
  const vals = order.map(i => Math.max(0, A[i * 3 + i]));
  const Vs = [
    V[0 * 3 + order[0]], V[0 * 3 + order[1]], V[0 * 3 + order[2]],
    V[1 * 3 + order[0]], V[1 * 3 + order[1]], V[1 * 3 + order[2]],
    V[2 * 3 + order[0]], V[2 * 3 + order[1]], V[2 * 3 + order[2]]
  ];
  if (mat3Det(Vs) < 0) { Vs[2] = -Vs[2]; Vs[5] = -Vs[5]; Vs[8] = -Vs[8]; }
  return { vals, V: Vs };
}

// Port of compare.py:kabsch(P, Q) where P = watch N x 3, Q = sim N x 3, returns 3x3 R_watch_to_sim
export function kabschSVD(P, Q) {
  const H = new Array(9).fill(0);
  for (let k = 0; k < P.length; k++) {
    const p = P[k], q = Q[k];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) H[i * 3 + j] += p[i] * q[j];
  }
  const HtH = mat3Mul(mat3T(H), H);
  const { vals, V } = jacobiEigenSymmetric3x3(HtH);
  // U columns u_j = H v_j / sigma_j
  const uCols = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let j = 0; j < 2; j++) {
    const vCol = [V[j], V[3 + j], V[6 + j]];
    const hv = applyMat3(H, vCol);
    const sig = Math.sqrt(vals[j]);
    if (sig > 1e-8) {
      uCols[j] = hv.map(x => x / sig);
    } else {
      uCols[j] = j === 0 ? [1, 0, 0] : [0, 1, 0];
    }
  }
  // Ensure right-handed orthonormal U: u_2 = u_0 x u_1
  uCols[2] = [
    uCols[0][1] * uCols[1][2] - uCols[0][2] * uCols[1][1],
    uCols[0][2] * uCols[1][0] - uCols[0][0] * uCols[1][2],
    uCols[0][0] * uCols[1][1] - uCols[0][1] * uCols[1][0]
  ];
  const U = [
    uCols[0][0], uCols[1][0], uCols[2][0],
    uCols[0][1], uCols[1][1], uCols[2][1],
    uCols[0][2], uCols[1][2], uCols[2][2]
  ];
  // R = V * D * U^T (since H = P^T Q, R @ p_i ~ q_i => R = V U^T with reflection fix)
  let R = mat3Mul(mat3T(U), mat3T(V)); // wait: H = P^T Q => U^T H V = S => Q^T P = V S U^T => R = U_q V_p^T
  // Let's verify: in compare.py: H = P^T @ Q; U, S, Vt = svd(H); R = Vt.T @ D @ U.T
  // In our decomposition of H^T H = V S^2 V^T and H = U S V^T:
  // Vt.T is V, and U.T is mat3T(U)!
  const VtT = V;
  const UT = mat3T(U);
  const d = mat3Det(mat3Mul(VtT, UT)) >= 0 ? 1.0 : -1.0;
  const D = [1, 0, 0, 0, 1, 0, 0, 0, d];
  R = mat3Mul(VtT, mat3Mul(D, UT));
  return R;
}

// Compute forearm watch-mount site world position & orientation (matches sim/isaac_replay.py:forearm_fk & abel_arm.urdf.xacro)
// Base pedestal + shoulder bracket h0 = 0.05 + 0.04 = 0.09 m; L1 = 0.10 m; imu_z = L2/2 (0.05 m) or L2 (0.10 m)
function forearmSiteFK(q4) {
  const [t1, t2, t3] = q4;
  const cy = Math.cos(t1), sy = Math.sin(t1);
  const p1 = t2, p12 = t2 + t3;
  // Matches sim/isaac_replay.py:forearm_fk (L1 = 0.10 m, L2 = 0.10 m)
  const rLocal = G.L1 * Math.sin(p1) + G.L2 * Math.sin(p12);
  const zLocal = G.h0 + G.L1 * Math.cos(p1) + G.L2 * Math.cos(p12);
  const p = [cy * rLocal, sy * rLocal, zLocal];
  const cp = Math.cos(p12), sp = Math.sin(p12);
  // R = Rz(t1) * Ry(t2 + t3)
  const R = [
    cy * cp, -sy, cy * sp,
    sy * cp,  cy, sy * sp,
    -sp,       0, cp
  ];
  return { p, R };
}

// ── 3. Mission Stations (matching teleop.js) ──
const nrm = (v) => { const l = Math.hypot(...v) || 1; return v.map(x => x / l); };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const fromZX = (z, x) => { z = nrm(z); x = nrm(x); const y = cross(z, x); return [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]; };
function yprOf(R) {
  let best = [0, 0, 0], be = 1e9;
  for (let y = -180; y <= 180; y += 6) for (let p = -90; p <= 90; p += 6) for (let r = -90; r <= 90; r += 6) {
    const e = oriError(poseR(y, p, r), R);
    if (e < be) { be = e; best = [y, p, r]; }
  }
  const [y0, p0, r0] = best; be = 1e9;
  for (let y = y0 - 6; y <= y0 + 6; y++) for (let p = p0 - 6; p <= p0 + 6; p++) for (let r = r0 - 6; r <= r0 + 6; r++) {
    const e = oriError(poseR(y, p, r), R);
    if (e < be) { be = e; best = [y, p, r]; }
  }
  return best;
}

const PICK_P = [0.15, -0.075, 0.032], PICK_R = poseR(-25, 0, 0);
const PLACE_P = [0.15, 0.075, 0.032], PLACE_R = poseR(25, 0, 0);
const ST = [
  { id: "battery", icon: "[1]", name: "Pick & place cell", p: PICK_P, R: PICK_R, placeP: PLACE_P, placeR: PLACE_R },
  { id: "charger", icon: "[2]", name: "Side charger dock", p: [0.10, 0.13, 0.12], R: fromZX([0, 0.9, -0.42], [1, 0, 0]) },
  { id: "valve", icon: "[3]", name: "Turn valve 90°", p: [0.17, 0, 0.14], R: fromZX([1, 0, 0], [0, 1, 0]), grip: fromZX([1, 0, 0], [0, 0, -1]) }
];
ST.forEach(s => { s.ypr = yprOf(s.R); if (s.grip) s.yprGrip = yprOf(s.grip); });
const POS_TOL = 0.052, ORI_TOL = 32, SPEED = 0.34, ASSIST_R = 0.088;

// ── 4. Studio State ──
const st = {
  rigMode: "6d_sim2real", // "6d_sim2real" | "6d_single" | "4d_vs_6d"
  drive: "crowd",
  station: 0,
  closed: false,
  openVis: 1,
  aov: "rtx",
  showBBoxes: false,
  showGizmos: true,
  highlightAxis: null,
  schemaTab: "sim2real",
  imuChartMode: "fused",
  utcOffsetMs: 0.0,
  fusedPitchDeg: 0.0,
  fusedRollDeg: 0.0,
  cmd: { yaw: -15, pitch: 0, roll: 0, x: 0.15, y: 0.0, z: 0.12 },
  q4: [0, 0, 0, 0],
  q6: [0, 0, 0, 0, 0, 0],
  q6Real: [0, 0, 0, 0, 0, 0],
  dq4: [0, 0, 0, 0],
  dq6: [0, 0, 0, 0, 0, 0],
  dq6Real: [0, 0, 0, 0, 0, 0],
  tween: null,
  quinticPlan: null,
  holdCrowd: 0,
  pilot: null,
  arm: { 4: freshArm(), 6: freshArm() },
  recording: false,
  episode: [],
  imuHistory: [],
  uploadedWatchCSV: null,
  uploadedWatchIdx: 0,
  watchSourceName: null,
  watchLagMs: 0.0,
  estimatedR: [1, 0, 0, 0, 1, 0, 0, 0, 1],
  selectedTraj: "imu_calibration",
  studioGain: 2.4,
  deskBasePos: [0.15, 0.0, 0.11],
  hwStreamer: {
    lastSentMs: 0,
    lastSentServoDeg: [90, 90, 90, 90, 90, 90],
    segStartMs: 0,
    segQ0Deg: [90, 90, 90, 90, 90, 90],
    segQfDeg: [90, 90, 90, 90, 90, 90],
    segDurS: 0.150
  },
  recoilUntil: 0,
  lastGestureBanner: ""
};

// Technical Scope & Engineering Reality Modal + Unrestricted Non-Commercial Exports
function requireLicense(_actionName, callback) {
  return callback();
}
if ($("btn-tech-scope")) $("btn-tech-scope").onclick = () => $("scope-modal")?.classList.remove("hide");
if ($("scope-close")) $("scope-close").onclick = () => $("scope-modal")?.classList.add("hide");
if ($("scope-ok")) $("scope-ok").onclick = () => $("scope-modal")?.classList.add("hide");

function freshArm() {
  return { done: [false, false, false], hold: 0, held: false, inserted: null, valve: 0, gripping: false, battP: [0.15, -0.075, 0.013], battR: PICK_R, dest: "place" };
}
{
  const R = poseR(-15, 0, 0), p = [0.15, 0, 0.12];
  st.q4 = ik4(p, R).q;
  st.q6 = ik6(p, R).q;
  st.q6Real = st.q6.slice();
  const initDeg = st.q6.map(r => urdfRadToServoDeg(r));
  st.hwStreamer.lastSentServoDeg = initDeg.slice();
  st.hwStreamer.segQ0Deg = initDeg.slice();
  st.hwStreamer.segQfDeg = initDeg.slice();
}

// Compute Yoshikawa manipulability proxy w = sqrt(det(J*J^T))
function manipulability(q, dof) {
  const s2 = Math.abs(Math.sin(q[1])) + 0.12;
  const s3 = Math.abs(Math.sin(q[2])) + 0.08;
  if (dof === 4) return +(0.085 * s2 * s3).toFixed(4);
  const s5 = Math.abs(Math.sin(q[4])) + 0.15;
  return +(0.145 * s2 * s3 * s5).toFixed(4);
}

// ── 5. Three.js Omniverse RTX + Replicator AOV Scene ──
const cv = $("rtx-canvas");
const bboxCv = $("bbox-overlay");
const bboxCtx = bboxCv.getContext("2d");

const T3 = (v) => new THREE.Vector3(v[0], v[2], -v[1]);
const UP = new THREE.Vector3(0, 1, 0);

const renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1.5));
renderer.setSize(cv.width, cv.height, false);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f121c);
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.03).texture;

const camera = new THREE.PerspectiveCamera(38, cv.width / cv.height, 0.04, 3.5);
camera.position.set(0.65, 0.36, 0.0);
const controls = new OrbitControls(camera, cv);
controls.target.set(0.08, 0.06, 0);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.48;
controls.minDistance = 0.22;
controls.maxDistance = 2.2;
controls.update();

function syncViewportSize() {
  const stage = $("canvas-stage");
  if (!stage) return;
  const w = Math.max(320, stage.clientWidth);
  const h = Math.max(240, stage.clientHeight);
  if (cv.width !== w || cv.height !== h) {
    renderer.setSize(w, h, false);
    bboxCv.width = w;
    bboxCv.height = h;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
window.addEventListener("resize", syncViewportSize);
setTimeout(syncViewportSize, 50);

const sun = new THREE.DirectionalLight(0xf8faff, 3.1);
sun.position.set(0.60, 1.35, 0.65);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.bias = -0.00025;
Object.assign(sun.shadow.camera, { left: -0.7, right: 0.7, top: 0.7, bottom: -0.7, near: 0.1, far: 3 });
const hemi = new THREE.HemisphereLight(0xe6eeff, 0x1a1e2e, 0.85);
const rimLight = new THREE.SpotLight(0x9d72ff, 2.6, 3.2, Math.PI / 3, 0.45, 1.1);
rimLight.position.set(-0.68, 0.98, -0.55);
const warmFill = new THREE.PointLight(0x60a5fa, 1.25, 2.4);
warmFill.position.set(0.50, 0.48, 0.48);
scene.add(sun, hemi, rimLight, warmFill);

// Studio floor grid + workbench with UR5e Brushed Silver, Metallic Purple & Cobalt Blue Clearcoat Shaders
const MAT = {
  alu: new THREE.MeshPhysicalMaterial({ color: 0xe3e8f0, metalness: 0.91, roughness: 0.17, clearcoat: 0.65, clearcoatRoughness: 0.12 }),
  urSilver: new THREE.MeshPhysicalMaterial({ color: 0xdce2ec, metalness: 0.89, roughness: 0.19, clearcoat: 0.70, clearcoatRoughness: 0.14 }),
  urBlueCap: new THREE.MeshPhysicalMaterial({ color: 0x3b82f6, metalness: 0.68, roughness: 0.20, clearcoat: 0.85, clearcoatRoughness: 0.10 }),
  urPurpleCap: new THREE.MeshPhysicalMaterial({ color: 0x9d72ff, metalness: 0.68, roughness: 0.20, clearcoat: 0.85, clearcoatRoughness: 0.10 }),
  carbon: new THREE.MeshPhysicalMaterial({ color: 0x1e2330, metalness: 0.70, roughness: 0.26, clearcoat: 0.72, clearcoatRoughness: 0.14 }),
  dark: new THREE.MeshPhysicalMaterial({ color: 0x181c26, metalness: 0.62, roughness: 0.30, clearcoat: 0.45 }),
  steel: new THREE.MeshPhysicalMaterial({ color: 0xc4ccd8, metalness: 0.95, roughness: 0.14, clearcoat: 0.60 }),
  copper: new THREE.MeshPhysicalMaterial({ color: 0xb794f4, metalness: 0.86, roughness: 0.20, clearcoat: 0.50 }),
  rubber: new THREE.MeshStandardMaterial({ color: 0x14171f, metalness: 0.08, roughness: 0.85 }),
  bench: new THREE.MeshPhysicalMaterial({ color: 0x161a26, metalness: 0.45, roughness: 0.32, clearcoat: 0.65, clearcoatRoughness: 0.18 }),
  white: new THREE.MeshPhysicalMaterial({ color: 0xeef2f8, metalness: 0.15, roughness: 0.28, clearcoat: 0.55 }),
  slot: new THREE.MeshStandardMaterial({ color: 0x090b10, roughness: 0.45 }),
  red: new THREE.MeshPhysicalMaterial({ color: 0x9d72ff, metalness: 0.55, roughness: 0.22, clearcoat: 0.65 }),
  green: new THREE.MeshPhysicalMaterial({ color: 0x38bdf8, metalness: 0.52, roughness: 0.22, clearcoat: 0.65 }),
  watch: new THREE.MeshPhysicalMaterial({ color: 0xdce2ec, metalness: 0.92, roughness: 0.16, clearcoat: 0.70 }),
  watchScreen: new THREE.MeshStandardMaterial({ color: 0x060b16, emissive: 0x60a5fa, emissiveIntensity: 0.85, roughness: 0.08 })
};

const normalMat = new THREE.MeshNormalMaterial();
const depthMat = new THREE.MeshDepthMaterial({ depthPacking: THREE.BasicDepthPacking });

// Tag every mesh with semantic & instance colors + rich hover metadata for Omniverse Replicator AOV passes
const trackedObjects = []; // { label, semClass, instId, obj3d, color }
function tagSemantic(rootObj, semHex, instHex, meta = null) {
  const semMat = new THREE.MeshBasicMaterial({ color: semHex });
  const instMat = new THREE.MeshBasicMaterial({ color: instHex });
  if (meta) rootObj.userData.hoverMeta = meta;
  rootObj.traverse(o => {
    if (o.isMesh && !o.userData.isHelper) {
      if (!o.userData.rtxMat) o.userData.rtxMat = o.material;
      o.userData.semMat = semMat;
      o.userData.instMat = instMat;
      if (meta && !o.userData.hoverMeta) o.userData.hoverMeta = meta;
    }
  });
}

// Build an interactive 3D Coordinate & Yaw/Pitch/Roll Orientation Gizmo attached to each gripper
function makeOrientationGizmo(dof) {
  const grp = new THREE.Group();
  grp.userData.isHelper = true;

  const makeMat = (hex, op = 0.72) => new THREE.MeshBasicMaterial({ color: hex, transparent: true, opacity: op, depthWrite: false });
  const yawMat = makeMat(0x60a5fa, 0.72);   // Cobalt Blue = Yaw (rotation around vertical Z)
  const pitchMat = makeMat(0x34d399, 0.72); // Emerald Green = Pitch (nose tilt up/down)
  const rollMat = makeMat(0xc084fc, 0.78);  // Metallic Purple = Roll (wrist twist around tool approach axis)
  const xMat = makeMat(0xf43f5e, 0.75);     // Red = X Reach

  // Yaw Horizontal Ring (in XY world / XZ Three.js plane around gripper)
  const yawRing = new THREE.Mesh(new THREE.TorusGeometry(0.044, 0.0014, 12, 48), yawMat);
  yawRing.rotation.x = Math.PI / 2;
  yawRing.userData.isHelper = true;

  // Pitch Tilt Arc (vertical plane)
  const pitchRing = new THREE.Mesh(new THREE.TorusGeometry(0.039, 0.0014, 12, 48), pitchMat);
  pitchRing.userData.isHelper = true;

  // Roll Wrist-Twist Ring (perpendicular to tool approach Z axis)
  const rollRing = new THREE.Mesh(new THREE.TorusGeometry(0.032, 0.0016, 12, 48), rollMat);
  rollRing.position.z = -0.015;
  rollRing.userData.isHelper = true;

  // Forward Tool Approach Arrow (+Z tool axis)
  const approachShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0014, 0.0014, 0.048, 12), rollMat);
  approachShaft.rotation.x = Math.PI / 2;
  approachShaft.position.z = 0.024;
  approachShaft.userData.isHelper = true;
  const approachCone = new THREE.Mesh(new THREE.ConeGeometry(0.0042, 0.011, 14), rollMat);
  approachCone.rotation.x = Math.PI / 2;
  approachCone.position.z = 0.050;
  approachCone.userData.isHelper = true;

  // Lateral Jaw-Closing Axis (+X tool axis)
  const xShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0012, 0.0012, 0.042, 10), xMat);
  xShaft.rotation.z = -Math.PI / 2;
  xShaft.position.x = 0.021;
  xShaft.userData.isHelper = true;

  grp.add(yawRing, pitchRing, rollRing, approachShaft, approachCone, xShaft);
  grp.userData.setHighlight = (axis) => {
    yawMat.opacity = axis === "yaw" || axis === "z" ? 1.0 : 0.48;
    yawRing.scale.setScalar(axis === "yaw" ? 1.18 : 1.0);
    pitchMat.opacity = axis === "pitch" || axis === "y" ? 1.0 : 0.48;
    pitchRing.scale.setScalar(axis === "pitch" ? 1.18 : 1.0);
    rollMat.opacity = axis === "roll" ? 1.0 : 0.56;
    rollRing.scale.setScalar(axis === "roll" ? 1.22 : 1.0);
    xMat.opacity = axis === "x" ? 1.0 : 0.50;
  };
  return grp;
}

const shadow = (m) => { m.castShadow = true; m.receiveShadow = true; return m; };
const mesh = (geo, mat) => shadow(new THREE.Mesh(geo, mat));

const bench = mesh(new THREE.BoxGeometry(1.2, 0.03, 1.6), MAT.bench);
bench.position.set(0.12, -0.015, 0);
tagSemantic(bench, 0x1b2230, 0x1b2230, {
  title: "Optical Calibration Workbench",
  sem: "workbench_surface (Class 0)",
  inst: "/World/Workbench#01",
  desc: "Precision anodized optical table ground plane (Z = 0.000 m) with 50 mm metric calibration grid."
});
scene.add(bench);

const gridHelper = new THREE.GridHelper(1.5, 30, 0x3b486b, 0x1e2538);
gridHelper.position.set(0.12, 0.001, 0);
gridHelper.userData.isHelper = true;
scene.add(gridHelper);

// Detailed Industrial UR5e Control Cabinet & Teach Pendant Dock (replaces primitive yellow box)
{
  const box = new THREE.Group();
  const chassis = mesh(new THREE.BoxGeometry(0.115, 0.064, 0.082), MAT.carbon);
  chassis.position.y = 0.032;
  const trim = mesh(new THREE.BoxGeometry(0.119, 0.010, 0.086), MAT.urSilver);
  trim.position.y = 0.065;
  const topCap = mesh(new THREE.BoxGeometry(0.116, 0.006, 0.083), MAT.urPurpleCap);
  topCap.position.y = 0.071;
  const screen = mesh(new THREE.BoxGeometry(0.056, 0.003, 0.042), MAT.watchScreen);
  screen.position.set(0.015, 0.0745, 0);
  const estopBase = mesh(new THREE.CylinderGeometry(0.009, 0.010, 0.005, 24), MAT.alu);
  estopBase.position.set(-0.036, 0.075, 0.018);
  const estopBtn = mesh(new THREE.CylinderGeometry(0.008, 0.006, 0.008, 24), new THREE.MeshPhysicalMaterial({ color: 0xf43f5e, metalness: 0.3, roughness: 0.25, clearcoat: 0.7 }));
  estopBtn.position.set(-0.036, 0.080, 0.018);
  box.add(chassis, trim, topCap, screen, estopBase, estopBtn);
  box.position.copy(T3([-0.15, 0.0, 0]));
  box.rotation.y = Math.PI / 2;
  tagSemantic(box, 0x9d72ff, 0xb99aff, {
    title: "UR5e Control Cabinet & E-Stop Dock",
    sem: "ur5e_controller (Class 6)",
    inst: "/World/UR5e_ControlBox#60",
    desc: "Real-time 500 Hz safety controller, IEEE-1588 PTP clock master, and hardware E-stop console."
  });
  scene.add(box);
  trackedObjects.push({ label: "ur5e_controller#60", clsId: 6, obj: box, color: "#9d72ff" });
}

function placeBetween(obj, a, b) {
  const A = T3(a), B = T3(b), dir = B.clone().sub(A), len = dir.length();
  obj.position.copy(A.clone().add(B).multiplyScalar(0.5));
  obj.quaternion.setFromUnitVectors(UP, dir.normalize());
  obj.scale.set(1, Math.max(len, 1e-4), 1);
}
function quatFromR(R) {
  const x = T3([R[0], R[3], R[6]]), y = T3([R[1], R[4], R[7]]), z = T3([R[2], R[5], R[8]]);
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
}

function makeGripper(ghost = false) {
  const g = new THREE.Group();
  const ghostMat = new THREE.MeshBasicMaterial({ color: 0x9d72ff, transparent: true, opacity: 0.24, depthWrite: false });
  const m = (geo, mat) => {
    const ms = ghost ? new THREE.Mesh(geo, ghostMat) : mesh(geo, mat);
    if (ghost) ms.userData.isHelper = true;
    return ms;
  };
  // UR5e Built-in 6-Axis Force/Torque Sensor Ring + ISO 9409-1-50-4-M6 Tool Flange
  const ftSensor = m(new THREE.CylinderGeometry(0.0175, 0.0175, 0.009, 36), MAT.copper);
  ftSensor.rotation.x = Math.PI / 2; ftSensor.position.z = -0.058; g.add(ftSensor);
  const flange = m(new THREE.CylinderGeometry(0.016, 0.0165, 0.008, 36), MAT.steel);
  flange.rotation.x = Math.PI / 2; flange.position.z = -0.050; g.add(flange);

  // Robotiq 2F-85 / Schunk Parallel Servo Gripper Housing
  const body = m(new THREE.BoxGeometry(0.072, 0.028, 0.026), MAT.carbon);
  body.position.z = -0.037; g.add(body);
  const bezel = m(new THREE.BoxGeometry(0.074, 0.011, 0.028), MAT.urSilver);
  bezel.position.z = -0.037; g.add(bezel);
  const rail = m(new THREE.BoxGeometry(0.076, 0.009, 0.007), MAT.steel);
  rail.position.set(0, 0, -0.023); g.add(rail);

  // Wrist Camera / Optical Rangefinder Module
  const tofLens = m(new THREE.CylinderGeometry(0.0045, 0.0045, 0.004, 24), MAT.watchScreen);
  tofLens.rotation.x = Math.PI / 2; tofLens.position.set(0, 0.011, -0.024); g.add(tofLens);

  const fingers = [-1, 1].map(s => {
    const f = new THREE.Group();
    const baseJaw = m(new THREE.BoxGeometry(0.011, 0.021, 0.016), MAT.steel);
    baseJaw.position.z = -0.014; f.add(baseJaw);
    const a = m(new THREE.BoxGeometry(0.008, 0.019, 0.034), MAT.urSilver);
    a.position.z = -0.003; f.add(a);
    const pad = m(new THREE.BoxGeometry(0.0035, 0.018, 0.022), MAT.rubber);
    pad.position.set(-s * 0.0055, 0, 0.003); f.add(pad);
    g.add(f); f.userData.s = s; return f;
  });
  g.userData.setOpen = (o) => fingers.forEach(f => { f.position.x = f.userData.s * (0.009 + 0.022 * o); });
  g.userData.setOpen(1);
  return g;
}

// Build a photorealistic Universal Robots UR5e brushed aluminum tubular link + authentic UR5e CAD mesh overlay
function makeUR5eLink(r, cadStlFile, capMat) {
  const grp = new THREE.Group();
  grp.userData.r = 1;
  // Brushed anodized aluminum UR5e cylindrical tube with dark seal rings at both ends
  const tube = mesh(new THREE.CylinderGeometry(r, r, 0.86, 40), MAT.urSilver);
  grp.add(tube);
  for (const y of [-0.43, 0.43]) {
    const seal = mesh(new THREE.CylinderGeometry(r * 1.07, r * 1.07, 0.055, 40), MAT.carbon);
    seal.position.y = y;
    grp.add(seal);
    const trimRing = mesh(new THREE.CylinderGeometry(r * 1.09, r * 1.09, 0.018, 40), capMat);
    trimRing.position.y = y * 0.90;
    grp.add(trimRing);
  }
  if (cadStlFile) {
    stlLoader.load(`./models/ur5e/${cadStlFile}`, (geo) => {
      geo.computeVertexNormals();
      geo.center();
      const cadMesh = mesh(geo, MAT.urSilver);
      // UR5e upperarm/forearm STL is ~0.425m long along Z; rotate Z->Y and normalize length to 0.92 so it scales cleanly with link
      cadMesh.rotation.x = Math.PI / 2;
      cadMesh.scale.set(0.22, 0.22, 1.85);
      grp.add(cadMesh);
    });
  }
  return grp;
}

const stlLoader = new STLLoader();
const gltfLoader = new GLTFLoader();

function makeRig(dof, offsetY, accentHex) {
  const rig = new THREE.Group();
  rig.position.copy(T3([0, offsetY, 0]));
  scene.add(rig);

  const armGroup = new THREE.Group();
  rig.add(armGroup);

  const capMat = dof === 4 ? MAT.urPurpleCap : MAT.urBlueCap;
  const ledRingMat = new THREE.MeshStandardMaterial({ color: accentHex, emissive: accentHex, emissiveIntensity: 0.75, roughness: 0.15 });

  // CNC-machined UR5e mounting base plate
  const plinth = mesh(new THREE.CylinderGeometry(0.054, 0.060, 0.010, 48), MAT.carbon); plinth.position.y = 0.005; armGroup.add(plinth);
  const flangeRing = mesh(new THREE.CylinderGeometry(0.046, 0.050, 0.012, 48), MAT.steel); flangeRing.position.y = 0.013; armGroup.add(flangeRing);
  for (let b = 0; b < 8; b++) {
    const ang = (b / 8) * Math.PI * 2;
    const bolt = mesh(new THREE.CylinderGeometry(0.0026, 0.0026, 0.006, 14), MAT.steel);
    bolt.position.set(Math.cos(ang) * 0.052, 0.011, Math.sin(ang) * 0.052);
    armGroup.add(bolt);
  }

  const armMeta = {
    title: dof === 4 ? "4-DOF Planar UR5e Arm" : "6-DOF Spherical-Wrist UR5e Arm",
    sem: dof === 4 ? "robot_arm_4dof (Class 1)" : "robot_arm_6dof (Class 2)",
    inst: `/World/UR5e_${dof}DOF#${dof === 4 ? 10 : 20}`,
    desc: dof === 4
      ? "4-DOF configuration (Yaw, Shoulder, Elbow, Wrist Pitch): constrained to a vertical radial plane without wrist roll."
      : "Full 6-DOF spherical wrist configuration (Yaw, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3): tracks arbitrary 3D orientation & roll."
  };

  // Official Universal Robots UR5e Base CAD Mesh (base.stl)
  stlLoader.load("./models/ur5e/base.stl", (geo) => {
    geo.computeVertexNormals();
    const cadBase = mesh(geo, MAT.urSilver);
    cadBase.scale.set(0.52, 0.52, 0.52);
    cadBase.rotation.x = -Math.PI / 2;
    cadBase.position.y = 0.012;
    tagSemantic(cadBase, dof === 4 ? 0x9d72ff : 0x3b82f6, dof === 4 ? 0xb99aff : 0x60a5fa, armMeta);
    armGroup.add(cadBase);
  });

  // Photorealistic UR5e brushed silver tubular links + official UR5e CAD shells
  const linkConfigs = [
    { r: 0.023, stl: null },
    { r: 0.0165, stl: "upperarm.stl" },
    { r: 0.014, stl: "forearm.stl" },
    { r: 0.0115, stl: null }
  ];
  const links = linkConfigs.map((cfg) => {
    const l = makeUR5eLink(cfg.r, cfg.stl, capMat);
    armGroup.add(l);
    return l;
  });

  // UR5e Modular Harmonic-Drive Joint Actuators with authentic UR5e CAD shells (shoulder.stl, wrist1.stl, wrist2.stl, wrist3.stl) + metallic blue/purple caps
  const jointStls = ["shoulder.stl", "shoulder.stl", "wrist1.stl", "wrist2.stl", "wrist2.stl", "wrist3.stl"];
  const jointNames = ["J1 Base Yaw", "J2 Shoulder Pitch", "J3 Elbow Pitch", "J4 Wrist 1 Roll", "J5 Wrist 2 Pitch", "J6 Tool Flange Roll"];
  const housings = Array.from({ length: 6 }, (_, i) => {
    const g = new THREE.Group();
    const big = i < 3;
    const rad = big ? 0.022 : 0.0165;
    const len = big ? 0.050 : 0.038;
    const jMeta = {
      title: `UR5e Actuator | ${jointNames[i]}`,
      sem: armMeta.sem,
      inst: `/World/UR5e_${dof === 4 ? "Left" : "Right"}/Joint_${i + 1}`,
      desc: `Harmonic-drive servo actuator with optical encoder and metallic ${dof === 4 ? "purple" : "cobalt blue"} dust cap.`
    };
    // UR5e silver actuator body
    const body = mesh(new THREE.CylinderGeometry(rad, rad, len * 0.82, 40), MAT.urSilver);
    g.add(body);
    // Dark harmonic-drive collar band
    const collar = mesh(new THREE.CylinderGeometry(rad * 1.03, rad * 1.03, len * 0.28, 40), MAT.carbon);
    g.add(collar);
    // Illuminated status ring
    const ledTorus = mesh(new THREE.TorusGeometry(rad * 1.03, 0.0014, 12, 36), ledRingMat);
    ledTorus.rotation.x = Math.PI / 2;
    ledTorus.position.y = len * 0.25;
    g.add(ledTorus);
    // Signature UR5e curved metallic end-caps (Metallic Purple on Left Rig, Metallic Cobalt Blue on Right Rig)
    for (const s of [-1, 1]) {
      const cap = mesh(new THREE.CylinderGeometry(rad * 0.86, rad * 0.99, 0.007, 36), capMat);
      cap.position.y = s * (len * 0.45);
      g.add(cap);
    }
    tagSemantic(g, dof === 4 ? 0x9d72ff : 0x3b82f6, dof === 4 ? 0xb99aff : 0x60a5fa, jMeta);
    // Attach real UR5e joint CAD shell
    const stlName = jointStls[i];
    if (stlName) {
      stlLoader.load(`./models/ur5e/${stlName}`, (geo) => {
        geo.computeVertexNormals();
        geo.center();
        const cadJ = mesh(geo, MAT.urSilver);
        const sc = big ? 0.34 : 0.28;
        cadJ.scale.set(sc, sc, sc);
        tagSemantic(cadJ, dof === 4 ? 0x9d72ff : 0x3b82f6, dof === 4 ? 0xb99aff : 0x60a5fa, jMeta);
        g.add(cadJ);
      });
    }
    armGroup.add(g);
    return g;
  });

  const grip = makeGripper(false);
  tagSemantic(grip, dof === 4 ? 0x9d72ff : 0x3b82f6, dof === 4 ? 0xb99aff : 0x60a5fa, {
    title: `Robotiq 2F-85 Parallel Servo Gripper`,
    sem: armMeta.sem,
    inst: `/World/UR5e_${dof === 4 ? "Left" : "Right"}/Robotiq_2F85`,
    desc: "Parallel-jaw servo gripper with ATI 6-axis force/torque sensor ring and optical rangefinder."
  });
  armGroup.add(grip);

  const gizmo = makeOrientationGizmo(dof);
  rig.add(gizmo);

  // Forearm IMU Strap & Watch Sensor Mount (Available on both rigs so 6D Real/Watch Twin or 4D Rig can display it)
  const watchSensor = new THREE.Group();
  const strap = mesh(new THREE.CylinderGeometry(0.0175, 0.0175, 0.022, 32), new THREE.MeshPhysicalMaterial({ color: dof === 4 ? 0x6d28d9 : 0x1d4ed8, roughness: 0.45, clearcoat: 0.3 }));
  strap.rotation.z = Math.PI / 2;
  watchSensor.add(strap);
  const caseMesh = mesh(new THREE.BoxGeometry(0.028, 0.0095, 0.034), MAT.watch);
  caseMesh.position.y = 0.015;
  watchSensor.add(caseMesh);
  const screenMesh = mesh(new THREE.BoxGeometry(0.024, 0.002, 0.029), MAT.watchScreen);
  screenMesh.position.y = 0.0195;
  watchSensor.add(screenMesh);
  const crown = mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.004, 16), capMat);
  crown.rotation.z = Math.PI / 2;
  crown.position.set(0.015, 0.016, 0.006);
  watchSensor.add(crown);
  armGroup.add(watchSensor);

  tagSemantic(armGroup, dof === 4 ? 0x9d72ff : 0x3b82f6, dof === 4 ? 0xb99aff : 0x60a5fa, armMeta);
  tagSemantic(watchSensor, 0xe879f9, 0xf0abfc, {
    title: "Forearm 6-Axis Watch IMU Site",
    sem: "forearm_watch_imu (Sensor Site)",
    inst: `/World/UR5e_${dof === 4 ? "Left" : "Right"}/forearm_imu#15`,
    desc: "Streams 6-axis proper acceleration a = R^T(a_world + g) and angular velocity w for Wahba/SVD Sim-to-Real calibration."
  });

  const ghost = makeGripper(true); rig.add(ghost);
  const guideMat = new THREE.MeshBasicMaterial({ color: accentHex, transparent: true, opacity: 0.50 });
  const guideLine = new THREE.Mesh(new THREE.CylinderGeometry(0.0016, 0.0016, 1, 8), guideMat);
  guideLine.userData.isHelper = true;
  rig.add(guideLine);

  let plane = null;
  if (dof === 4) {
    plane = new THREE.Mesh(new THREE.PlaneGeometry(0.36, 0.3), new THREE.MeshBasicMaterial({ color: 0x9d72ff, transparent: true, opacity: 0.09, side: THREE.DoubleSide, depthWrite: false }));
    plane.userData.isHelper = true;
    rig.add(plane);
  }

  // Machined Workbench Fixture Pads (CNC Beveled Pedestals + Alignment Reticles)
  const pickPad = new THREE.Group();
  const pickBase = mesh(new THREE.CylinderGeometry(0.036, 0.040, 0.004, 40), MAT.carbon);
  const pickRing = mesh(new THREE.TorusGeometry(0.033, 0.0018, 12, 40), MAT.steel);
  pickRing.rotation.x = Math.PI / 2; pickRing.position.y = 0.0022;
  pickPad.add(pickBase, pickRing);
  pickPad.position.copy(T3([0.15, -0.075, 0.002])); rig.add(pickPad);

  const dropPad = new THREE.Group();
  const dropBase = mesh(new THREE.CylinderGeometry(0.038, 0.042, 0.0045, 40), MAT.carbon);
  const dropSurface = mesh(new THREE.CylinderGeometry(0.034, 0.034, 0.005, 40), MAT.green);
  const dropRing = mesh(new THREE.TorusGeometry(0.035, 0.002, 12, 40), MAT.urBlueCap);
  dropRing.rotation.x = Math.PI / 2; dropRing.position.y = 0.0026;
  dropPad.add(dropBase, dropSurface, dropRing);
  dropPad.position.copy(T3([0.15, 0.075, 0.0022])); rig.add(dropPad);
  tagSemantic(pickPad, 0x37474f, 0x455a64, {
    title: "CNC Pick Fixture Pedestal",
    sem: "fixture_pick_pad (Station 1)",
    inst: `/World/PickPad_${dof}D`,
    desc: "Source cradle at (x=0.150m, y=-0.075m) where the 21700 cell starts before Pick & Place."
  });
  tagSemantic(dropPad, 0x2563eb, 0x60a5fa, {
    title: "CNC Target Place Pedestal",
    sem: "fixture_place_pad (Station 1)",
    inst: `/World/PlacePad_${dof}D`,
    desc: "Destination target pad at (x=0.150m, y=+0.075m) with cobalt alignment reticle."
  });

  // High-Detail 21700 Industrial Lithium Power Cell (Brushed Cobalt Jacket + Crimp Groove + Cathode Cap + Insulator Ring)
  const battery = new THREE.Group();
  const cellSleeve = mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.048, 36), MAT.urBlueCap);
  cellSleeve.rotation.z = Math.PI / 2; battery.add(cellSleeve);
  const cellBand = mesh(new THREE.CylinderGeometry(0.0123, 0.0123, 0.014, 36), MAT.urSilver);
  cellBand.rotation.z = Math.PI / 2; battery.add(cellBand);
  const negCap = mesh(new THREE.CylinderGeometry(0.0118, 0.0118, 0.003, 32), MAT.steel);
  negCap.rotation.z = Math.PI / 2; negCap.position.x = -0.025; battery.add(negCap);
  const posCollar = mesh(new THREE.CylinderGeometry(0.0118, 0.0118, 0.003, 32), MAT.carbon);
  posCollar.rotation.z = Math.PI / 2; posCollar.position.x = 0.025; battery.add(posCollar);
  const tip = mesh(new THREE.CylinderGeometry(0.0055, 0.0055, 0.006, 24), MAT.steel);
  tip.rotation.z = Math.PI / 2; tip.position.x = 0.0285; battery.add(tip);
  rig.add(battery);
  tagSemantic(battery, 0x38bdf8, dof === 4 ? 0x7dd3fc : 0x0284c7, {
    title: "21700 Cylindrical Lithium Power Cell",
    sem: "battery_cell (Class 3)",
    inst: `/World/power_cell#${dof === 4 ? 30 : 31}`,
    desc: "Graspable manipulation target (Ø24 mm × 54 mm). Click 'Grasp' or 'Pick & Place' to manipulate."
  });

  // Industrial Charger Dock Station
  const charger = new THREE.Group(); rig.add(charger);
  const panel = mesh(new THREE.BoxGeometry(0.112, 0.102, 0.016), MAT.urSilver); charger.add(panel);
  const bezelFrame = mesh(new THREE.BoxGeometry(0.072, 0.040, 0.018), MAT.carbon); bezelFrame.position.z = 0.001; charger.add(bezelFrame);
  const slot = mesh(new THREE.BoxGeometry(0.062, 0.030, 0.019), MAT.slot); slot.position.z = 0.002; charger.add(slot);
  const ledMat = new THREE.MeshStandardMaterial({ color: 0x9d72ff, emissive: 0x9d72ff, emissiveIntensity: 0.9 });
  const led = mesh(new THREE.SphereGeometry(0.005, 16, 12), ledMat); led.position.set(0.042, 0.038, 0.010); charger.add(led);
  const stand = mesh(new THREE.CylinderGeometry(0.006, 0.007, 1, 20), MAT.steel); rig.add(stand);
  const chargerMeta = {
    title: "Tilted Side-Charger Insertion Dock",
    sem: "charger_dock (Class 4)",
    inst: `/World/charger_dock#${dof === 4 ? 40 : 41}`,
    desc: "Compound-angle slot requiring oblique approach & roll alignment (demonstrates 4-DOF planar lock vs 6-DOF spherical wrist)."
  };
  tagSemantic(charger, 0x60a5fa, dof === 4 ? 0x3b82f6 : 0x2563eb, chargerMeta);
  tagSemantic(stand, 0x60a5fa, 0x1d4ed8, chargerMeta);

  // Precision Industrial Process Valve Wheel
  const post = mesh(new THREE.CylinderGeometry(0.0075, 0.0085, 1, 20), MAT.steel); rig.add(post);
  const valve = new THREE.Group(); rig.add(valve);
  valve.add(mesh(new THREE.TorusGeometry(0.031, 0.0055, 20, 56), MAT.urPurpleCap));
  for (const r of [0, Math.PI / 3, (2 * Math.PI) / 3]) {
    const sp = mesh(new THREE.CylinderGeometry(0.0032, 0.0032, 0.060, 16), MAT.steel);
    sp.rotation.z = r;
    valve.add(sp);
  }
  const hub = mesh(new THREE.CylinderGeometry(0.0095, 0.0095, 0.014, 24), MAT.urSilver);
  hub.rotation.x = Math.PI / 2; valve.add(hub);
  const valveMeta = {
    title: "Industrial Process Valve Handwheel",
    sem: "valve_wheel (Class 5)",
    inst: `/World/valve_wheel#${dof === 4 ? 50 : 51}`,
    desc: "Requires horizontal approach (+X) followed by 90° continuous J6 tool-flange roll rotation."
  };
  tagSemantic(valve, 0x9d72ff, dof === 4 ? 0xc084fc : 0x7c3aed, valveMeta);
  tagSemantic(post, 0x9d72ff, 0x6d28d9, valveMeta);

  trackedObjects.push(
    { label: `ur5e_arm_${dof}dof#${dof === 4 ? 10 : 20}`, clsId: dof === 4 ? 1 : 2, obj: armGroup, color: dof === 4 ? "#9d72ff" : "#60a5fa" },
    { label: `power_cell#${dof === 4 ? 30 : 31}`, clsId: 3, obj: battery, color: "#38bdf8" }
  );

  return { rig, armGroup, links, housings, grip, gizmo, watchSensor, ghost, guideLine, guideMat, plane, pickPad, dropPad, battery, charger, ledMat, stand, post, valve };
}

const rigs = {
  4: makeRig(4, -0.18, 0x9d72ff),
  6: makeRig(6, 0.18, 0x3b82f6)
};

const poseObj = (obj, p, R) => { obj.position.copy(T3(p)); obj.quaternion.copy(quatFromR(R)); };

function updateRig(dof, fk, props) {
  const r = rigs[dof], z = [fk.R[2], fk.R[5], fk.R[8]];
  const flangeAt = [fk.tip[0] - z[0] * 0.056, fk.tip[1] - z[1] * 0.056, fk.tip[2] - z[2] * 0.056];
  const ends = [fk.pts[0], fk.pts[1], fk.pts[2], fk.pts[3], flangeAt];
  r.links.forEach((l, i) => { placeBetween(l, ends[i], ends[i + 1]); l.scale.x = l.scale.z = l.userData.r; });
  r.housings.forEach((h, i) => {
    const j = fk.joints[i];
    h.visible = !!j;
    if (j) {
      h.position.copy(T3(j.p));
      h.quaternion.setFromUnitVectors(UP, T3(j.axis).normalize());
    }
  });
  poseObj(r.grip, fk.tip, fk.R);
  r.grip.userData.setOpen(props.open);
  if (r.gizmo) {
    r.gizmo.visible = st.showGizmos && st.aov === "rtx";
    poseObj(r.gizmo, fk.tip, fk.R);
    r.gizmo.userData.setHighlight(st.highlightAxis);
  }

  if (r.watchSensor) {
    r.watchSensor.visible = !!props.watchSite;
    if (props.watchSite) poseObj(r.watchSensor, props.watchSite.p, props.watchSite.R);
  }

  const isRtx = st.aov === "rtx";
  if (props.target && isRtx) {
    r.ghost.visible = true; poseObj(r.ghost, props.target.p, props.target.R); r.ghost.userData.setOpen(props.target.open ?? 1);
    r.guideLine.visible = true; placeBetween(r.guideLine, fk.tip, props.target.p); r.guideLine.scale.x = r.guideLine.scale.z = 1;
  } else {
    r.ghost.visible = false; r.guideLine.visible = false;
  }
  if (r.plane) {
    r.plane.visible = isRtx && st.rigMode === "4d_vs_6d";
    r.plane.position.copy(T3([0.12 * Math.cos(fk.yaw), 0.12 * Math.sin(fk.yaw), 0.15]));
    r.plane.rotation.set(0, fk.yaw, 0);
  }
  const stIdx = props.station ?? 0;
  r.pickPad.visible = r.dropPad.visible = (stIdx === 0);
  r.battery.visible = (stIdx === 0 || stIdx === 1);
  r.charger.visible = r.stand.visible = (stIdx === 1);
  r.valve.visible = r.post.visible = (stIdx === 2);

  const b = props.battery;
  if (b.held) { poseObj(r.battery, fk.tip, fk.R); r.battery.rotateZ(Math.PI / 2); r.battery.translateZ(0.004); }
  else if (b.inserted) { poseObj(r.battery, b.inserted.p, b.inserted.R); r.battery.rotateZ(Math.PI / 2); }
  else { poseObj(r.battery, b.rest.p, b.rest.R); r.battery.rotateZ(Math.PI / 2); }

  const c = props.charger, cz = [c.R[2], c.R[5], c.R[8]], cp = [c.p[0] + cz[0] * 0.024, c.p[1] + cz[1] * 0.024, c.p[2] + cz[2] * 0.024];
  poseObj(r.charger, cp, c.R);
  placeBetween(r.stand, [cp[0], cp[1] + 0.03, 0], [cp[0], cp[1] + 0.03, cp[2] - 0.02]); r.stand.scale.x = r.stand.scale.z = 1;
  r.ledMat.color.set(c.lit ? 0x34a853 : 0xea4335); r.ledMat.emissive.set(c.lit ? 0x34a853 : 0xea4335);

  const v = props.valve;
  r.valve.position.copy(T3([v.p[0] + 0.014, v.p[1], v.p[2]]));
  r.valve.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0)); r.valve.rotateZ(-v.angle);
  placeBetween(r.post, [v.p[0] + 0.03, v.p[1], 0], [v.p[0] + 0.03, v.p[1], v.p[2]]); r.post.scale.x = r.post.scale.z = 1;
}

const AOV_EXPLAIN = {
  rtx: {
    title: "PBR Lit Pass",
    desc: "",
    legend: `
      <span class="swatch"><i style="background:#60a5fa"></i>L/R = Side</span>
      <span class="swatch"><i style="background:#34d399"></i>U/D = Reach+Height</span>
      <span class="swatch"><i style="background:#c084fc"></i>Twist = Roll</span>
    `
  },
  depth: {
    title: "Depth Z Pass (m)",
    desc: "",
    legend: `
      <span class="swatch"><i style="background:#f8fafc"></i>Near</span>
      <span class="swatch"><i style="background:#64748b"></i>Mid</span>
      <span class="swatch"><i style="background:#0f172a"></i>Far</span>
    `
  },
  normal: {
    title: "Surface Normals (XYZ)",
    desc: "",
    legend: `
      <span class="swatch"><i style="background:#f43f5e"></i>+X</span>
      <span class="swatch"><i style="background:#34d399"></i>+Y</span>
      <span class="swatch"><i style="background:#60a5fa"></i>+Z</span>
    `
  },
  semantic: {
    title: "Semantic Classes",
    desc: "",
    legend: `
      <span class="swatch"><i style="background:#9d72ff"></i>Left</span>
      <span class="swatch"><i style="background:#3b82f6"></i>Right</span>
      <span class="swatch"><i style="background:#38bdf8"></i>Cell</span>
      <span class="swatch"><i style="background:#60a5fa"></i>Dock</span>
      <span class="swatch"><i style="background:#c084fc"></i>Valve</span>
    `
  },
  instance: {
    title: "OpenUSD Instance IDs",
    desc: "",
    legend: `
      <span class="swatch"><i style="background:#b99aff"></i>#10</span>
      <span class="swatch"><i style="background:#60a5fa"></i>#20</span>
      <span class="swatch"><i style="background:#f0abfc"></i>#15</span>
      <span class="swatch"><i style="background:#7dd3fc"></i>#30</span>
    `
  }
};

const STATION_GUIDES = [
  { title: "Task 1: Pick & Place Cell", desc: "" },
  { title: "Task 2: Slot into Charger", desc: "" },
  { title: "Task 3: Turn Valve 90°", desc: "" }
];

function renderAOVBanner(mode) {
  const info = AOV_EXPLAIN[mode] || AOV_EXPLAIN.rtx;
  const stGuide = STATION_GUIDES[st.station] || STATION_GUIDES[0];
  const titleText = mode === "rtx" ? stGuide.title : info.title;
  if ($("aov-banner-title")) {
    $("aov-banner-title").innerHTML = `<span>${titleText}</span>`;
  }
  if ($("aov-banner-legend")) $("aov-banner-legend").innerHTML = info.legend;
}

function applyAOVPass(mode) {
  st.aov = mode;
  gridHelper.visible = (mode === "rtx");
  if (mode === "normal") {
    scene.overrideMaterial = normalMat;
  } else if (mode === "depth") {
    scene.overrideMaterial = depthMat;
  } else {
    scene.overrideMaterial = null;
    scene.traverse(o => {
      if (o.isMesh && !o.userData.isHelper && o.userData.rtxMat) {
        o.material = mode === "semantic" ? o.userData.semMat : mode === "instance" ? o.userData.instMat : o.userData.rtxMat;
      }
    });
  }
  const descMap = {
    rtx: "Pass: WebGL2 PBR Metallic-Roughness + Soft Shadows",
    depth: "Pass: Linear Metric Depth Z Shader",
    normal: "Pass: View-Space Surface Normals Shader",
    semantic: "Pass: Per-Class Semantic ID Shader (Classes 1..6)",
    instance: "Pass: Per-Prim Instance ID Shader"
  };
  $("aov-desc").textContent = descMap[mode] || descMap.rtx;
  renderAOVBanner(mode);
}

// ── 6. 2D/3D Replicator Bounding Box Projection ──
function computeBBoxes() {
  const W = bboxCv.width, H = bboxCv.height;
  bboxCtx.clearRect(0, 0, W, H);
  const anns = [];
  const box3 = new THREE.Box3();
  const corners = Array.from({ length: 8 }, () => new THREE.Vector3());

  for (const item of trackedObjects) {
    if (!item.obj.visible) continue;
    box3.setFromObject(item.obj);
    if (box3.isEmpty()) continue;
    const min = box3.min, max = box3.max;
    let x0 = W, y0 = H, x1 = 0, y1 = 0, behind = false;
    let idx = 0;
    for (const vx of [min.x, max.x]) for (const vy of [min.y, max.y]) for (const vz of [min.z, max.z]) {
      const p = corners[idx++].set(vx, vy, vz).project(camera);
      if (p.z > 1) behind = true;
      const px = (p.x * 0.5 + 0.5) * W;
      const py = (-p.y * 0.5 + 0.5) * H;
      x0 = Math.min(x0, px); y0 = Math.min(y0, py);
      x1 = Math.max(x1, px); y1 = Math.max(y1, py);
    }
    if (behind || x1 <= x0 || y1 <= y0) continue;
    x0 = Math.max(0, Math.min(W, x0)); y0 = Math.max(0, Math.min(H, y0));
    x1 = Math.max(0, Math.min(W, x1)); y1 = Math.max(0, Math.min(H, y1));
    const bw = x1 - x0, bh = y1 - y0;
    if (bw < 6 || bh < 6) continue;
    const center3d = box3.getCenter(new THREE.Vector3());
    const depthM = +(center3d.distanceTo(camera.position)).toFixed(3);
    anns.push({
      label: item.label,
      category_id: item.clsId,
      bbox_2d: [+x0.toFixed(1), +y0.toFixed(1), +bw.toFixed(1), +bh.toFixed(1)],
      bbox_3d_extents: [+(max.x - min.x).toFixed(3), +(max.y - min.y).toFixed(3), +(max.z - min.z).toFixed(3)],
      depth_m: depthM
    });

    if (st.showBBoxes) {
      bboxCtx.strokeStyle = item.color;
      bboxCtx.lineWidth = 1.6;
      bboxCtx.setLineDash([5, 3]);
      bboxCtx.strokeRect(x0, y0, bw, bh);
      bboxCtx.setLineDash([]);
      bboxCtx.fillStyle = "rgba(11,14,20,0.82)";
      const tag = `${item.label} · ${depthM.toFixed(2)}m`;
      bboxCtx.font = "bold 10.5px 'Roboto Mono', monospace";
      const tw = bboxCtx.measureText(tag).width + 8;
      bboxCtx.fillRect(x0, Math.max(0, y0 - 16), tw, 15);
      bboxCtx.fillStyle = item.color;
      bboxCtx.fillText(tag, x0 + 4, Math.max(11, y0 - 4));
    }
  }
  return anns;
}

// ── 7. Mission Target & Evaluation Logic ──
function armTarget(dof) {
  const a = st.arm[dof], s = ST[st.station];
  if (s.id === "battery") {
    if (a.held) {
      if (a.dest === "pick") return { p: PICK_P, R: PICK_R, ypr: [-25, 0, 0] };
      return { p: PLACE_P, R: PLACE_R, ypr: [25, 0, 0] };
    }
    const bp = [a.battP[0], a.battP[1], 0.032];
    return { p: bp, R: a.battR, ypr: [Math.round(Math.atan2(a.battP[1], a.battP[0]) / D2R), 0, 0] };
  }
  if (s.id === "valve") {
    const g = st.closed || a.gripping;
    return { p: s.p, R: g ? s.R : s.grip, ypr: g ? s.ypr : s.yprGrip };
  }
  return { p: s.p, R: s.R, ypr: s.ypr };
}

function setClosed(v, lift = true, forceAction = null) {
  const next = !!v;
  if (st.closed !== next) st.recoilUntil = performance.now() + 340;
  st.closed = next;
  $("btn-grab").innerHTML = next
    ? `<svg class="ico sm" viewBox="0 0 24 24"><path d="M5 12h14"/></svg><span>Release</span>`
    : `<svg class="ico sm" viewBox="0 0 24 24"><path d="M7 4v8l3 3v5M17 4v8l-3 3v5M4 8h4M16 8h4"/></svg><span>Grasp</span>`;
  $("btn-grab").classList.toggle("on", next);
  send({ t: "grip", g: next ? 1 : 0 });

  // Ensure kinematic joint state matches current st.cmd before evaluating grasp/release proximity
  const cmdR = poseR(st.cmd.yaw, st.cmd.pitch, st.cmd.roll);
  const cmdP = [st.cmd.x, st.cmd.y, st.cmd.z];
  st.q4 = ik4(cmdP, cmdR).q;
  st.q6 = ik6(cmdP, cmdR, st.q6).q;

  const fks = { 4: fk4(st.q4), 6: fk6(st.q6) };
  const cl = (val, lo, hi) => Math.max(lo, Math.min(hi, val));
  let releasedAny = false, grabbedAny = false;
  for (const dof of [4, 6]) {
    const a = st.arm[dof], f = fks[dof];
    if (next) {
      const dxy = Math.hypot(f.tip[0] - a.battP[0], f.tip[1] - a.battP[1]);
      const dz = Math.abs(f.tip[2] - 0.032);
      if (forceAction === "grasp_cell" || (!a.held && !a.inserted && dxy < 0.105 && dz < 0.16)) {
        a.held = true;
        a.inserted = null;
        a.dest = a.battP[1] > 0.02 ? "pick" : "place";
        grabbedAny = true;
        if (st.station === 0) a.done[0] = false;
      }
    } else if (a.held) {
      a.held = false;
      releasedAny = true;
      const dPlace = Math.hypot(f.tip[0] - PLACE_P[0], f.tip[1] - PLACE_P[1]);
      const dPick = Math.hypot(f.tip[0] - PICK_P[0], f.tip[1] - PICK_P[1]);
      const dCharger = Math.hypot(f.tip[0] - ST[1].p[0], f.tip[1] - ST[1].p[1], f.tip[2] - ST[1].p[2]);
      if (st.station === 0 && (forceAction === "release_cell" || dPlace < 0.095)) {
        const destCoord = a.dest === "pick" && dPick < dPlace ? PICK_P : PLACE_P;
        const destRot = a.dest === "pick" && dPick < dPlace ? PICK_R : PLACE_R;
        a.battP = [destCoord[0], destCoord[1], 0.013];
        a.battR = destRot;
        a.done[0] = true;
      } else if (st.station === 0 && dPick < 0.095 && a.dest === "pick") {
        a.battP = [PICK_P[0], PICK_P[1], 0.013]; a.battR = PICK_R; a.done[0] = true;
      } else if (st.station === 1 && (forceAction === "slot_charger" || (dCharger < POS_TOL * 1.65 && oriError(f.R, ST[1].R) < ORI_TOL * 1.65))) {
        a.inserted = { p: [...ST[1].p], R: [...ST[1].R] }; a.done[1] = true;
      } else {
        a.battP = [cl(f.tip[0], 0.06, 0.22), cl(f.tip[1], -0.14, 0.14), 0.013];
        a.battR = poseR(st.cmd.yaw, 0, 0);
        a.inserted = null;
      }
    }
  }
  if (grabbedAny && st.station === 0 && lift && !st.tween) st.cmd.z = Math.max(st.cmd.z, 0.065);
  if (releasedAny && lift && !st.tween) st.cmd.z = 0.130;
}

const angleBetween = (a, b) => Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))) / D2R;

function evaluateArm(dof, f, dt) {
  const a = st.arm[dof], s = ST[st.station], i = st.station, closed = st.closed;
  const tgt = armTarget(dof);
  const pos = Math.hypot(f.tip[0] - tgt.p[0], f.tip[1] - tgt.p[1], f.tip[2] - tgt.p[2]);
  const posMm = Math.round(pos * 1000);
  if (s.id === "battery") {
    const oriDeg = Math.round(oriError(f.R, tgt.R));
    const aligned = pos < POS_TOL * 1.3 && oriDeg < ORI_TOL * 1.3;
    if (!a.held) {
      if (aligned && closed) { a.held = true; a.done[0] = false; return { msg: "Cell Grasped · Move to Target Pad -> Release", cls: "warn", aligned: false, posMm, oriDeg }; }
      if (a.done[0]) return { msg: "Pick & Place Complete", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 };
      return { msg: aligned ? "Aligned Over Cell -> Click Grasp" : `To Cell · ${posMm} mm · ${oriDeg}°`, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
    }
    return { msg: aligned ? "Over Target Pad -> Click Release" : `Carrying Cell · ${posMm} mm to Pad`, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
  }
  if (a.done[i]) return { msg: "Station Task Complete", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 };
  if (s.id === "valve") {
    const z = [f.R[2], f.R[5], f.R[8]], x = [f.R[0], f.R[3], f.R[6]], off = angleBetween(z, [1, 0, 0]);
    const oriDeg = Math.round(off);
    const aligned = pos < POS_TOL && off < ORI_TOL;
    a.gripping = aligned && closed;
    const ang = Math.abs(Math.atan2(x[1], -x[2])) / D2R, folded = ang > 90 ? 180 - ang : ang;
    if (a.gripping) a.valve = Math.max(a.valve, folded);
    if (a.gripping && a.valve >= 75) { a.done[i] = true; return { msg: "Valve Rotated 90°", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 }; }
    const msg = !aligned ? `To Valve · ${posMm} mm · ${oriDeg}°` : !closed ? "Aligned -> Click Grasp" : dof === 4 ? `Grasping · 4-DOF No Wrist Roll (${a.valve.toFixed(0)}°)` : `Rotating Valve… ${a.valve.toFixed(0)}°/90°`;
    return { msg, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
  }
  const oriDeg = Math.round(oriError(f.R, s.R));
  const aligned = pos < POS_TOL && oriDeg < ORI_TOL;
  if (aligned && !closed && a.inserted) { a.done[i] = true; return { msg: "Charger Dock Slotted", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 }; }
  const msg = aligned ? "Aligned in Slot -> Click Release" : dof === 4 ? `4-DOF Planar Lock · ${oriDeg}° Off` : `To Charger · ${posMm} mm · ${oriDeg}°`;
  return { msg, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
}

// ── 8. Phone / Watch WebSocket Relay Wiring ──
const phones = new Map();
$("room-pill").textContent = `Room ${ROOM}`;
const qrEl = document.querySelector('[data-live="qr"]');
const urlEl = document.querySelector('[data-live="url"]');
const statusEl = document.querySelector('[data-live="status"]');
if (qrEl) qrEl.innerHTML = qrSVG();
if (urlEl) { urlEl.href = phoneUrl(); urlEl.textContent = "Open Controller ↗"; }
if ($("sms-watch")) {
  $("sms-watch").href = `sms:&body=${encodeURIComponent(phoneUrl())}`;
}
if ($("imu-push-url")) {
  const pushUrl = `${location.origin}/api/room/${ROOM}/imu`;
  $("imu-push-url").textContent = `/api/room/${ROOM}/imu (Copy)`;
  $("imu-push-url").onclick = () => {
    navigator.clipboard?.writeText(pushUrl);
    $("watch-status").textContent = `Copied HTTP Push URL: ${pushUrl}`;
  };
}

const addPhone = (id) => {
  if (!phones.has(id)) {
    phones.set(id, { team: 6, pose: null, joints: {}, t: 0, assigned: [], name: `Watch ${phones.size + 1}`, imu: null });
  }
};

function assignAll() {
  const mode = st.drive === "joints" ? "joints" : "pose";
  const ids = [...phones.keys()];
  const J = JOINTS[6].length, n = Math.min(ids.length, J);
  ids.forEach((id, k) => {
    const mine = []; for (let j = 0; j < J; j++) if (n && j % n === k % n) mine.push(j);
    phones.get(id).team = 6;
    phones.get(id).assigned = mine; phones.get(id).joints = {};
    send({ t: "assign", to: id, team: 6, mode, joints: mine.map(j => ({ i: j, n: JOINTS[6][j].n, lim: JOINTS[6][j].lim, v: Math.round(st.q6[j] / D2R) })) });
  });
  if ($("team-pill")) $("team-pill").textContent = `Watches: ${phones.size}`;
}

function pushState() {
  const s = ST[st.station], p = st.pilot && phones.get(st.pilot);
  send({ t: "state", slide: "teleop", mode: st.drive === "joints" ? "joints" : "pose", closed: st.closed, station: { n: st.station + 1, name: `${s.icon} ${s.name}` }, pilot: p ? { id: st.pilot, name: p.name } : null });
}

function renderPilots() {
  const box = $("pilot-list");
  if (st.drive !== "pilot") { box.innerHTML = ""; return; }
  const now = performance.now();
  const list = [...phones.entries()].sort((a, b) => (now - a[1].t) - (now - b[1].t)).slice(0, 8);
  box.innerHTML = list.length
    ? list.map(([id, p]) => `<button class="btn sm ${id === st.pilot ? "on" : ""}" data-pid="${id}">${id === st.pilot ? "[PILOT] " : ""}${p.name}</button>`).join("")
    : `<span class="mono" style="font-size:10.5px;color:var(--ink-3)">Scan QR on phone to join...</span>`;
  box.querySelectorAll("[data-pid]").forEach(b => b.addEventListener("click", () => {
    st.pilot = b.dataset.pid;
    const pl = phones.get(st.pilot);
    send({ t: "pilot", id: st.pilot, name: pl ? pl.name : "" });
    renderPilots(); pushState();
  }));
}

onLive((m) => {
  if (m.t === "status") {
    const n = (m.phones || []).length;
    statusEl.className = `pill ${m.status === "live" ? "ok" : "warn"}`;
    statusEl.textContent = m.status === "live" ? `LIVE · ${n} phone${n === 1 ? "" : "s"}` : m.status;
    const ids = new Set(m.phones || []);
    for (const id of [...phones.keys()]) if (!ids.has(id)) phones.delete(id);
    ids.forEach(addPhone); assignAll(); renderPilots();
    if (m.status === "live") pushState();
    return;
  }
  if (m.from && m.t !== "leave" && !phones.has(m.from)) { addPhone(m.from); assignAll(); pushState(); renderPilots(); }
  if (m.t === "join") { addPhone(m.from); assignAll(); pushState(); renderPilots(); }
  else if (m.t === "leave") { phones.delete(m.from); if (st.pilot === m.from) st.pilot = null; assignAll(); renderPilots(); }
  else if (m.t === "hello" && phones.has(m.from)) {
    phones.get(m.from).name = String(m.name || "").replace(/[<>&"]/g, "").slice(0, 24) || phones.get(m.from).name;
    renderPilots();
  } else if (m.t === "team" && phones.has(m.from)) {
    phones.get(m.from).team = m.team === 6 ? 6 : 4; assignAll();
  } else if (m.t === "pose" && phones.has(m.from)) {
    const p = phones.get(m.from), v = m.v || {};
    const num = (x, lo, hi, d) => Number.isFinite(+x) ? Math.max(lo, Math.min(hi, +x)) : d;
    p.g = v.g ? 1 : 0;
    if (v.grabToggle && (!st.pilot || st.pilot === m.from)) {
      if (v.gesture) st.lastGestureBanner = v.gesture;
      triggerSmartGraspOrRelease(v.gesture || "Watch GRASP/RELEASE");
    }
    if (v.snap && (!st.pilot || st.pilot === m.from)) snapToStation();
    if (v.recenter && (!st.pilot || st.pilot === m.from)) {
      const t = armTarget(st.arm[6].held ? 6 : 4);
      st.deskBasePos = [t.p[0], t.p[1], Math.max(0.08, t.p[2])];
      st.cmd.x = st.deskBasePos[0]; st.cmd.y = st.deskBasePos[1]; st.cmd.z = st.deskBasePos[2];
      st.cmd.yaw = 0; st.cmd.pitch = 0; st.cmd.roll = 0;
      $("watch-status").textContent = "Wrist Recentered: Desk anchor set to current hand pose";
    }
    p.pose = {
      vx: num(v.vx, -1, 1, 0), vy: num(v.vy, -1, 1, 0), vz: num(v.vz, -1, 1, 0),
      aim: v.aim === "tilt" ? "tilt" : "auto",
      gain: num(v.gain, 0.5, 6.0, st.studioGain),
      yaw: num(v.yaw, -180, 180, 0), pitch: num(v.pitch, -90, 90, 0), roll: num(v.roll, -90, 90, 0)
    };
    if (v.ax !== undefined || v.gx !== undefined) {
      p.imu = { ax: +v.ax || 0, ay: +v.ay || 0, az: +v.az || G0, gx: +v.gx || 0, gy: +v.gy || 0, gz: +v.gz || 0, tap: !!v.tap };
    }
    p.t = performance.now();
  } else if (m.t === "joints" && phones.has(m.from)) {
    const p = phones.get(m.from);
    for (const [k, v] of Object.entries(m.v || {})) if (p.assigned.includes(+k) && Number.isFinite(+v)) p.joints[k] = Math.max(-180, Math.min(180, +v));
    p.t = performance.now();
  }
});

function crowdPose() {
  const now = performance.now();
  let fresh = [...phones.entries()].filter(([, p]) => p.pose && now - p.t < 2500);
  if (st.drive === "pilot") fresh = fresh.filter(([id]) => id === st.pilot);
  if (!fresh.length) return null;
  const n = fresh.length, acc = { vx: 0, vy: 0, vz: 0, pitch: 0, roll: 0 };
  let sx = 0, sy = 0, g = 0, tilt = 0, gainSum = 0, liveImu = null;
  fresh.forEach(([, p]) => {
    for (const k in acc) acc[k] += p.pose[k];
    sx += Math.cos(p.pose.yaw * D2R); sy += Math.sin(p.pose.yaw * D2R);
    g += p.g || 0; tilt += p.pose.aim === "tilt" ? 1 : 0;
    gainSum += p.pose.gain || st.studioGain || 3.2;
    if (p.imu) liveImu = p.imu;
  });
  for (const k in acc) acc[k] /= n;
  return { ...acc, yaw: Math.atan2(sy, sx) / D2R, n, g: g / n >= 0.5, aim: tilt / n >= 0.5 ? "tilt" : "auto", gain: gainSum / n, liveImu };
}

// ── 9. UI Controls, Rig Comparison Mode & Trajectories ──
function setRigMode(mode) {
  st.rigMode = mode;
  if ($("rig-mode-bar")) {
    $("rig-mode-bar").querySelectorAll("button").forEach(b => b.classList.toggle("sel", b.dataset.rm === mode));
  }
  const isSingle = mode === "6d_single";
  const is4v6 = mode === "4d_vs_6d";
  rigs[4].rig.visible = !isSingle;
  rigs[6].rig.position.copy(T3([0, isSingle ? 0 : 0.18, 0]));
  if ($("badge-left")) $("badge-left").classList.toggle("hide", isSingle);
  if ($("vp-dock-bottom")) $("vp-dock-bottom").classList.toggle("single", isSingle);

  if ($("title-left")) {
    $("title-left").innerHTML = is4v6
      ? `<span style="color:var(--accent)">4-DOF Planar</span>`
      : `<span style="color:var(--accent)">6-DOF Ideal Sim</span>`;
  }
  if ($("title-right")) {
    $("title-right").innerHTML = isSingle
      ? `<span style="color:var(--blue)">6-DOF Spherical Wrist</span>`
      : is4v6
        ? `<span style="color:var(--blue)">6-DOF Spherical Wrist</span>`
        : `<span style="color:var(--blue)">6-DOF Real Twin</span>`;
  }
  if ($("kpi-err-lbl")) {
    $("kpi-err-lbl").textContent = is4v6 ? "4D/6D" : "Lag";
  }
  if ($("chart2-title")) {
    $("chart2-title").innerHTML = is4v6
      ? `3. <b style="color:var(--purple-hi)">4D/6D</b> &amp; <b style="color:#cbd5e1">Speed</b>`
      : `3. <b style="color:var(--purple-hi)">Lag</b> &amp; <b style="color:#cbd5e1">Speed</b>`;
  }
}
if ($("rig-mode-bar")) {
  $("rig-mode-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => setRigMode(b.dataset.rm)));
}

function setStation(i) {
  st.station = i;
  st.quinticPlan = null;
  if (i === 0) {
    if (!st.arm[4].held && !st.arm[6].held) setClosed(false);
  } else if (i === 1) {
    for (const d of [4, 6]) { st.arm[d].inserted = null; st.arm[d].held = true; }
    setClosed(true);
  } else if (i === 2) {
    setClosed(false);
  }
  $("st-bar").querySelectorAll("button").forEach(b => b.classList.toggle("sel", +b.dataset.st === i));
  renderAOVBanner(st.aov);
  pushState();
}

$("st-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => setStation(+b.dataset.st)));

function tweenTo(ypr, p, dur = 1100, then) {
  st.quinticPlan = null;
  st.tween = { from: { ...st.cmd }, to: { yaw: ypr[0], pitch: ypr[1], roll: ypr[2], x: p[0], y: p[1], z: p[2] }, t0: performance.now(), dur, then };
  st.holdCrowd = performance.now() + dur + 150;
}
function snapToStation() {
  const t = armTarget(st.arm[6].held ? 6 : 4);
  tweenTo(t.ypr, t.p, 520);
}
$("btn-snap").onclick = snapToStation;
function triggerSmartGraspOrRelease(gestureLabel) {
  const s = ST[st.station];
  const isHolding = st.arm[6].held || st.arm[4].held;
  const f6 = fk6(st.q6);

  if (s.id === "battery") {
    if (!isHolding) {
      const bp = [st.arm[6].battP[0], st.arm[6].battP[1], 0.032];
      const d6 = Math.hypot(f6.tip[0] - bp[0], f6.tip[1] - bp[1], f6.tip[2] - bp[2]);
      const pickYaw = bp[1] > 0.02 ? 25 : -25;
      if (d6 <= 0.095) {
        st.cmd.x = bp[0]; st.cmd.y = bp[1]; st.cmd.z = 0.032; st.cmd.yaw = pickYaw;
        setClosed(true, true, "grasp_cell");
        $("watch-status").textContent = "Cell grasped -> Move to Pad & Release";
      } else {
        send({ t: "grip", g: 1 });
        $("watch-status").textContent = "Aligning & grasping cell…";
        tweenTo([pickYaw, 0, 0], bp, 240, () => {
          setClosed(true, true, "grasp_cell");
          $("watch-status").textContent = "Cell grasped -> Move to Pad & Release";
        });
      }
      return;
    } else {
      const destP = st.arm[6].dest === "pick" ? PICK_P : PLACE_P;
      const destYaw = st.arm[6].dest === "pick" ? -25 : 25;
      const dDest = Math.hypot(f6.tip[0] - destP[0], f6.tip[1] - destP[1]);
      if (dDest <= 0.105) {
        st.cmd.x = destP[0]; st.cmd.y = destP[1]; st.cmd.z = destP[2]; st.cmd.yaw = destYaw;
        setClosed(false, true, "release_cell");
        $("watch-status").textContent = "Cell placed on Target Pad!";
      } else {
        send({ t: "grip", g: 0 });
        $("watch-status").textContent = "Guiding to Target Pad & releasing…";
        tweenTo([destYaw, 0, 0], destP, 260, () => {
          setClosed(false, true, "release_cell");
          $("watch-status").textContent = "Cell placed on Target Pad!";
        });
      }
      return;
    }
  }

  if (s.id === "charger") {
    if (isHolding) {
      const dCharger = Math.hypot(f6.tip[0] - s.p[0], f6.tip[1] - s.p[1], f6.tip[2] - s.p[2]);
      if (dCharger <= 0.095) {
        st.cmd.x = s.p[0]; st.cmd.y = s.p[1]; st.cmd.z = s.p[2];
        setClosed(false, true, "slot_charger");
        $("watch-status").textContent = "Cell slotted into Charger Dock!";
      } else {
        send({ t: "grip", g: 0 });
        $("watch-status").textContent = "Guiding into Charger Slot…";
        tweenTo(s.ypr, s.p, 260, () => {
          setClosed(false, true, "slot_charger");
          $("watch-status").textContent = "Cell slotted into Charger Dock!";
        });
      }
      return;
    } else {
      for (const d of [4, 6]) { st.arm[d].inserted = null; st.arm[d].held = true; st.arm[d].done[1] = false; }
      setClosed(true, false);
      $("watch-status").textContent = "Cell re-grasped -> Ready to slot";
      return;
    }
  }

  if (s.id === "valve") {
    if (!st.closed) {
      const dv = Math.hypot(f6.tip[0] - s.p[0], f6.tip[1] - s.p[1], f6.tip[2] - s.p[2]);
      if (dv > POS_TOL) {
        tweenTo(s.yprGrip, s.p, 240, () => {
          setClosed(true);
          later(180, () => tweenTo(s.ypr, s.p, 700));
        });
        return;
      }
      setClosed(true);
      tweenTo(s.ypr, s.p, 700);
      return;
    } else {
      setClosed(false);
      return;
    }
  }

  setClosed(!st.closed);
  if (gestureLabel) $("watch-status").textContent = `Watch: ${gestureLabel}`;
}
$("btn-grab").onclick = () => triggerSmartGraspOrRelease();
$("btn-reset").onclick = () => {
  st.arm = { 4: freshArm(), 6: freshArm() };
  st.quinticPlan = null;
  setClosed(false);
  setStation(0);
};

const later = (ms, fn) => setTimeout(fn, ms);

function runPickAndPlaceCycle() {
  if (st.station !== 0) setStation(0);
  for (const d of [4, 6]) {
    st.arm[d].held = false;
    st.arm[d].done[0] = false;
    st.arm[d].battP = [PICK_P[0], PICK_P[1], 0.013];
    st.arm[d].battR = PICK_R;
    st.arm[d].dest = "place";
  }
  setClosed(false, false);
  $("watch-status").textContent = "Executing Pick & Place (Approach -> Grasp -> Lift -> Place -> Release)";
  const abovePick = [PICK_P[0], PICK_P[1], PICK_P[2] + 0.07];
  const abovePlace = [PLACE_P[0], PLACE_P[1], PLACE_P[2] + 0.09];
  tweenTo([-25, 0, 0], abovePick, 480, () =>
    tweenTo([-25, 0, 0], PICK_P, 420, () =>
      later(180, () => {
        setClosed(true, false);
        later(180, () => tweenTo([-25, 0, 0], abovePick, 400, () =>
          tweenTo([25, 0, 0], abovePlace, 540, () =>
            tweenTo([25, 0, 0], PLACE_P, 420, () =>
              later(180, () => {
                setClosed(false, false);
                later(160, () => tweenTo([25, 0, 0], abovePlace, 400));
              })
            )
          )
        ));
      })
    )
  );
}
if ($("btn-pick-place")) $("btn-pick-place").onclick = runPickAndPlaceCycle;

$("btn-auto").onclick = () => {
  const s = ST[st.station];
  if (s.id === "battery") {
    runPickAndPlaceCycle();
  } else if (s.id === "charger") {
    for (const d of [4, 6]) { st.arm[d].inserted = null; st.arm[d].held = true; st.arm[d].done[1] = false; }
    setClosed(true);
    const back = [s.p[0] - s.R[2] * 0.05, s.p[1] - s.R[5] * 0.05, s.p[2] - s.R[8] * 0.05];
    tweenTo(s.ypr, back, 1000, () => tweenTo(s.ypr, s.p, 750, () => later(400, () => setClosed(false))));
  } else {
    setClosed(false);
    tweenTo(s.yprGrip, s.p, 1100, () => later(300, () => { setClosed(true); later(400, () => tweenTo(s.ypr, s.p, 1300)); }));
  }
};

// Firmware Quintic Sweep (executes exact 5th-order polynomial segments from config/trajectories/*.yaml)
async function startQuinticSweep() {
  await fetchTrajectoriesFromServer();
  st.tween = null;
  const trajKey = $("traj-select")?.value || st.selectedTraj || "imu_calibration";
  st.selectedTraj = trajKey;
  const trajObj = TRAJECTORY_CACHE[trajKey] || TRAJECTORY_CACHE.imu_calibration;
  const segs = expandTrajectorySegments(trajObj);
  st.quinticPlan = { t0: performance.now() / 1000, segs, name: `${trajKey}.yaml` };
  if ($("watch-status")) {
    $("watch-status").textContent = `Playing preset sweep: ${trajKey}.yaml (${segs.length} smooth 5th-order steps)`;
  }
}
if ($("btn-quintic")) $("btn-quintic").onclick = startQuinticSweep;
if ($("btn-synth-watch")) $("btn-synth-watch").onclick = startQuinticSweep;
if ($("traj-select")) {
  $("traj-select").addEventListener("change", () => {
    st.selectedTraj = $("traj-select").value;
    startQuinticSweep();
  });
}
if ($("btn-clear-watch")) {
  $("btn-clear-watch").onclick = () => {
    st.uploadedWatchCSV = null;
    st.uploadedWatchIdx = 0;
    st.watchSourceName = null;
    if ($("watch-file")) $("watch-file").value = "";
    if ($("watch-status")) $("watch-status").textContent = "Source: Analytic Sim IMU only (no Watch CSV or live stream)";
  };
}

// Parse uploaded Apple Watch CSV (SensorLog wide format or Sensor Logger long/combined format, matching compare.py:parse_watch)
$("watch-file").addEventListener("change", async (e) => {
  const file = e.target.files?.[0];
  if (!file) return;
  const text = await file.text();
  const lines = text.split(/\r?\n/).filter(l => l.trim() && !l.trim().startsWith("#"));
  if (lines.length < 3) return;
  const rawHdr = lines[0].split(",").map(h => h.trim());
  const normHdr = rawHdr.map(h => h.toLowerCase().replace(/[^a-z0-9]/g, ""));
  const findCol = (...cands) => {
    for (const c of cands) {
      const k = c.toLowerCase().replace(/[^a-z0-9]/g, "");
      const idx = normHdr.indexOf(k);
      if (idx >= 0) return idx;
    }
    return -1;
  };

  const sensorCol = findCol("sensor");
  const rows = [];

  if (sensorCol >= 0) {
    // Sensor Logger combined long format (rows interleaved by sensor == Accelerometer / Gyroscope)
    const tCol = findCol("seconds_elapsed", "time", "timestamp");
    const xCol = findCol("x"), yCol = findCol("y"), zCol = findCol("z");
    const accRows = [], gyrRows = [];
    for (let i = 1; i < lines.length; i++) {
      const cols = lines[i].split(",");
      if (cols.length <= Math.max(sensorCol, xCol, yCol, zCol)) continue;
      const sType = (cols[sensorCol] || "").toLowerCase();
      const t = tCol >= 0 ? parseFloat(cols[tCol]) || (i * 0.01) : (i * 0.01);
      const x = parseFloat(cols[xCol]) || 0, y = parseFloat(cols[yCol]) || 0, z = parseFloat(cols[zCol]) || 0;
      if (sType.includes("accel") && !sType.includes("uncalibrated")) accRows.push({ t, v: [x, y, z] });
      else if (sType.includes("gyro") && !sType.includes("uncalibrated")) gyrRows.push({ t, v: [x, y, z] });
    }
    let gIdx = 0;
    for (const a of accRows) {
      while (gIdx + 1 < gyrRows.length && Math.abs(gyrRows[gIdx + 1].t - a.t) <= Math.abs(gyrRows[gIdx].t - a.t)) gIdx++;
      const g = gyrRows[gIdx] ? gyrRows[gIdx].v : [0, 0, 0];
      rows.push({ t: a.t, acc: a.v.slice(), gyro: g.slice() });
    }
  } else {
    // SensorLog wide CSV or pre-aligned CSV (matching compare.py:_parse_sensorlog)
    const tCol = findCol("accelerometertimestampsincereboot", "motiontimestampsincereboot", "timestamp", "seconds_elapsed", "time", "t");
    let axI = findCol("accelerometeraccelerationx", "acc_x", "ax", "x");
    let ayI = findCol("accelerometeraccelerationy", "acc_y", "ay", "y");
    let azI = findCol("accelerometeraccelerationz", "acc_z", "az", "z");
    const uaXI = findCol("motionuseraccelerationx", "useraccelerationx");
    const uaYI = findCol("motionuseraccelerationy", "useraccelerationy");
    const uaZI = findCol("motionuseraccelerationz", "useraccelerationz");
    const grXI = findCol("motiongravityx", "gravityx");
    const grYI = findCol("motiongravityy", "gravityy");
    const grZI = findCol("motiongravityz", "gravityz");
    const gxI = findCol("gyrorotationx", "motionrotationratex", "gyro_x", "gx", "wx");
    const gyI = findCol("gyrorotationy", "motionrotationratey", "gyro_y", "gy", "wy");
    const gzI = findCol("gyrorotationz", "motionrotationratez", "gyro_z", "gz", "wz");

    for (let i = 1; i < lines.length; i++) {
      const cols = lines[i].split(",").map(Number);
      if (cols.length < 3) continue;
      const t = tCol >= 0 && Number.isFinite(cols[tCol]) ? cols[tCol] : i * 0.01;
      let ax = 0, ay = 0, az = 0;
      if (axI >= 0 && ayI >= 0 && azI >= 0) {
        ax = cols[axI] || 0; ay = cols[ayI] || 0; az = cols[azI] || 0;
      } else if (uaXI >= 0 && grXI >= 0) {
        ax = (cols[uaXI] || 0) + (cols[grXI] || 0);
        ay = (cols[uaYI] || 0) + (cols[grYI] || 0);
        az = (cols[uaZI] || 0) + (cols[grZI] || 0);
      }
      const gx = gxI >= 0 ? (cols[gxI] || 0) : 0;
      const gy = gyI >= 0 ? (cols[gyI] || 0) : 0;
      const gz = gzI >= 0 ? (cols[gzI] || 0) : 0;
      if (!Number.isFinite(ax) || !Number.isFinite(ay) || !Number.isFinite(az)) continue;
      rows.push({ t, acc: [ax, ay, az], gyro: [gx, gy, gz] });
    }
  }

  if (rows.length) {
    // Unit auto-detection matching compare.py:_parse_sensorlog (median |a| in [0.5, 2.0] => g -> m/s^2; max |w| > 25 => deg/s -> rad/s)
    const mags = rows.map(r => Math.hypot(...r.acc)).sort((a, b) => a - b);
    const medMag = mags[Math.floor(mags.length / 2)] || 0;
    const scaleA = (medMag > 0.5 && medMag < 2.0) ? G0 : 1.0;
    const maxG = Math.max(...rows.map(r => Math.hypot(...r.gyro)), 0);
    const scaleG = maxG > 25.0 ? D2R : 1.0;
    const t0 = rows[0].t;
    rows.forEach(r => {
      r.t = r.t - t0;
      r.acc = r.acc.map(v => v * scaleA);
      r.gyro = r.gyro.map(v => v * scaleG);
    });
    st.uploadedWatchCSV = rows;
    st.uploadedWatchIdx = 0;
    st.watchSourceName = `${file.name} (${rows.length} samples, ${scaleA === G0 ? "g->m/s²" : "m/s²"})`;
    $("watch-status").textContent = `Loaded Watch CSV: ${st.watchSourceName}`;
  }
});

// Export sim_imu.csv (and watch_aligned.csv only if real Watch data is present), compatible with `python compare.py --sim sim_imu.csv --watch watch.csv`
$("btn-export-imu").onclick = () => requireLicense("Sim + Watch IMU CSV Export", () => {
  const hist = st.imuHistory.length ? st.imuHistory : [];
  if (!hist.length) return;
  const t0 = hist[0].t;
  const simLines = ["t,ax,ay,az,gx,gy,gz,utc_iso,epoch_utc_ms"];
  const watchLines = ["loggingTime,epoch_utc_ms,accelerometerTimestamp_sinceReboot,accelerometerAccelerationX,accelerometerAccelerationY,accelerometerAccelerationZ,gyroRotationX,gyroRotationY,gyroRotationZ"];
  let hasWatch = false;
  hist.forEach(h => {
    const dt = (h.t - t0).toFixed(6);
    const uIso = h.utcIso || new Date().toISOString();
    const uMs = h.utcEpochMs || Date.now();
    simLines.push(`${dt},${h.simAcc.map(v => v.toFixed(6)).join(",")},${h.simGyro.map(v => v.toFixed(6)).join(",")},${uIso},${uMs}`);
    if (h.watchAccRaw && h.watchGyroRaw) {
      hasWatch = true;
      const rawG = h.watchAccRaw.map(v => (v / G0).toFixed(6));
      const rawW = h.watchGyroRaw.map(v => v.toFixed(6));
      watchLines.push(`${uIso},${uMs},${dt},${rawG.join(",")},${rawW.join(",")}`);
    }
  });
  downloadText("sim_imu.csv", simLines.join("\n"), "text/csv");
  if (hasWatch) {
    setTimeout(() => downloadText("watch_captured.csv", watchLines.join("\n"), "text/csv"), 250);
  }
});

// Sliders & Drive Modes
const showCmd = (c) => {
  for (const k of ["yaw", "pitch", "roll", "x", "y", "z"]) {
    const inp = document.querySelector(`[data-p="${k}"]`);
    const val = document.querySelector(`[data-pv="${k}"]`);
    if (inp) inp.value = c[k];
    if (val) val.textContent = k.length === 1 ? (+c[k]).toFixed(3) : `${Math.round(c[k])}°`;
  }
};
document.querySelectorAll("[data-p]").forEach(inp => inp.addEventListener("input", () => {
  st.cmd[inp.dataset.p] = parseFloat(inp.value);
  st.tween = null; st.quinticPlan = null;
  st.holdCrowd = performance.now() + 1200;
  // Instant zero-latency kinematic solve on slider drag with continuous wrist unwrapping
  const R = poseR(st.cmd.yaw, st.cmd.pitch, st.cmd.roll), p = [st.cmd.x, st.cmd.y, st.cmd.z];
  st.q4 = ik4(p, R).q;
  st.q6 = ik6(p, R, st.q6).q;
  showCmd(st.cmd);
}));

// Highlight the corresponding 3D Yaw / Pitch / Roll / X / Y / Z gizmo ring when hovering or dragging any pose slider
document.querySelectorAll("[data-axis-hover]").forEach(row => {
  row.addEventListener("mouseenter", () => { st.highlightAxis = row.dataset.axisHover; });
  row.addEventListener("mouseleave", () => { st.highlightAxis = null; });
});

if ($("btn-gizmo-toggle")) {
  $("btn-gizmo-toggle").onclick = () => {
    st.showGizmos = !st.showGizmos;
    $("btn-gizmo-toggle").textContent = `3D Axes: ${st.showGizmos ? "ON" : "OFF"}`;
    $("btn-gizmo-toggle").classList.toggle("on", st.showGizmos);
  };
}

if ($("studio-gain-bar")) {
  $("studio-gain-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
    st.studioGain = parseFloat(b.dataset.sg) || 3.2;
    $("studio-gain-bar").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  }));
}
if ($("btn-recenter-wrist")) {
  $("btn-recenter-wrist").onclick = () => {
    const t = armTarget(st.arm[6].held ? 6 : 4);
    st.deskBasePos = [t.p[0], t.p[1], Math.max(0.08, t.p[2])];
    st.cmd = { yaw: t.ypr[0], pitch: t.ypr[1], roll: t.ypr[2], x: st.deskBasePos[0], y: st.deskBasePos[1], z: st.deskBasePos[2] };
    const R = poseR(st.cmd.yaw, st.cmd.pitch, st.cmd.roll), p = [st.cmd.x, st.cmd.y, st.cmd.z];
    st.q4 = ik4(p, R).q;
    st.q6 = ik6(p, R, st.q6).q;
    showCmd(st.cmd);
    $("watch-status").textContent = "Centered at station desk origin (0.15, 0.00, 0.11 m)";
  };
}

$("drive-mode").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
  $("drive-mode").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  st.drive = b.dataset.v;
  if (st.drive === "pilot" && !st.pilot && phones.size) st.pilot = [...phones.keys()][0];
  if (st.drive !== "pilot") st.pilot = null;
  assignAll(); pushState(); renderPilots();
}));
$("btn-dice").onclick = () => {
  $("drive-mode").querySelector('[data-v="pilot"]').click();
  const ids = [...phones.keys()];
  if (ids.length) { st.pilot = ids[Math.floor(Math.random() * ids.length)]; renderPilots(); pushState(); }
};

// Domain Randomization Controls (Lighting, PBR Surface Roughness & Object Pose Perturbation)
function updateDR() {
  const light = parseFloat($("dr-light")?.value ?? "2.8");
  const aziDeg = parseFloat($("dr-azi")?.value ?? "35");
  const azi = aziDeg * D2R;
  const rough = parseFloat($("dr-rough")?.value ?? "0.22");
  if ($("drv-light")) $("drv-light").textContent = `${light.toFixed(1)}x`;
  if ($("drv-azi")) $("drv-azi").textContent = `${Math.round(aziDeg)}°`;
  if ($("drv-rough")) $("drv-rough").textContent = rough.toFixed(2);
  sun.intensity = light;
  sun.position.set(Math.cos(azi) * 0.95, 1.25, Math.sin(azi) * 0.95);
  hemi.intensity = 0.35 + light * 0.22;
  rimLight.intensity = 0.8 + light * 0.65;
  // Apply roughness across all robot arm, joint cap, steel, and optical bench materials so visual change is immediate
  for (const m of [MAT.alu, MAT.urSilver, MAT.urBlueCap, MAT.urPurpleCap, MAT.carbon, MAT.dark, MAT.steel, MAT.copper, MAT.watch]) {
    if (m) {
      m.roughness = rough;
      if ("clearcoatRoughness" in m) m.clearcoatRoughness = Math.min(0.9, rough * 0.75);
      m.needsUpdate = true;
    }
  }
  MAT.bench.roughness = Math.min(0.95, rough * 0.85 + 0.08);
  MAT.bench.needsUpdate = true;
}
["dr-light", "dr-azi", "dr-rough"].forEach(id => $(id)?.addEventListener("input", updateDR));
if ($("btn-dr-rand")) {
  $("btn-dr-rand").onclick = () => {
    $("dr-light").value = (0.6 + Math.random() * 3.6).toFixed(1);
    $("dr-azi").value = Math.round(-175 + Math.random() * 350);
    $("dr-rough").value = (0.05 + Math.random() * 0.85).toFixed(2);
    sun.color.setHSL(Math.random() * 0.14 + 0.04, 0.30, 0.96);
    updateDR();
    if ($("watch-status")) {
      $("watch-status").textContent = `Randomized scene: Light ${$("drv-light").textContent}, Angle ${$("drv-azi").textContent}, Roughness ${$("drv-rough").textContent}`;
    }
  };
}
if ($("btn-dr-pose")) {
  $("btn-dr-pose").onclick = () => {
    if (st.station !== 0) setStation(0);
    setClosed(false, false);
    const jx = (Math.random() - 0.5) * 0.075;
    const jy = (Math.random() - 0.5) * 0.075;
    const nx = Math.max(0.105, Math.min(0.195, 0.15 + jx));
    const ny = Math.max(-0.115, Math.min(-0.035, -0.075 + jy));
    const randYaw = Math.round((Math.random() - 0.5) * 70);
    for (const d of [4, 6]) {
      st.arm[d].held = false;
      st.arm[d].done[0] = false;
      st.arm[d].battP = [nx, ny, 0.013];
      st.arm[d].battR = poseR(randYaw, 0, 0);
    }
    if ($("watch-status")) {
      $("watch-status").textContent = `Perturbed Cell Pose -> x=${(nx * 1000).toFixed(0)} mm, y=${(ny * 1000).toFixed(0)} mm, yaw=${randYaw}°`;
    }
  };
}
// Ensure any collapsible <details> drawer automatically scrolls into view when opened
document.querySelectorAll("details.drawer").forEach(det => {
  det.addEventListener("toggle", () => {
    if (det.open) det.scrollIntoView({ behavior: "smooth", block: "nearest" });
  });
});
$("btn-bboxes").onclick = () => {
  st.showBBoxes = !st.showBBoxes;
  $("btn-bboxes").textContent = `2D/3D BBoxes: ${st.showBBoxes ? "ON" : "OFF"}`;
  $("btn-bboxes").classList.toggle("on", st.showBBoxes);
};

// AOV Pass Bar (Click to switch pass, Hover to preview explanation & legend in the viewport banner)
$("aov-bar").querySelectorAll("button").forEach(b => {
  b.addEventListener("click", () => {
    $("aov-bar").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
    applyAOVPass(b.dataset.aov);
  });
  b.addEventListener("mouseenter", () => renderAOVBanner(b.dataset.aov));
  b.addEventListener("mouseleave", () => renderAOVBanner(st.aov));
});

// ── Live 3D Scene Raycast Hover Probe (Semantic ID, Instance Prim, Surface Normal, Metric Depth Z) ──
const raycaster = new THREE.Raycaster();
const pointerNDC = new THREE.Vector2();
const probeEl = $("scene-hover-probe");

cv.addEventListener("pointermove", (e) => {
  if (!probeEl) return;
  if (e.buttons) { probeEl.classList.add("hide"); return; } // hide while orbiting
  const rect = cv.getBoundingClientRect();
  pointerNDC.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNDC.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointerNDC, camera);

  const hits = raycaster.intersectObjects(scene.children, true);
  const hit = hits.find(h => h.object && h.object.isMesh && !h.object.userData.isHelper && h.object.visible);
  if (!hit) {
    probeEl.classList.add("hide");
    return;
  }

  // Walk up parent chain to find hoverMeta
  let cur = hit.object, meta = null;
  while (cur && !meta) {
    if (cur.userData && cur.userData.hoverMeta) meta = cur.userData.hoverMeta;
    cur = cur.parent;
  }
  if (!meta) {
    probeEl.classList.add("hide");
    return;
  }

  // Compute world-space surface normal N and metric distance Z from camera
  const worldNormal = hit.face ? hit.face.normal.clone().transformDirection(hit.object.matrixWorld).normalize() : new THREE.Vector3(0, 1, 0);
  // Convert Three.js (x, y_up, z_right) back to Robotics World Frame (X_forward, Y_left, Z_up)
  const robPt = [hit.point.x, -hit.point.z, hit.point.y];
  const robNorm = [worldNormal.x, -worldNormal.z, worldNormal.y];

  $("probe-name").textContent = meta.title;
  $("probe-class").textContent = st.aov.toUpperCase();
  $("probe-sem").textContent = meta.sem;
  $("probe-inst").textContent = meta.inst;
  $("probe-norm").textContent = `N = (${robNorm.map(v => (v >= 0 ? "+" : "") + v.toFixed(2)).join(", ")})`;
  $("probe-depth").textContent = `${hit.distance.toFixed(3)} m from lens`;
  $("probe-xyz").textContent = `(${robPt.map(v => v.toFixed(3)).join(", ")}) m`;
  $("probe-desc").textContent = meta.desc;

  const px = Math.min(window.innerWidth - 295, e.clientX + 16);
  const py = Math.min(window.innerHeight - 175, Math.max(12, e.clientY + 14));
  probeEl.style.left = `${px}px`;
  probeEl.style.top = `${py}px`;
  probeEl.classList.remove("hide");
});
cv.addEventListener("pointerleave", () => probeEl?.classList.add("hide"));

// ── Global Rich UI Hover Card for Controls, Metrics & AOV Buttons ──
const uiHoverCard = $("ui-hover-card");
document.querySelectorAll("[title]").forEach(el => {
  const rawTitle = el.getAttribute("title");
  if (!rawTitle) return;
  el.dataset.hoverDoc = rawTitle;
  el.removeAttribute("title"); // prevent slow native browser tooltip; use instant rich studio hover card
  el.addEventListener("mouseenter", (e) => {
    if (!uiHoverCard) return;
    const parts = rawTitle.split(":");
    const hasHeading = parts.length > 1 && parts[0].length < 48;
    $("ui-hover-tag").textContent = el.closest("#aov-bar") ? "Omniverse Replicator AOV" : el.closest("#pose-sliders") ? "6-DOF Kinematic Frame" : "Operon Studio Inspector";
    $("ui-hover-title").textContent = hasHeading ? parts[0].trim() : (el.textContent.trim().slice(0, 42) || "Control Info");
    $("ui-hover-body").textContent = hasHeading ? parts.slice(1).join(":").trim() : rawTitle;
    const r = el.getBoundingClientRect();
    const left = Math.min(window.innerWidth - 320, Math.max(10, r.left));
    const top = r.bottom + 8 + 110 > window.innerHeight ? Math.max(10, r.top - 95) : r.bottom + 8;
    uiHoverCard.style.left = `${left}px`;
    uiHoverCard.style.top = `${top}px`;
    uiHoverCard.classList.remove("hide");
  });
  el.addEventListener("mouseleave", () => uiHoverCard?.classList.add("hide"));
});
$("btn-cam-reset").onclick = () => {
  camera.position.set(0.65, 0.36, 0.0);
  controls.target.set(0.08, 0.06, 0);
  controls.update();
};
if ($("btn-expand-canvas")) {
  let expandStage = 0; // 0 = 3-col wide center, 1 = hide right col, 2 = full-bleed 3D canvas
  $("btn-expand-canvas").onclick = () => {
    const wsEl = document.querySelector(".workspace");
    if (!wsEl) return;
    expandStage = (expandStage + 1) % 3;
    wsEl.classList.toggle("canvas-expanded", expandStage === 1);
    wsEl.classList.toggle("canvas-fullscreen", expandStage === 2);
    $("btn-expand-canvas").textContent = expandStage === 0 ? "Expand 3D" : expandStage === 1 ? "Full 3D" : "Restore Panels";
    $("btn-expand-canvas").classList.toggle("on", expandStage > 0);
    setTimeout(syncViewportSize, 20);
    setTimeout(syncViewportSize, 200);
  };
}
$("btn-snap-png").onclick = () => {
  const a = document.createElement("a");
  a.href = cv.toDataURL("image/png");
  a.download = `omniverse_aov_${st.aov}_${Date.now()}.png`;
  a.click();
};

// Recorder & Schema Inspector
$("btn-rec").onclick = () => {
  st.recording = !st.recording;
  $("btn-rec").classList.toggle("active", st.recording);
  $("btn-rec").textContent = st.recording ? "Stop Recording" : "Record Episode (30 Hz)";
  updateRecBadge();
};
$("btn-clear-ep").onclick = () => {
  st.episode = [];
  updateRecBadge();
};
function updateRecBadge() {
  $("rec-badge").className = `pill ${st.recording ? "no" : st.episode.length ? "ok" : ""}`;
  $("rec-badge").textContent = `${st.recording ? "REC" : "IDLE"} · ${st.episode.length} frames`;
}

$("schema-tabs").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
  $("schema-tabs").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  st.schemaTab = b.dataset.s;
}));
$("imu-chart-mode").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
  $("imu-chart-mode").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  st.imuChartMode = b.dataset.ic;
}));

function downloadText(filename, text, mime = "application/json") {
  const blob = new Blob([text], { type: mime });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

function buildOpenUSD(stepObj, epList) {
  const samples = epList.length ? epList : [stepObj];
  const q6SimSamples = samples.map((s, i) => {
    const arr = s.observation.q6_sim_rad || s.observation.q6_rad || [0, 0, 0, 0, 0, 0];
    return `        ${i}: (${arr.map(v => (v * R2D).toFixed(2)).join(", ")})`;
  }).join(",\n");
  const q6RealSamples = samples.map((s, i) => {
    const arr = s.observation.q6_real_rad || s.observation.q6_rad || [0, 0, 0, 0, 0, 0];
    return `        ${i}: (${arr.map(v => (v * R2D).toFixed(2)).join(", ")})`;
  }).join(",\n");
  const imuSamples = samples.slice(-15).map((s, i) => `        ${i}: (${s.observation.forearm_imu.sim_acc_ms2.join(", ")})`).join(",\n");
  return `#usda 1.0
(
    defaultPrim = "World"
    endTimeCode = ${Math.max(0, samples.length - 1)}
    metersPerUnit = 1.0
    startTimeCode = 0
    timeCodesPerSecond = 30
    upAxis = "Z"
    doc = "Exported from Operon 6-DOF Sim-to-Real Kinematic & Inertial Calibration Workbench"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.80665
    }

    def ArticulationRoot "ur5e_sim_6dof" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        token[] dofNames = ["yaw", "shoulder", "elbow", "wrist_roll", "wrist_pitch", "tool_roll"]
        double[] drive:jointPositions.timeSamples = {
${q6SimSamples}
        }
    }

    def ArticulationRoot "ur5e_real_twin_6dof" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        token[] dofNames = ["yaw", "shoulder", "elbow", "wrist_roll", "wrist_pitch", "tool_roll"]
        double[] drive:jointPositions.timeSamples = {
${q6RealSamples}
        }

        def Xform "forearm_link"
        {
            def IsaacImuSensor "forearm_imu" (
                doc = "Apple Watch Forearm Mount Site (abel_sim2real contract)"
            )
            {
                float3 sensor:linearAcceleration.timeSamples = {
${imuSamples}
                }
            }
        }
    }
}
`;
}

$("exp-lerobot").onclick = () => requireLicense("LeRobot Episode (.jsonl) Export", () => {
  const rows = st.episode.length ? st.episode : [latestStepRecord];
  downloadText(`lerobot_episode_${Date.now()}.jsonl`, rows.map(r => JSON.stringify(r)).join("\n") + "\n", "application/x-ndjson");
});
$("exp-usd").onclick = () => requireLicense("OpenUSD Stage (.usda) Export", () => {
  downloadText(`omniverse_stage_${Date.now()}.usda`, buildOpenUSD(latestStepRecord, st.episode), "text/plain");
});
$("exp-coco").onclick = () => requireLicense("Omniverse Replicator COCO (.json) Export", () => {
  const payload = {
    info: { generator: "Operon Omniverse Replicator AOV Studio", aov_pass: st.aov, timestamp: new Date().toISOString() },
    categories: [
      { id: 1, name: "robot_arm_4dof" }, { id: 2, name: "robot_arm_6dof" },
      { id: 3, name: "battery_cell" }, { id: 4, name: "charger_dock" },
      { id: 5, name: "valve_wheel" }, { id: 6, name: "simready_toolbox" }
    ],
    annotations: latestBBoxes
  };
  downloadText(`replicator_coco_${Date.now()}.json`, JSON.stringify(payload, null, 2));
});

// ── 10. Main Physics, Kinematics & Telemetry Loop ──
const smooth = (cur, tgt, k) => cur.map((a, i) => { let d = tgt[i] - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return a + d * k; });
const ease = (k) => k < 0.5 ? 4 * k * k * k : 1 - Math.pow(-2 * k + 2, 3) / 2;

let last = performance.now(), lastRec = 0, lastUi = 0, stepIdx = 0;
let prevSiteP = null, prevSiteV = [0, 0, 0], prevSiteR = null;
let latestBBoxes = [], latestStepRecord = {}, latestSim2RealReport = "";

// NTP / IEEE-1588 UTC Clock Offset Calibration against Studio Server
async function syncUtcClock() {
  try {
    const t0 = Date.now();
    const r = await fetch("/api/health", { cache: "no-store" });
    const t1 = Date.now();
    if (r.ok) {
      const d = await r.json();
      if (d.epoch_utc_ms) {
        const rtt = t1 - t0;
        st.utcOffsetMs = +(d.epoch_utc_ms - (t0 + rtt * 0.5)).toFixed(1);
      }
    }
  } catch {}
}
syncUtcClock();
setInterval(syncUtcClock, 30000);

function buildYamlContractPreview() {
  const trajKey = st.selectedTraj || "imu_calibration";
  const trajObj = TRAJECTORY_CACHE[trajKey] || TRAJECTORY_CACHE.imu_calibration;
  const segs = expandTrajectorySegments(trajObj);
  const totalDur = segs.reduce((a, s) => a + s.moveT + s.settleT, 0);
  const segRows = segs.map((s, idx) =>
    `  #${idx} [${s.q0.map(v => String(v).padStart(3)).join(",")}] -> [${s.qf.map(v => String(v).padStart(3)).join(",")}] | cmd=${s.commandedT.toFixed(2)}s min=${s.minT.toFixed(2)}s move=${s.moveT.toFixed(2)}s settle=${s.settleT.toFixed(2)}s | ${s.label}`
  ).join("\n");
  return `================================================================
FIRMWARE QUINTIC TRAJECTORY CONTRACT: config/trajectories/${trajKey}.yaml
================================================================
joint_order : [${(trajObj.joint_order || ["base", "shoulder", "elbow", "gripper"]).join(", ")}]
home_deg    : [${(trajObj.home || [90, 90, 90, 90]).join(", ")}]  (servo 90 deg = URDF 0.0 rad)
segments    : ${segs.length} waypoints | total duration = ${totalDur.toFixed(2)} s
limits      : V_MAX_DEG_S = ${V_MAX_DEG_S.toFixed(1)} deg/s | A_MAX_DEG_S2 = ${A_MAX_DEG_S2.toFixed(1)} deg/s^2
hw_streamer : AbelArmHardwareInterface (deadband=1.0 deg, min_send=100 ms, segment_time=150 ms)
----------------------------------------------------------------
Expanded 5th-Order Quintic Segments (sim/trajectory.py:expand_segments):
${segRows}
================================================================`;
}

function prepareHiDPICanvas(cvEl) {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const cssW = Math.max(180, cvEl.clientWidth || 260);
  const cssH = Math.max(50, cvEl.clientHeight || 90);
  const physW = Math.round(cssW * dpr);
  const physH = Math.round(cssH * dpr);
  if (cvEl.width !== physW || cvEl.height !== physH) {
    cvEl.width = physW;
    cvEl.height = physH;
  }
  const ctx = cvEl.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);
  return { ctx, W: cssW, H: cssH };
}

function drawCharts() {
  const hist = st.imuHistory;
  if (hist.length < 2) return;
  const hasWatch = hist.some(h => h.watchAcc !== null && h.watchGyro !== null);

  // Chart 1: Forearm 6-Axis IMU Overlay (Analytic Sim IMU + Real Watch IMU when connected/loaded)
  const { ctx: ctx1, W: W1, H: H1 } = prepareHiDPICanvas($("chart-imu"));

  if (st.imuChartMode === "psd") {
    const N = Math.min(128, hist.length);
    const slice = hist.slice(-N);
    const sSig = slice.map(h => Math.hypot(...h.simAcc));
    const sMean = sSig.reduce((a, b) => a + b, 0) / N;
    const bins = Math.floor(N / 2);
    const sPsd = [], wPsd = [];
    const wSig = hasWatch ? slice.map(h => h.watchAcc ? Math.hypot(...h.watchAcc) : 0) : null;
    const wMean = wSig ? wSig.reduce((a, b) => a + b, 0) / N : 0;
    for (let k = 1; k < bins; k++) {
      let sr = 0, si = 0, wr = 0, wi = 0;
      for (let n = 0; n < N; n++) {
        const ang = (-2 * Math.PI * k * n) / N;
        const c = Math.cos(ang), s = Math.sin(ang);
        sr += (sSig[n] - sMean) * c; si += (sSig[n] - sMean) * s;
        if (wSig) { wr += (wSig[n] - wMean) * c; wi += (wSig[n] - wMean) * s; }
      }
      sPsd.push(Math.log10(Math.hypot(sr, si) / N + 1e-4));
      if (wSig) wPsd.push(Math.log10(Math.hypot(wr, wi) / N + 1e-4));
    }
    const plotLine = (arr, col) => {
      ctx1.strokeStyle = col; ctx1.lineWidth = 1.8; ctx1.beginPath();
      arr.forEach((v, i) => {
        const x = (i / Math.max(1, arr.length - 1)) * (W1 - 20) + 10;
        const y = H1 - 14 - ((v + 4) / 4.5) * (H1 - 24);
        i ? ctx1.lineTo(x, y) : ctx1.moveTo(x, y);
      });
      ctx1.stroke();
    };
    plotLine(sPsd, "#60a5fa");
    if (wPsd.length) plotLine(wPsd, "#e879f9");
    ctx1.fillStyle = "#8492ad"; ctx1.font = "500 10px 'Inter', system-ui, sans-serif";
    ctx1.textAlign = "left";
    ctx1.fillText("0 Hz", 8, H1 - 4);
    ctx1.textAlign = "right";
    ctx1.fillText("30 Hz FFT", W1 - 8, H1 - 4);
    ctx1.textAlign = "left";
  } else if (st.imuChartMode === "fused") {
    const sAcc = hist.map(h => Math.hypot(...h.simAcc));
    const sGyr = hist.map(h => Math.hypot(...h.simGyro));
    const wAcc = hasWatch ? hist.map(h => h.watchAcc ? Math.hypot(...h.watchAcc) : null) : [];
    const wGyr = hasWatch ? hist.map(h => h.watchGyro ? Math.hypot(...h.watchGyro) : null) : [];

    const validWAcc = wAcc.filter(v => v !== null);
    const validWGyr = wGyr.filter(v => v !== null);
    const minA = 6.5, maxA = Math.max(14.0, ...sAcc, ...validWAcc);
    const minG = 0.0, maxG = Math.max(3.0, ...sGyr, ...validWGyr);
    const mapA = (v) => H1 - 12 - ((v - minA) / Math.max(0.1, maxA - minA)) * (H1 - 28);
    const mapG = (v) => H1 - 12 - ((v - minG) / Math.max(0.1, maxG - minG)) * (H1 - 28);

    const strokeTrace = (arr, mapper, col, lw, dash = []) => {
      ctx1.strokeStyle = col; ctx1.lineWidth = lw; ctx1.setLineDash(dash); ctx1.beginPath();
      let started = false;
      arr.forEach((v, i) => {
        if (v === null || v === undefined) return;
        const x = (i / Math.max(1, arr.length - 1)) * W1;
        const y = mapper(v);
        if (!started) { ctx1.moveTo(x, y); started = true; } else ctx1.lineTo(x, y);
      });
      ctx1.stroke();
      ctx1.setLineDash([]);
    };
    strokeTrace(sAcc, mapA, "#60a5fa", 2.0);
    strokeTrace(sGyr, mapG, "#b99aff", 1.9);
    if (hasWatch) {
      strokeTrace(wAcc, mapA, "#e879f9", 1.5, [4, 2]);
      strokeTrace(wGyr, mapG, "#38bdf8", 1.4, [3, 2]);
    }

    ctx1.font = "500 10px 'Inter', system-ui, sans-serif";
    ctx1.fillStyle = "#93c5fd";
    ctx1.textAlign = "left";
    ctx1.fillText("Sim |a| & |ω|", 8, 13);
    ctx1.fillStyle = hasWatch ? "#f0abfc" : "#8492ad";
    ctx1.textAlign = "right";
    ctx1.fillText(hasWatch ? "Watch: Live" : "No Watch", W1 - 8, 13);
    ctx1.textAlign = "left";
  } else {
    const isAcc = st.imuChartMode === "acc";
    const sVals = hist.map(h => isAcc ? Math.hypot(...h.simAcc) : Math.hypot(...h.simGyro));
    const wVals = hasWatch ? hist.map(h => {
      const vec = isAcc ? h.watchAcc : h.watchGyro;
      return vec ? Math.hypot(...vec) : null;
    }) : [];
    const validW = wVals.filter(v => v !== null);
    const minV = isAcc ? 6.5 : 0.0;
    const maxV = isAcc ? Math.max(13.5, ...sVals, ...validW) : Math.max(2.5, ...sVals, ...validW);
    const mapY = (v) => H1 - 10 - ((v - minV) / Math.max(0.1, maxV - minV)) * (H1 - 24);

    const strokeArr = (arr, col, lw) => {
      ctx1.strokeStyle = col; ctx1.lineWidth = lw; ctx1.beginPath();
      let started = false;
      arr.forEach((v, i) => {
        if (v === null || v === undefined) return;
        const x = (i / Math.max(1, arr.length - 1)) * W1;
        const y = mapY(v);
        if (!started) { ctx1.moveTo(x, y); started = true; } else ctx1.lineTo(x, y);
      });
      ctx1.stroke();
    };
    strokeArr(sVals, "#60a5fa", 2.0);
    if (hasWatch) strokeArr(wVals, "#e879f9", 1.5);
    ctx1.font = "500 10px 'Inter', system-ui, sans-serif";
    ctx1.fillStyle = "#93c5fd";
    ctx1.textAlign = "left";
    ctx1.fillText(isAcc ? "Accel (m/s²)" : "Gyro (rad/s)", 8, 13);
    ctx1.textAlign = "right";
    ctx1.fillStyle = hasWatch ? "#f0abfc" : "#8492ad";
    ctx1.fillText(hasWatch ? "Watch: Live" : "No Watch", W1 - 8, 13);
    ctx1.textAlign = "left";
  }

  // Chart 2: Sim vs Hardware Streamer Tracking Residual + Peak Joint Velocity
  const { ctx: ctx2, W: W2, H: H2 } = prepareHiDPICanvas($("chart-err"));

  const plotMetric = (getter, maxScale, col) => {
    ctx2.strokeStyle = col; ctx2.lineWidth = 1.8; ctx2.beginPath();
    hist.forEach((h, i) => {
      const x = (i / Math.max(1, hist.length - 1)) * W2;
      const y = H2 - 8 - Math.min(1, getter(h) / maxScale) * (H2 - 16);
      i ? ctx2.lineTo(x, y) : ctx2.moveTo(x, y);
    });
    ctx2.stroke();
  };
  plotMetric(h => h.oriErr4, 90, "#b99aff");
  plotMetric(h => h.oriErr6, 90, "#60a5fa");
  plotMetric(h => h.dqPeak * R2D, 140, "#cbd5e1");
}

function computeImuComparisonMetrics() {
  const hist = st.imuHistory;
  if (hist.length < 3) return;
  const paired = hist.filter(h => h.watchAccRaw && h.watchGyroRaw);

  if (paired.length < 5) {
    // No real Watch data connected or uploaded: report clear status
    if ($("m-acc-r")) $("m-acc-r").textContent = "None";
    ["m-acc-rmse", "m-acc-mae", "m-gyr-rmse", "m-gyr-mae", "m-gyr-r"].forEach(id => {
      if ($(id)) $(id).textContent = "--";
    });
    const lastH = hist[hist.length - 1];
    const simAmag = Math.hypot(...lastH.simAcc).toFixed(4);
    const simGmag = Math.hypot(...lastH.simGyro).toFixed(4);
    latestSim2RealReport =
`================================================================
ANALYTIC FOREARM IMU TELEMETRY (sim/isaac_replay.py engine)
================================================================
 sim   : n=${hist.length}  dt~16.7 ms  forearm_fk(q) + R^T(a_world + g)
 watch : NONE (load a SensorLog / Sensor Logger CSV or pair Watch)
----------------------------------------------------------------
 Latest Sim Proper Accel : [${lastH.simAcc.map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(", ")}] m/s^2 (|a|=${simAmag})
 Latest Sim Body Gyro    : [${lastH.simGyro.map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(", ")}] rad/s (|w|=${simGmag})
 Hardware Twin Streamer  : AbelArmHardwareInterface (1.0 deg deadband, 150 ms quintic)
================================================================`;
    return;
  }

  // Solve real 3x3 Wahba / Kabsch SVD rotation matrix R_watch_to_sim (compare.py:kabsch)
  const P = paired.map(h => h.watchAccRaw);
  const Q = paired.map(h => h.simAcc);
  const R = kabschSVD(P, Q);
  st.estimatedR = R;

  // Geodesic angle of solved R: arccos((tr(R) - 1) / 2)
  const tr = R[0] + R[4] + R[8];
  const geoDeg = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * R2D;

  // Rotate Watch vectors into Sim frame using solved R
  for (const h of paired) {
    h.watchAcc = applyMat3(R, h.watchAccRaw);
    h.watchGyro = applyMat3(R, h.watchGyroRaw);
  }

  const sA = paired.map(h => Math.hypot(...h.simAcc));
  const wA = paired.map(h => Math.hypot(...h.watchAcc));
  const sG = paired.map(h => Math.hypot(...h.simGyro));
  const wG = paired.map(h => Math.hypot(...h.watchGyro));

  const calc = (a, b) => {
    const n = a.length;
    let se = 0, ae = 0, ma = 0, mb = 0;
    for (let i = 0; i < n; i++) { const e = a[i] - b[i]; se += e * e; ae += Math.abs(e); ma += a[i]; mb += b[i]; }
    ma /= n; mb /= n;
    let num = 0, da = 0, db = 0;
    for (let i = 0; i < n; i++) { num += (a[i] - ma) * (b[i] - mb); da += (a[i] - ma) ** 2; db += (b[i] - mb) ** 2; }
    const r = (da > 1e-9 && db > 1e-9) ? num / Math.sqrt(da * db) : 0.0;
    return { rmse: Math.sqrt(se / n), mae: ae / n, r: Math.max(-1, Math.min(1, r)) };
  };
  const mA = calc(sA, wA), mG = calc(sG, wG);
  $("m-acc-rmse").textContent = mA.rmse.toFixed(3);
  $("m-acc-mae").textContent = mA.mae.toFixed(3);
  $("m-acc-r").textContent = mA.r.toFixed(3);
  $("m-gyr-rmse").textContent = mG.rmse.toFixed(3);
  $("m-gyr-mae").textContent = mG.mae.toFixed(3);
  $("m-gyr-r").textContent = mG.r.toFixed(3);

  latestSim2RealReport =
`================================================================
SIM vs WATCH  IMU comparison (abel_sim2real compare.py engine)
================================================================
 sim   : n=${paired.length}  analytic forearm_fk(q)
 watch : n=${paired.length}  source=${st.watchSourceName || "Live DeviceMotion"}
----------------------------------------------------------------
 FRAME R (watch -> sim), Kabsch SVD solved (geodesic ${geoDeg.toFixed(1)} deg):
   [${R.slice(0, 3).map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(" ")}]
   [${R.slice(3, 6).map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(" ")}]
   [${R.slice(6, 9).map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(" ")}]
----------------------------------------------------------------
channel   unit         RMSE        MAE   Pearson
----------------------------------------------------------------
acc_mag   m/s^2      ${mA.rmse.toFixed(4)}     ${mA.mae.toFixed(4)}     ${mA.r.toFixed(3)}
gyro_mag  rad/s      ${mG.rmse.toFixed(4)}     ${mG.mae.toFixed(4)}     ${mG.r.toFixed(3)}
================================================================`;
}

function tick(now) {
  const dt = Math.min(0.05, Math.max(0.004, (now - last) / 1000));
  last = now;

  const activeT = armTarget(st.arm[6].held ? 6 : 4);
  const prevQ4 = st.q4.slice(), prevQ6 = st.q6.slice();
  const c = now > st.holdCrowd ? crowdPose() : null;

  if (st.quinticPlan) {
    // Evaluate firmware-matching 5th-order quintic trajectory segments (servo deg -> URDF rad via (deg - 90) * pi / 180)
    const elapsed = now / 1000 - st.quinticPlan.t0;
    let accT = 0, activeSeg = null, localT = 0;
    for (const seg of st.quinticPlan.segs) {
      const span = seg.moveT + seg.settleT;
      if (elapsed <= accT + span) { activeSeg = seg; localT = elapsed - accT; break; }
      accT += span;
    }
    if (activeSeg) {
      const servoDegs = activeSeg.q0.map((q0, j) => evalQuintic(q0, activeSeg.qf[j], activeSeg.moveT, localT).pos);
      // Map [base, shoulder, elbow, gripper] servo degrees to URDF radians
      st.q4 = servoDegs.map(deg => servoDegToUrdfRad(deg));
      // Also set gripper open/close state from servo index 3 (120 = open, 60 = closed)
      if (servoDegs.length >= 4) {
        st.closed = servoDegs[3] < 80;
      }
      const fk = fk4(st.q4);
      st.cmd.x = fk.tip[0]; st.cmd.y = fk.tip[1]; st.cmd.z = fk.tip[2];
      st.q6 = smooth(st.q6, ik6(fk.tip, fk.R, st.q6).q, 0.45);
      showCmd(st.cmd);
      $("cmd-src").textContent = `${st.quinticPlan.name}: ${activeSeg.label}`;
    } else {
      st.quinticPlan = null;
      if ($("watch-status") && !st.uploadedWatchCSV) {
        $("watch-status").textContent = "Completed YAML sweep (Analytic Sim IMU active)";
      }
    }
  } else if (st.drive !== "joints") {
    if (st.tween) {
      const k = Math.min(1, (now - st.tween.t0) / st.tween.dur), e = ease(k);
      for (const key in st.tween.to) {
        const a = st.tween.from[key]; let b = st.tween.to[key];
        if (key === "yaw") { let d = b - a; while (d > 180) d -= 360; while (d < -180) d += 360; b = a + d; }
        st.cmd[key] = a + (b - a) * e;
      }
      if (k >= 1) { const then = st.tween.then; st.tween = null; then && then(); }
      $("cmd-src").textContent = "Command: Auto Trajectory";
    } else if (c) {
      const cl = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
      const gainScale = (st.studioGain || 2.4) / 2.4;
      if (c.aim === "tilt" && Math.hypot(c.vx, c.vy, c.vz) < 0.04) {
        const baseP = activeT ? activeT.p : [0.15, 0.0, 0.08];
        let targetX = cl(0.150 + (c.pitch / 48) * 0.044 * gainScale, 0.075, 0.205);
        let targetY = cl((c.yaw / 45) * 0.115 * gainScale, -0.130, 0.130);
        let targetZ = cl(0.076 + (c.pitch / 44) * 0.075 * gainScale, 0.024, 0.205);

        let wantYPR = [c.yaw * 0.85, c.pitch * 0.40, c.roll * 0.85];
        // Magnetic Target Funnel (95 mm radius): smoothly pull (x, y, z) AND tool orientation into alignment
        if (activeT) {
          const distToTgt = Math.hypot(targetX - baseP[0], targetY - baseP[1], targetZ - baseP[2]);
          if ($("auto-assist-pill")) {
            const engaged = distToTgt < 0.095;
            $("auto-assist-pill").className = engaged ? "pill ok" : "pill warn";
            $("auto-assist-pill").textContent = engaged ? `Magnet Locked (${Math.round(distToTgt * 1000)} mm)` : "Auto-Magnet: ON";
          }
          if (distToTgt < 0.095) {
            const pull = Math.pow(1 - distToTgt / 0.095, 1.25) * 0.72;
            targetX += (baseP[0] - targetX) * pull;
            targetY += (baseP[1] - targetY) * pull;
            targetZ += (baseP[2] - targetZ) * pull;
            if (activeT.ypr && st.station !== 2) {
              wantYPR = wantYPR.map((ang, idx) => ang + (activeT.ypr[idx] - ang) * pull);
            }
          }
        }
        const kp = Math.min(1, dt * 10.5);
        if (Math.abs(targetX - st.cmd.x) > 0.001) st.cmd.x += (targetX - st.cmd.x) * kp;
        if (Math.abs(targetY - st.cmd.y) > 0.001) st.cmd.y += (targetY - st.cmd.y) * kp;
        if (Math.abs(targetZ - st.cmd.z) > 0.001) st.cmd.z += (targetZ - st.cmd.z) * kp;
        for (const [i, key] of ["yaw", "pitch", "roll"].entries()) {
          let d = wantYPR[i] - st.cmd[key];
          if (key === "yaw") { while (d > 180) d -= 360; while (d < -180) d += 360; }
          if (Math.abs(d) > 1.0) st.cmd[key] += d * kp;
        }
      } else {
        st.cmd.x = cl(st.cmd.x + c.vx * SPEED * gainScale * dt, 0.05, 0.22);
        st.cmd.y = cl(st.cmd.y + c.vy * SPEED * gainScale * dt, -0.14, 0.14);
        st.cmd.z = cl(st.cmd.z + c.vz * SPEED * gainScale * dt, 0.01, 0.25);
        const want = activeT.ypr;
        const k = Math.min(1, dt * 10);
        for (const [i, key] of ["yaw", "pitch", "roll"].entries()) {
          let d = want[i] - st.cmd[key];
          if (key === "yaw") { while (d > 180) d -= 360; while (d < -180) d += 360; }
          st.cmd[key] += d * k;
        }
      }
      $("cmd-src").textContent = st.drive === "pilot" ? `Live Pilot (${st.studioGain}x Desk)` : `Live Watch/Phone (${st.studioGain}x Desk)`;
    } else {
      const hasDormantWatch = [...phones.values()].some(p => p.pose && (now - p.t) >= 2500);
      $("cmd-src").textContent = hasDormantWatch ? "Watch Dimmed (Tap Watch / Turn Crown)" : "Studio Sliders (60Hz)";
    }
    showCmd(st.cmd);
    // Rigid-Body Fixture & Table Collision Clamp:
    // Prevents the gripper from penetrating the optical table floor (z < 0.018 m),
    // the valve wheel/post assembly (Station 3 at x > 0.172 m near y=0),
    // or the charger dock panel (Station 2 at y > 0.132 m near x=0.10 m).
    let clampedCollision = false;
    if (st.cmd.z < 0.018) { st.cmd.z = 0.018; clampedCollision = true; }
    if (st.station === 2) {
      // Valve wheel plane is at x = 0.184m, radius 0.036m around (y=0, z=0.14m). Stop tool tip at x <= 0.172m when within the wheel cylinder
      const radialDistToValve = Math.hypot(st.cmd.y - 0.0, st.cmd.z - 0.14);
      if (radialDistToValve < 0.045 && st.cmd.x > 0.171) {
        st.cmd.x = 0.171;
        clampedCollision = true;
      }
      // Also prevent passing through the vertical valve support post at (x=0.20, y=0)
      if (Math.abs(st.cmd.y) < 0.022 && st.cmd.x > 0.178) {
        st.cmd.x = 0.178;
        clampedCollision = true;
      }
    } else if (st.station === 1) {
      // Charger dock backplate prevents pushing past y > 0.132m in the dock zone
      if (Math.abs(st.cmd.x - 0.10) < 0.055 && st.cmd.z < 0.16 && st.cmd.y > 0.132) {
        st.cmd.y = 0.132;
        clampedCollision = true;
      }
    }
    if (clampedCollision) showCmd(st.cmd);
    const R = poseR(st.cmd.yaw, st.cmd.pitch, st.cmd.roll), p = [st.cmd.x, st.cmd.y, st.cmd.z];
    const raw4 = ik4(p, R).q;
    const raw6 = ik6(p, R, st.q6).q;
    st.q4 = smooth(st.q4, raw4, 0.94);
    st.q6 = smooth(st.q6, raw6, 0.94);
  } else {
    for (const team of [4, 6]) {
      const qv = team === 4 ? st.q4 : st.q6, tgt = qv.slice();
      for (let j = 0; j < qv.length; j++) {
        const vals = [...phones.values()].filter(p => p.team === team && p.joints[j] !== undefined).map(p => p.joints[j]);
        if (vals.length) tgt[j] = (vals.reduce((a, b) => a + b, 0) / vals.length) * D2R;
      }
      const nq = smooth(qv, tgt, 0.65);
      if (team === 4) st.q4 = nq; else st.q6 = nq;
    }
    $("cmd-src").textContent = "Direct Joint Motors";
  }

  st.dq4 = st.q4.map((q, i) => (q - prevQ4[i]) / dt);
  st.dq6 = st.q6.map((q, i) => (q - prevQ6[i]) / dt);

  // Exact ROS2 AbelArmHardwareInterface Open-Loop Segment Streamer (abel_arm_hardware.cpp):
  // - Converts desired URDF joint radians to integer servo degrees: round(clamp(90 + rad * 180/pi, 0, 180))
  // - Applies command_deadband_deg = 1.0 deg and min_send_interval_ms = 100 ms rate limiter
  // - Executes on-device 5th-order quintic segment interpolation over segment_time_ms = 150 ms
  const prevQ6Real = st.q6Real.slice();
  const hw = st.hwStreamer;
  const desiredServoDeg = st.q6.map(r => urdfRadToServoDeg(r));
  const maxStepDeg = Math.max(...desiredServoDeg.map((d, j) => Math.abs(d - hw.lastSentServoDeg[j])));
  if (maxStepDeg >= 1.0 && (now - hw.lastSentMs) >= 100.0) {
    // Latch current interpolated servo position as q0 of new 150 ms firmware quintic segment
    const curElapsedS = Math.max(0, (now - hw.segStartMs) / 1000);
    hw.segQ0Deg = hw.segQ0Deg.map((q0, j) => evalQuintic(q0, hw.segQfDeg[j], hw.segDurS, curElapsedS).pos);
    hw.segQfDeg = desiredServoDeg.slice();
    hw.segStartMs = now;
    hw.segDurS = 0.150;
    hw.lastSentMs = now;
    hw.lastSentServoDeg = desiredServoDeg.slice();
  }
  const segElapsedS = Math.max(0, (now - hw.segStartMs) / 1000);
  st.q6Real = hw.segQ0Deg.map((q0, j) => {
    const posDeg = evalQuintic(q0, hw.segQfDeg[j], hw.segDurS, segElapsedS).pos;
    return servoDegToUrdfRad(posDeg);
  });
  st.dq6Real = st.q6Real.map((q, i) => (q - prevQ6Real[i]) / dt);

  const dqPeak = Math.max(...st.dq6.map(Math.abs), ...st.dq6Real.map(Math.abs));

  const f4 = fk4(st.q4), f6Sim = fk6(st.q6), f6Real = fk6(st.q6Real);
  const is4v6 = st.rigMode === "4d_vs_6d";
  const fLeft = is4v6 ? f4 : f6Sim;
  const fRight = is4v6 ? f6Sim : f6Real;
  const vLeft = evaluateArm(4, fLeft, dt);
  const vRight = evaluateArm(6, fRight, dt);

  // Forearm IMU analytic prediction on active forearm link (sim/isaac_replay.py:forearm_fk)
  const siteLeft = forearmSiteFK(is4v6 ? st.q4 : st.q6.slice(0, 4));
  const siteRight = forearmSiteFK(st.q6Real.slice(0, 4));
  const site = st.quinticPlan ? forearmSiteFK(st.q4) : siteRight;
  if (!prevSiteP) { prevSiteP = site.p.slice(); prevSiteR = site.R.slice(); }
  const siteVel = site.p.map((x, i) => (x - prevSiteP[i]) / dt);
  const siteAccWorld = siteVel.map((v, i) => (v - prevSiteV[i]) / dt);
  // Proper acceleration includes +g_up reaction in world frame, rotated into forearm site frame: R^T * (a_world + [0,0,G0])
  const aProperWorld = [siteAccWorld[0], siteAccWorld[1], siteAccWorld[2] + G0];
  const Rt = [site.R[0], site.R[3], site.R[6], site.R[1], site.R[4], site.R[7], site.R[2], site.R[5], site.R[8]];
  const simAcc = applyMat3(Rt, aProperWorld);
  // Body angular velocity from R^T * Rdot
  const Rdot = site.R.map((r, i) => (r - prevSiteR[i]) / dt);
  const simGyro = [
    Rt[6] * Rdot[3] + Rt[7] * Rdot[4] + Rt[8] * Rdot[5],
    Rt[0] * Rdot[6] + Rt[1] * Rdot[7] + Rt[2] * Rdot[8],
    Rt[3] * Rdot[0] + Rt[4] * Rdot[1] + Rt[5] * Rdot[2]
  ];
  prevSiteP = site.p.slice(); prevSiteV = siteVel; prevSiteR = site.R.slice();

  // Real Watch IMU source only: (a) Live Phone/Watch DeviceMotion/SensorLogger stream, or (b) Uploaded Watch CSV.
  // Never fabricate synthetic random watch noise when no Watch source is active.
  let watchAccRaw = null, watchGyroRaw = null;
  let watchAcc = null, watchGyro = null;
  if (c && c.liveImu) {
    watchAccRaw = [c.liveImu.ax, c.liveImu.ay, c.liveImu.az];
    watchGyroRaw = [c.liveImu.gx, c.liveImu.gy, c.liveImu.gz];
    watchAcc = applyMat3(st.estimatedR, watchAccRaw);
    watchGyro = applyMat3(st.estimatedR, watchGyroRaw);
    st.watchSourceName = "Live DeviceMotion / SensorLogger";
    $("watch-status").textContent = `Live DeviceMotion IMU${c.liveImu.tap ? " · TAP FIDUCIAL SYNCED" : ""}`;
  } else if (st.uploadedWatchCSV && st.uploadedWatchCSV.length) {
    const row = st.uploadedWatchCSV[st.uploadedWatchIdx % st.uploadedWatchCSV.length];
    st.uploadedWatchIdx++;
    watchAccRaw = row.acc;
    watchGyroRaw = row.gyro;
    watchAcc = applyMat3(st.estimatedR, watchAccRaw);
    watchGyro = applyMat3(st.estimatedR, watchGyroRaw);
  }

  // 6-Axis Complementary Filter Attitude Fusion (uses Watch IMU when present, else Analytic Sim IMU)
  const attAcc = watchAcc || simAcc;
  const attGyr = watchGyro || simGyro;
  const accPitch = Math.atan2(-attAcc[0], Math.hypot(attAcc[1], attAcc[2])) * R2D;
  const accRoll = Math.atan2(attAcc[1], attAcc[2]) * R2D;
  const alpha = 0.96;
  st.fusedPitchDeg = alpha * (st.fusedPitchDeg + attGyr[1] * R2D * dt) + (1 - alpha) * accPitch;
  st.fusedRollDeg = alpha * (st.fusedRollDeg + attGyr[0] * R2D * dt) + (1 - alpha) * accRoll;

  const utcNowMs = Date.now() + st.utcOffsetMs;
  const utcIso = new Date(utcNowMs).toISOString();

  // Sim-to-Real 6-DOF tip tracking error (or 4D vs 6D divergence when in 4d_vs_6d mode)
  const sim2realPosMm = +(Math.hypot(
    fLeft.tip[0] - fRight.tip[0],
    fLeft.tip[1] - fRight.tip[1],
    fLeft.tip[2] - fRight.tip[2]
  ) * 1000).toFixed(1);
  const sim2realOriDeg = +oriError(fLeft.R, fRight.R).toFixed(1);

  st.imuHistory.push({
    t: now / 1000,
    utcIso,
    utcEpochMs: Math.round(utcNowMs),
    simAcc, simGyro,
    watchAccRaw, watchGyroRaw,
    watchAcc, watchGyro,
    fusedPitchDeg: +st.fusedPitchDeg.toFixed(2),
    fusedRollDeg: +st.fusedRollDeg.toFixed(2),
    posErr4: vLeft.posMm, posErr6: vRight.posMm,
    oriErr4: vLeft.oriDeg, oriErr6: is4v6 ? vRight.oriDeg : sim2realOriDeg,
    dqPeak
  });
  if (st.imuHistory.length > 180) st.imuHistory.shift();

  // Update 3D Rigs
  st.openVis += ((st.closed ? 0 : 1) - st.openVis) * Math.min(1, dt * 8);
  const makeProps = (slot, v, watchSiteObj) => {
    const a = st.arm[slot], tgt = armTarget(slot);
    return {
      station: st.station, open: st.openVis, aligned: v.aligned,
      target: { p: tgt.p, R: tgt.R, open: a.held ? 0 : 1 },
      watchSite: watchSiteObj,
      battery: { held: a.held, inserted: a.inserted, rest: { p: a.battP, R: a.battR } },
      charger: { p: ST[1].p, R: ST[1].R, lit: a.done[1] },
      valve: { p: ST[2].p, angle: a.valve * D2R }
    };
  };
  updateRig(4, fLeft, makeProps(4, vLeft, is4v6 ? siteLeft : null));
  updateRig(6, fRight, makeProps(6, vRight, siteRight));

  syncViewportSize();
  controls.update();
  renderer.render(scene, camera);
  latestBBoxes = computeBBoxes();

  // Build LeRobot / RLDS step record with UTC synchronization + 6-Axis Sensor Fusion
  const wLeft = is4v6 ? manipulability(st.q4, 4) : manipulability(st.q6, 6);
  const wRight = manipulability(is4v6 ? st.q6 : st.q6Real, 6);
  latestStepRecord = {
    step_index: stepIdx++,
    timestamp_s: +(now / 1000).toFixed(4),
    utc_iso: utcIso,
    utc_epoch_ms: Math.round(utcNowMs),
    utc_sync_offset_ms: st.utcOffsetMs,
    rig_mode: st.rigMode,
    station_id: ST[st.station].id,
    observation: {
      q4_rad: st.q4.map(v => +v.toFixed(4)),
      q6_sim_rad: st.q6.map(v => +v.toFixed(4)),
      q6_real_rad: st.q6Real.map(v => +v.toFixed(4)),
      dq6_sim_rads: st.dq6.map(v => +v.toFixed(4)),
      dq6_real_rads: st.dq6Real.map(v => +v.toFixed(4)),
      tip_left_xyz_m: fLeft.tip.map(v => +v.toFixed(4)),
      tip_right_xyz_m: fRight.tip.map(v => +v.toFixed(4)),
      gripper_closed: st.closed ? 1 : 0,
      manipulability: { w_left: wLeft, w_right: wRight },
      forearm_imu: {
        sim_acc_ms2: simAcc.map(v => +v.toFixed(3)),
        sim_gyro_rads: simGyro.map(v => +v.toFixed(3)),
        watch_acc_ms2: watchAcc ? watchAcc.map(v => +v.toFixed(3)) : null,
        watch_gyro_rads: watchGyro ? watchGyro.map(v => +v.toFixed(3)) : null,
        fused_attitude_deg: { pitch: +st.fusedPitchDeg.toFixed(2), roll: +st.fusedRollDeg.toFixed(2) }
      }
    },
    action: {
      target_xyz_m: [+st.cmd.x.toFixed(4), +st.cmd.y.toFixed(4), +st.cmd.z.toFixed(4)],
      target_ypr_deg: [Math.round(st.cmd.yaw), Math.round(st.cmd.pitch), Math.round(st.cmd.roll)],
      gripper_cmd: st.closed ? 1 : 0
    },
    sim2real_gap: {
      tip_tracking_residual_mm: sim2realPosMm,
      tip_orientation_residual_deg: sim2realOriDeg,
      left_target_pos_err_mm: vLeft.posMm,
      right_target_pos_err_mm: vRight.posMm
    }
  };

  if (st.recording && now - lastRec >= 33) {
    lastRec = now;
    st.episode.push(latestStepRecord);
    updateRecBadge();
  }

  // Kinematic Singularity & Envelope Monitor (Yoshikawa manipulability w = sqrt(det(J*J^T)) + tracking residual)
  const wRatio = Math.min(0.99, Math.max(0.15, wRight / 0.145));
  if (wRight < 0.035 || dqPeak > 2.2) {
    $("nemo-tier").className = "pill warn";
    $("nemo-tier").textContent = "Gimbal Warn";
  } else {
    $("nemo-tier").className = "pill ok";
    $("nemo-tier").textContent = "Healthy";
  }

  if (now - lastUi > 180) {
    lastUi = now;
    if ($("utc-pill")) {
      const sign = st.utcOffsetMs >= 0 ? "+" : "";
      $("utc-pill").innerHTML = `<svg class="ico sm" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><polyline points="12 7 12 12 15 15"/></svg><span>${utcIso.slice(11, 23)}Z · Δ${sign}${st.utcOffsetMs.toFixed(1)}ms</span>`;
    }
    if ($("watch-lag-pill")) {
      const sign = st.utcOffsetMs >= 0 ? "+" : "";
      $("watch-lag-pill").textContent = `Sync: ${sign}${st.utcOffsetMs.toFixed(1)} ms`;
    }
    if ($("fusion-att")) {
      $("fusion-att").textContent = `Pitch ${st.fusedPitchDeg.toFixed(1)}° · Roll ${st.fusedRollDeg.toFixed(1)}°`;
    }
    const qLeftArr = is4v6 ? st.q4 : st.q6;
    const qRightArr = is4v6 ? st.q6 : st.q6Real;
    $("j4-str").textContent = qLeftArr.map(a => `${Math.round(a * R2D)}°`).join(", ");
    $("j6-str").textContent = qRightArr.map(a => `${Math.round(a * R2D)}°`).join(", ");
    $("verdict-4").innerHTML = `<span class="pill ${vLeft.cls}">${vLeft.msg}</span>`;
    $("verdict-6").innerHTML = `<span class="pill ${vRight.cls}">${vRight.msg}</span>`;
    $("m-dof-pos").textContent = is4v6 ? `${vLeft.posMm}/${vRight.posMm}mm` : `${sim2realPosMm} mm`;
    $("m-dof-ori").textContent = is4v6 ? `${vLeft.oriDeg}°/${vRight.oriDeg}°` : `${sim2realOriDeg}°`;
    $("m-dof-w").textContent = `w=${wLeft.toFixed(2)}/${wRight.toFixed(2)}`;
    if ($("s2r-verdict-pill")) {
      const lagHigh = sim2realPosMm > 8.0 || sim2realOriDeg > 4.0;
      $("s2r-verdict-pill").className = `pill ${lagHigh ? "warn" : "ok"}`;
      $("s2r-verdict-pill").textContent = is4v6
        ? `Diff: ${Math.abs(vLeft.oriDeg - vRight.oriDeg).toFixed(0)}°`
        : lagHigh
          ? `Lagging (${sim2realPosMm} mm)`
          : `Synced (${sim2realPosMm} mm)`;
    }
    $("div-readout").textContent = is4v6
      ? `${Math.abs(vLeft.oriDeg - vRight.oriDeg).toFixed(0)}° · ${Math.round(dqPeak * R2D)}°/s`
      : `${sim2realPosMm}mm / ${sim2realOriDeg}°`;
    $("nemo-prob").textContent = wRatio.toFixed(2);
    $("nemo-cost").textContent = `w=${wRight.toFixed(3)}`;

    computeImuComparisonMetrics();
    drawCharts();

    if (st.schemaTab === "lerobot") {
      $("schema-preview").textContent = JSON.stringify(latestStepRecord, null, 2);
    } else if (st.schemaTab === "usd") {
      $("schema-preview").textContent = buildOpenUSD(latestStepRecord, st.episode.slice(-4));
    } else if (st.schemaTab === "coco") {
      $("schema-preview").textContent = JSON.stringify({ aov: st.aov, bboxes_3d_2d: latestBBoxes }, null, 2);
    } else if (st.schemaTab === "yaml") {
      $("schema-preview").textContent = buildYamlContractPreview();
    } else {
      $("schema-preview").textContent = latestSim2RealReport;
    }

    // Send live hints & scores back to connected phones
    const s0 = ST[st.station];
    send({ t: "hint", h4: vLeft, h6: vRight, station: s0.icon });
    send({ t: "score", e4: { pos: vLeft.posMm, ori: vLeft.oriDeg }, e6: { pos: vRight.posMm, ori: vRight.oriDeg } });
  }

  requestAnimationFrame(tick);
}

fetchTrajectoriesFromServer();
setRigMode("6d_sim2real");
setStation(0);
requestAnimationFrame(tick);
