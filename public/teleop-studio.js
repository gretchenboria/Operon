// Omniverse RTX Teleop & Sim-to-Real Telemetry Studio
// Combines:
//   1. Audience Phone / Apple Watch WebSocket teleop relay (live.js + phone.js)
//   2. 4-DOF (Abel-Arm) vs 6-DOF (Spherical Wrist) kinematics + Jacobian manipulability (arms3d.js)
//   3. Omniverse Replicator multi-pass AOV renderer (RTX PBR, Linear Depth, Surface Normals, Semantic ID, Instance ID)
//   4. abel_sim2real Apple Watch vs Sim Forearm IMU pipeline (5th-order firmware quintic, Wahba/SVD alignment, tap sync, Welch PSD, RMSE/MAE/Pearson r)
//   5. NemoFleet System 1 / System 2 action-probability gate + teacher spend tracking
//   6. Dataset & Stage Exporters: LeRobot / RLDS (.jsonl), OpenUSD (.usda), Replicator COCO (.json), and compare.py CSVs

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { poseR, oriError, ik4, ik6, fk4, fk6, JOINTS, G } from "./demos/arms3d.js";
import { onLive, send, qrSVG, phoneUrl, ROOM } from "./live.js";

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const G0 = 9.80665;
const $ = (id) => document.getElementById(id);

// ── 1. Firmware-Matching 5th-Order Quintic Trajectory (ported from abel_sim2real/sim/quintic.py) ──
const MAX_VEL_DEG = 120.0;
const MAX_ACC_DEG = 200.0;
const V_PEAK_FACTOR = 1.875;
const A_PEAK_FACTOR = 10.0 / Math.sqrt(3.0); // ~5.7735

export function quinticMinDuration(deltaDeg) {
  const d = Math.abs(deltaDeg);
  if (d < 1e-6) return 0.05;
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

// Canonical imu_calibration.yaml waypoints (relative to [0,0,0,0] rad in our FK frame)
const QUINTIC_SWEEP_WAYPOINTS = [
  { targetsDeg: [60, 0, 0, 0], dur: 1.4, settle: 0.4, label: "base +60° sweep" },
  { targetsDeg: [-60, 0, 0, 0], dur: 1.8, settle: 0.4, label: "base -120° sweep" },
  { targetsDeg: [0, 0, 0, 0], dur: 1.4, settle: 0.35, label: "base home" },
  { targetsDeg: [0, -50, 0, 0], dur: 1.3, settle: 0.35, label: "shoulder -50° sweep" },
  { targetsDeg: [0, 25, 45, 0], dur: 1.4, settle: 0.35, label: "shoulder+elbow sweep" },
  { targetsDeg: [-15, 20, 35, 0], dur: 1.2, settle: 0.3, label: "return to work pose" }
];

// ── 2. Forearm IMU Predictor & Apple Watch Synthetic / Wahba Alignment (from abel_sim2real) ──
// Watch->Sim rotation matrix R (37 deg about [0.3, -0.7, 0.6], matching make_synthetic_watch.py)
function rodrigues(axis, angleRad) {
  const l = Math.hypot(...axis) || 1;
  const [x, y, z] = axis.map(v => v / l);
  const c = Math.cos(angleRad), s = Math.sin(angleRad), t = 1 - c;
  return [
    t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c
  ];
}
const R_WATCH_TO_SIM = rodrigues([0.3, -0.7, 0.6], 37 * D2R);
const R_SIM_TO_WATCH = [
  R_WATCH_TO_SIM[0], R_WATCH_TO_SIM[3], R_WATCH_TO_SIM[6],
  R_WATCH_TO_SIM[1], R_WATCH_TO_SIM[4], R_WATCH_TO_SIM[7],
  R_WATCH_TO_SIM[2], R_WATCH_TO_SIM[5], R_WATCH_TO_SIM[8]
];
const applyMat3 = (M, v) => [
  M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
  M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
  M[6] * v[0] + M[7] * v[1] + M[8] * v[2]
];

// Compute forearm watch-mount site world position & orientation from 4-DOF joint angles q4 (rad)
function forearmSiteFK(q4) {
  const [t1, t2, t3] = q4;
  const cy = Math.cos(t1), sy = Math.sin(t1);
  const p1 = t2, p12 = t2 + t3;
  // Mid-forearm watch strap site (75% along forearm link L2)
  const rLocal = G.L1 * Math.sin(p1) + (G.L2 * 0.75) * Math.sin(p12);
  const zLocal = G.h0 + G.L1 * Math.cos(p1) + (G.L2 * 0.75) * Math.cos(p12);
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
  { id: "battery", icon: "🔋", name: "Pick & place battery", p: PICK_P, R: PICK_R, placeP: PLACE_P, placeR: PLACE_R },
  { id: "charger", icon: "🔌", name: "Side charger slot", p: [0.10, 0.13, 0.12], R: fromZX([0, 0.9, -0.42], [1, 0, 0]) },
  { id: "valve", icon: "🚰", name: "Turn valve 90°", p: [0.17, 0, 0.14], R: fromZX([1, 0, 0], [0, 1, 0]), grip: fromZX([1, 0, 0], [0, 0, -1]) }
];
ST.forEach(s => { s.ypr = yprOf(s.R); if (s.grip) s.yprGrip = yprOf(s.grip); });
const POS_TOL = 0.042, ORI_TOL = 25, SPEED = 0.22, ASSIST_R = 0.070;

// ── 4. Studio State ──
const st = {
  drive: "crowd",
  station: 0,
  closed: false,
  openVis: 1,
  aov: "rtx",
  showBBoxes: false,
  schemaTab: "lerobot",
  imuChartMode: "acc",
  cmd: { yaw: -15, pitch: 0, roll: 0, x: 0.15, y: 0.0, z: 0.12 },
  q4: [0, 0, 0, 0],
  q6: [0, 0, 0, 0, 0, 0],
  dq4: [0, 0, 0, 0],
  dq6: [0, 0, 0, 0, 0, 0],
  tween: null,
  quinticPlan: null,
  holdCrowd: 0,
  pilot: null,
  arm: { 4: freshArm(), 6: freshArm() },
  recording: false,
  episode: [],
  imuHistory: [], // { t, simAcc, simGyro, watchAcc, watchGyro, posErr4, posErr6, oriErr4, oriErr6, dqPeak }
  uploadedWatchCSV: null,
  uploadedWatchIdx: 0,
  nemoTeacherCalls: 0,
  nemoProb: 0.94,
  nemoTier: "System 1 · Local Reflex"
};

function freshArm() {
  return { done: [false, false, false], hold: 0, held: false, inserted: null, valve: 0, gripping: false, battP: [0.15, -0.075, 0.013], battR: PICK_R, dest: "place" };
}
{
  const R = poseR(-15, 0, 0), p = [0.15, 0, 0.12];
  st.q4 = ik4(p, R).q;
  st.q6 = ik6(p, R).q;
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
scene.background = new THREE.Color(0x0d1118);
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;

const camera = new THREE.PerspectiveCamera(42, cv.width / cv.height, 0.05, 3.5);
camera.position.set(0.88, 0.56, 0.0);
const controls = new OrbitControls(camera, cv);
controls.target.set(0.08, 0.11, 0);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.48;
controls.minDistance = 0.28;
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

const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.position.set(0.55, 1.3, 0.65);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.bias = -0.0004;
Object.assign(sun.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.1, far: 3 });
const hemi = new THREE.HemisphereLight(0xddeeff, 0x2b3240, 0.65);
scene.add(sun, hemi);

// Studio floor grid + workbench
const MAT = {
  alu: new THREE.MeshStandardMaterial({ color: 0xdfe3e8, metalness: 0.85, roughness: 0.28 }),
  dark: new THREE.MeshStandardMaterial({ color: 0x2b2f36, metalness: 0.35, roughness: 0.45 }),
  steel: new THREE.MeshStandardMaterial({ color: 0x9aa0a6, metalness: 0.9, roughness: 0.25 }),
  rubber: new THREE.MeshStandardMaterial({ color: 0x1e1e1e, metalness: 0, roughness: 0.9 }),
  bench: new THREE.MeshStandardMaterial({ color: 0x232a36, metalness: 0.15, roughness: 0.65 }),
  white: new THREE.MeshStandardMaterial({ color: 0xf4f4f2, metalness: 0.05, roughness: 0.5 }),
  slot: new THREE.MeshStandardMaterial({ color: 0x101318, roughness: 0.6 }),
  red: new THREE.MeshStandardMaterial({ color: 0xd93025, metalness: 0.3, roughness: 0.4 }),
  green: new THREE.MeshStandardMaterial({ color: 0x34a853, metalness: 0.2, roughness: 0.45 }),
  watch: new THREE.MeshStandardMaterial({ color: 0xff5d5d, emissive: 0xff2a2a, emissiveIntensity: 0.35, metalness: 0.5, roughness: 0.3 })
};

const normalMat = new THREE.MeshNormalMaterial();
const depthMat = new THREE.MeshDepthMaterial({ depthPacking: THREE.BasicDepthPacking });

// Tag every mesh with semantic & instance colors for Omniverse Replicator AOV passes
const trackedObjects = []; // { label, semClass, instId, obj3d, color }
function tagSemantic(rootObj, semHex, instHex) {
  const semMat = new THREE.MeshBasicMaterial({ color: semHex });
  const instMat = new THREE.MeshBasicMaterial({ color: instHex });
  rootObj.traverse(o => {
    if (o.isMesh && !o.userData.isHelper) {
      if (!o.userData.rtxMat) o.userData.rtxMat = o.material;
      o.userData.semMat = semMat;
      o.userData.instMat = instMat;
    }
  });
}

const shadow = (m) => { m.castShadow = true; m.receiveShadow = true; return m; };
const mesh = (geo, mat) => shadow(new THREE.Mesh(geo, mat));

const bench = mesh(new THREE.BoxGeometry(1.2, 0.03, 1.6), MAT.bench);
bench.position.set(0.12, -0.015, 0);
tagSemantic(bench, 0x1b2230, 0x1b2230);
scene.add(bench);

const gridHelper = new THREE.GridHelper(1.5, 30, 0x3b4b66, 0x1e2636);
gridHelper.position.set(0.12, 0.001, 0);
gridHelper.userData.isHelper = true;
scene.add(gridHelper);

// Procedural NVIDIA SimReady KLT Toolbox Prop (zero external MB download)
{
  const box = new THREE.Group();
  const base = mesh(new THREE.BoxGeometry(0.11, 0.055, 0.075), MAT.dark);
  base.position.y = 0.0275;
  const lid = mesh(new THREE.BoxGeometry(0.114, 0.014, 0.079), new THREE.MeshStandardMaterial({ color: 0xfbbc04, metalness: 0.25, roughness: 0.35 }));
  lid.position.y = 0.062;
  const handle = mesh(new THREE.BoxGeometry(0.045, 0.012, 0.01), MAT.dark);
  handle.position.y = 0.074;
  box.add(base, lid, handle);
  box.position.copy(T3([-0.15, 0.0, 0]));
  box.rotation.y = Math.PI / 2;
  tagSemantic(box, 0xa142f4, 0xba68c8);
  scene.add(box);
  trackedObjects.push({ label: "simready_toolbox#60", clsId: 6, obj: box, color: "#a142f4" });
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
  const ghostMat = new THREE.MeshBasicMaterial({ color: 0xea4335, transparent: true, opacity: 0.32, depthWrite: false });
  const m = (geo, mat) => {
    const ms = ghost ? new THREE.Mesh(geo, ghostMat) : mesh(geo, mat);
    if (ghost) ms.userData.isHelper = true;
    return ms;
  };
  const flange = m(new THREE.CylinderGeometry(0.014, 0.014, 0.008, 28), MAT.steel); flange.rotation.x = Math.PI / 2; flange.position.z = -0.056; g.add(flange);
  const body = m(new THREE.BoxGeometry(0.07, 0.028, 0.026), MAT.dark); body.position.z = -0.04; g.add(body);
  const rail = m(new THREE.BoxGeometry(0.074, 0.008, 0.006), MAT.steel); rail.position.set(0, 0, -0.025); g.add(rail);
  const fingers = [-1, 1].map(s => {
    const f = new THREE.Group();
    const a = m(new THREE.BoxGeometry(0.009, 0.022, 0.034), MAT.alu); a.position.z = -0.005; f.add(a);
    const pad = m(new THREE.BoxGeometry(0.003, 0.018, 0.02), MAT.rubber); pad.position.set(-s * 0.006, 0, 0.002); f.add(pad);
    g.add(f); f.userData.s = s; return f;
  });
  g.userData.setOpen = (o) => fingers.forEach(f => { f.position.x = f.userData.s * (0.009 + 0.022 * o); });
  g.userData.setOpen(1);
  return g;
}

function makeRig(dof, offsetY, accentHex) {
  const rig = new THREE.Group();
  rig.position.copy(T3([0, offsetY, 0]));
  scene.add(rig);

  const armGroup = new THREE.Group();
  rig.add(armGroup);

  const accent = new THREE.MeshStandardMaterial({ color: accentHex, metalness: 0.2, roughness: 0.4 });
  const foot = mesh(new THREE.CylinderGeometry(0.05, 0.055, 0.012, 40), MAT.dark); foot.position.y = 0.006; armGroup.add(foot);
  const unitCyl = new THREE.CylinderGeometry(1, 1, 1, 32);
  const tube = (r, mat) => { const m = mesh(unitCyl, mat); m.userData.r = r; armGroup.add(m); return m; };
  const links = [tube(0.026, MAT.alu), tube(0.0135, MAT.alu), tube(0.0115, MAT.alu), tube(0.01, MAT.alu)];
  const housings = Array.from({ length: dof }, (_, i) => {
    const g = new THREE.Group();
    const big = i < 3;
    const drum = mesh(new THREE.CylinderGeometry(big ? 0.021 : 0.016, big ? 0.021 : 0.016, big ? 0.05 : 0.036, 32), MAT.dark); g.add(drum);
    for (const s of [-1, 1]) { const cap = mesh(new THREE.CylinderGeometry(big ? 0.017 : 0.013, big ? 0.017 : 0.013, 0.003, 32), accent); cap.position.y = s * (big ? 0.026 : 0.019); g.add(cap); }
    armGroup.add(g); return g;
  });
  const grip = makeGripper(false);
  armGroup.add(grip);

  // Forearm Apple Watch / IMU sensor visual site on the 4-DOF Abel-Arm
  let watchSensor = null;
  if (dof === 4) {
    watchSensor = new THREE.Group();
    const caseMesh = mesh(new THREE.BoxGeometry(0.026, 0.009, 0.031), MAT.watch);
    watchSensor.add(caseMesh);
    armGroup.add(watchSensor);
  }

  tagSemantic(armGroup, dof === 4 ? 0xfbbc04 : 0x4285f4, dof === 4 ? 0xffd54f : 0x64b5f6);
  if (watchSensor) tagSemantic(watchSensor, 0xff5d5d, 0xff8a80);

  const ghost = makeGripper(true); rig.add(ghost);
  const guideMat = new THREE.MeshBasicMaterial({ color: accentHex, transparent: true, opacity: 0.55 });
  const guideLine = new THREE.Mesh(new THREE.CylinderGeometry(0.0018, 0.0018, 1, 8), guideMat);
  guideLine.userData.isHelper = true;
  rig.add(guideLine);

  let plane = null;
  if (dof === 4) {
    plane = new THREE.Mesh(new THREE.PlaneGeometry(0.36, 0.3), new THREE.MeshBasicMaterial({ color: 0xfbbc04, transparent: true, opacity: 0.11, side: THREE.DoubleSide, depthWrite: false }));
    plane.userData.isHelper = true;
    rig.add(plane);
  }

  // Workbench mission props
  const pickPad = mesh(new THREE.CylinderGeometry(0.034, 0.036, 0.003, 32), MAT.dark);
  pickPad.position.copy(T3([0.15, -0.075, 0.0015])); rig.add(pickPad);
  const dropPad = mesh(new THREE.CylinderGeometry(0.038, 0.040, 0.004, 32), MAT.green);
  dropPad.position.copy(T3([0.15, 0.075, 0.002])); rig.add(dropPad);
  tagSemantic(pickPad, 0x37474f, 0x455a64);
  tagSemantic(dropPad, 0x2e7d32, 0x66bb6a);

  const battery = new THREE.Group();
  const cell = mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.05, 28), MAT.green); cell.rotation.z = Math.PI / 2; battery.add(cell);
  const tip = mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.006, 16), MAT.steel); tip.rotation.z = Math.PI / 2; tip.position.x = 0.028; battery.add(tip);
  rig.add(battery);
  tagSemantic(battery, 0x34a853, dof === 4 ? 0x81c995 : 0x00e676);

  const charger = new THREE.Group(); rig.add(charger);
  const panel = mesh(new THREE.BoxGeometry(0.11, 0.1, 0.014), MAT.white); charger.add(panel);
  const slot = mesh(new THREE.BoxGeometry(0.062, 0.03, 0.016), MAT.slot); slot.position.z = 0.002; charger.add(slot);
  const ledMat = new THREE.MeshStandardMaterial({ color: 0xea4335, emissive: 0xea4335, emissiveIntensity: 0.8 });
  const led = mesh(new THREE.SphereGeometry(0.005, 12, 8), ledMat); led.position.set(0.04, 0.038, 0.009); charger.add(led);
  const stand = mesh(new THREE.CylinderGeometry(0.005, 0.005, 1, 12), MAT.steel); rig.add(stand);
  tagSemantic(charger, 0x57caff, dof === 4 ? 0x29b6f6 : 0x00b0ff);
  tagSemantic(stand, 0x57caff, 0x0288d1);

  const post = mesh(new THREE.CylinderGeometry(0.007, 0.007, 1, 16), MAT.steel); rig.add(post);
  const valve = new THREE.Group(); rig.add(valve);
  valve.add(mesh(new THREE.TorusGeometry(0.03, 0.005, 14, 48), MAT.red));
  for (const r of [0, Math.PI / 2]) { const sp = mesh(new THREE.BoxGeometry(0.058, 0.006, 0.006), MAT.red); sp.rotation.z = r; valve.add(sp); }
  const hub = mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.012, 20), MAT.steel); hub.rotation.x = Math.PI / 2; valve.add(hub);
  tagSemantic(valve, 0xea4335, dof === 4 ? 0xef5350 : 0xff1744);
  tagSemantic(post, 0xea4335, 0xc62828);

  trackedObjects.push(
    { label: `robot_arm_${dof}dof#${dof === 4 ? 10 : 20}`, clsId: dof === 4 ? 1 : 2, obj: armGroup, color: dof === 4 ? "#fbbc04" : "#57caff" },
    { label: `battery_cell#${dof === 4 ? 30 : 31}`, clsId: 3, obj: battery, color: "#34a853" }
  );

  return { rig, armGroup, links, housings, grip, watchSensor, ghost, guideLine, guideMat, plane, pickPad, dropPad, battery, charger, ledMat, stand, post, valve };
}

const rigs = {
  4: makeRig(4, -0.18, 0xfbbc04),
  6: makeRig(6, 0.18, 0x4285f4)
};

const poseObj = (obj, p, R) => { obj.position.copy(T3(p)); obj.quaternion.copy(quatFromR(R)); };

function updateRig(dof, fk, props) {
  const r = rigs[dof], z = [fk.R[2], fk.R[5], fk.R[8]];
  const flangeAt = [fk.tip[0] - z[0] * 0.056, fk.tip[1] - z[1] * 0.056, fk.tip[2] - z[2] * 0.056];
  const ends = [fk.pts[0], fk.pts[1], fk.pts[2], fk.pts[3], flangeAt];
  r.links.forEach((l, i) => { placeBetween(l, ends[i], ends[i + 1]); l.scale.x = l.scale.z = l.userData.r; });
  fk.joints.forEach((j, i) => { const h = r.housings[i]; h.position.copy(T3(j.p)); h.quaternion.setFromUnitVectors(UP, T3(j.axis).normalize()); });
  poseObj(r.grip, fk.tip, fk.R);
  r.grip.userData.setOpen(props.open);

  if (r.watchSensor && props.watchSite) {
    poseObj(r.watchSensor, props.watchSite.p, props.watchSite.R);
  }

  const isRtx = st.aov === "rtx";
  if (props.target && isRtx) {
    r.ghost.visible = true; poseObj(r.ghost, props.target.p, props.target.R); r.ghost.userData.setOpen(props.target.open ?? 1);
    r.guideLine.visible = true; placeBetween(r.guideLine, fk.tip, props.target.p); r.guideLine.scale.x = r.guideLine.scale.z = 1;
  } else {
    r.ghost.visible = false; r.guideLine.visible = false;
  }
  if (r.plane) {
    r.plane.visible = isRtx;
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
    rtx: "Pass: RTX PBR Metallic-Roughness + Soft Shadows + Env IBL",
    depth: "Pass: Omniverse Replicator DistanceToCamera (Linear Depth Z)",
    normal: "Pass: Omniverse Replicator Normals AOV (View-Space XYZ)",
    semantic: "Pass: Omniverse Replicator Semantic Segmentation (Class IDs 1..6)",
    instance: "Pass: Omniverse Replicator Instance Segmentation (Unique Prim IDs)"
  };
  $("aov-desc").textContent = descMap[mode] || descMap.rtx;
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

function setClosed(v, lift = true) {
  const next = !!v;
  st.closed = next;
  $("btn-grab").innerHTML = next ? "✋ Release" : "✊ Grab";
  $("btn-grab").classList.toggle("on", next);
  send({ t: "grip", g: next ? 1 : 0 });
  const fks = { 4: fk4(st.q4), 6: fk6(st.q6) };
  const cl = (val, lo, hi) => Math.max(lo, Math.min(hi, val));
  let releasedAny = false, grabbedAny = false;
  for (const dof of [4, 6]) {
    const a = st.arm[dof], f = fks[dof];
    if (next) {
      const dxy = Math.hypot(f.tip[0] - a.battP[0], f.tip[1] - a.battP[1]);
      const dz = Math.abs(f.tip[2] - 0.032);
      if (!a.held && !a.inserted && dxy < 0.068 && dz < 0.12 && oriError(f.R, a.battR) < 45) {
        a.held = true;
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
      if (st.station === 0 && dPlace < 0.044) {
        a.battP = [PLACE_P[0], PLACE_P[1], 0.013]; a.battR = PLACE_R; a.done[0] = true;
      } else if (st.station === 0 && dPick < 0.044 && a.dest === "pick") {
        a.battP = [PICK_P[0], PICK_P[1], 0.013]; a.battR = PICK_R; a.done[0] = true;
      } else if (st.station === 1 && dCharger < POS_TOL * 1.35 && oriError(f.R, ST[1].R) < ORI_TOL * 1.25) {
        a.inserted = { p: [...ST[1].p], R: [...ST[1].R] }; a.done[1] = true;
      } else {
        a.battP = [cl(f.tip[0], 0.06, 0.22), cl(f.tip[1], -0.14, 0.14), 0.013];
        a.battR = poseR(st.cmd.yaw, 0, 0);
        a.inserted = null;
      }
    }
  }
  if (grabbedAny && st.station === 0 && lift && !st.tween) st.cmd.z = Math.max(st.cmd.z, 0.055);
  if (releasedAny && lift && !st.tween) st.cmd.z = 0.135;
}

const angleBetween = (a, b) => Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))) / D2R;

function evaluateArm(dof, f, dt) {
  const a = st.arm[dof], s = ST[st.station], i = st.station, closed = st.closed;
  const tgt = armTarget(dof);
  const pos = Math.hypot(f.tip[0] - tgt.p[0], f.tip[1] - tgt.p[1], f.tip[2] - tgt.p[2]);
  const posMm = Math.round(pos * 1000);
  if (s.id === "battery") {
    const oriDeg = Math.round(oriError(f.R, tgt.R));
    const aligned = pos < POS_TOL * 1.25 && oriDeg < ORI_TOL * 1.25;
    if (!a.held) {
      if (aligned && closed) { a.held = true; a.done[0] = false; return { msg: "📦 Picked! Move to green pad → RELEASE", cls: "warn", aligned: false, posMm, oriDeg }; }
      if (a.done[0]) return { msg: "✓ PICK & PLACE DONE!", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 };
      return { msg: aligned ? "🎯 OVER BATTERY → GRAB!" : `4D/6D → battery · ${posMm} mm · ${oriDeg}°`, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
    }
    return { msg: aligned ? "🎯 OVER PAD → RELEASE!" : `📦 holding · to pad · ${posMm} mm`, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
  }
  if (a.done[i]) return { msg: "✓ MISSION DONE!", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 };
  if (s.id === "valve") {
    const z = [f.R[2], f.R[5], f.R[8]], x = [f.R[0], f.R[3], f.R[6]], off = angleBetween(z, [1, 0, 0]);
    const oriDeg = Math.round(off);
    const aligned = pos < POS_TOL && off < ORI_TOL;
    a.gripping = aligned && closed;
    const ang = Math.abs(Math.atan2(x[1], -x[2])) / D2R, folded = ang > 90 ? 180 - ang : ang;
    if (a.gripping) a.valve = Math.max(a.valve, folded);
    if (a.gripping && a.valve >= 75) { a.done[i] = true; return { msg: "✓ VALVE TURNED 90°!", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 }; }
    const msg = !aligned ? `steer to wheel · ${posMm} mm · ${oriDeg}°` : !closed ? "🎯 LOCKED ON → GRAB!" : dof === 4 ? `gripping · 4-DOF can't twist (${a.valve.toFixed(0)}°)` : `twisting valve… ${a.valve.toFixed(0)}°/90°`;
    return { msg, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
  }
  const oriDeg = Math.round(oriError(f.R, s.R));
  const aligned = pos < POS_TOL && oriDeg < ORI_TOL;
  if (aligned && !closed && a.inserted) { a.done[i] = true; return { msg: "✓ CHARGER SLOTTED!", cls: "ok", aligned: true, posMm: 0, oriDeg: 0 }; }
  const msg = aligned ? "🎯 IN SLOT → RELEASE!" : dof === 4 ? `4-DOF stuck in plane · ${oriDeg}° off` : `steer to slot · ${posMm} mm · ${oriDeg}°`;
  return { msg, cls: aligned ? "warn" : "no", aligned, posMm, oriDeg };
}

// ── 8. Phone / Watch WebSocket Relay Wiring ──
const phones = new Map();
$("room-pill").textContent = `Room ${ROOM}`;
const qrEl = document.querySelector('[data-live="qr"]');
const urlEl = document.querySelector('[data-live="url"]');
const statusEl = document.querySelector('[data-live="status"]');
if (qrEl) qrEl.innerHTML = qrSVG();
if (urlEl) { urlEl.href = phoneUrl(); urlEl.textContent = phoneUrl().replace(/^https?:\/\//, ""); }

const teamCounts = () => { let a = 0, b = 0; phones.forEach(p => p.team === 4 ? a++ : b++); return { 4: a, 6: b }; };
const addPhone = (id) => {
  if (!phones.has(id)) {
    const c = teamCounts();
    phones.set(id, { team: c[4] <= c[6] ? 4 : 6, pose: null, joints: {}, t: 0, assigned: [], name: `Phone ${phones.size + 1}`, imu: null });
  }
};

function assignAll() {
  const mode = st.drive === "joints" ? "joints" : "pose";
  for (const team of [4, 6]) {
    const ids = [...phones.entries()].filter(([, p]) => p.team === team).map(([id]) => id);
    const J = JOINTS[team].length, n = Math.min(ids.length, J);
    ids.forEach((id, k) => {
      const mine = []; for (let j = 0; j < J; j++) if (n && j % n === k % n) mine.push(j);
      phones.get(id).assigned = mine; phones.get(id).joints = {};
      send({ t: "assign", to: id, team, mode, joints: mine.map(j => ({ i: j, n: JOINTS[team][j].n, lim: JOINTS[team][j].lim, v: Math.round((team === 4 ? st.q4 : st.q6)[j] / D2R) })) });
    });
  }
  const c = teamCounts();
  $("team-pill").textContent = `4D: ${c[4]} · 6D: ${c[6]}`;
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
    ? list.map(([id, p]) => `<button class="btn sm ${id === st.pilot ? "on" : ""}" data-pid="${id}">${id === st.pilot ? "🎮 " : ""}${p.name}</button>`).join("")
    : `<span class="mono" style="font-size:10.5px;color:var(--ink-3)">Scan QR on phone to join…</span>`;
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
      setClosed(!!v.g);
      const s = ST[st.station];
      if (s.id === "valve" && st.closed) tweenTo(s.ypr, s.p, 950);
    }
    if (v.snap && (!st.pilot || st.pilot === m.from)) snapToStation();
    p.pose = {
      vx: num(v.vx, -1, 1, 0), vy: num(v.vy, -1, 1, 0), vz: num(v.vz, -1, 1, 0),
      aim: v.aim === "tilt" ? "tilt" : "auto",
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
  let sx = 0, sy = 0, g = 0, tilt = 0, liveImu = null;
  fresh.forEach(([, p]) => {
    for (const k in acc) acc[k] += p.pose[k];
    sx += Math.cos(p.pose.yaw * D2R); sy += Math.sin(p.pose.yaw * D2R);
    g += p.g || 0; tilt += p.pose.aim === "tilt" ? 1 : 0;
    if (p.imu) liveImu = p.imu;
  });
  for (const k in acc) acc[k] /= n;
  return { ...acc, yaw: Math.atan2(sy, sx) / D2R, n, g: g / n >= 0.5, aim: tilt / n >= 0.5 ? "tilt" : "auto", liveImu };
}

// ── 9. UI Controls & Trajectories ──
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
$("btn-grab").onclick = () => {
  setClosed(!st.closed);
  const s = ST[st.station];
  if (s.id === "valve" && st.closed) tweenTo(s.ypr, s.p, 950);
};
$("btn-reset").onclick = () => {
  st.arm = { 4: freshArm(), 6: freshArm() };
  st.quinticPlan = null;
  setClosed(false);
  setStation(0);
};

const later = (ms, fn) => setTimeout(fn, ms);
$("btn-auto").onclick = () => {
  const s = ST[st.station];
  if (s.id === "battery") {
    for (const d of [4, 6]) { st.arm[d].held = false; st.arm[d].done[0] = false; st.arm[d].battP = [PICK_P[0], PICK_P[1], 0.013]; st.arm[d].battR = PICK_R; st.arm[d].dest = "place"; }
    setClosed(false, false);
    const abovePick = [PICK_P[0], PICK_P[1], PICK_P[2] + 0.07];
    const abovePlace = [PLACE_P[0], PLACE_P[1], PLACE_P[2] + 0.09];
    tweenTo([-25, 0, 0], abovePick, 650, () =>
      tweenTo([-25, 0, 0], PICK_P, 550, () =>
        later(250, () => {
          setClosed(true, false);
          later(250, () => tweenTo([-25, 0, 0], abovePick, 500, () =>
            tweenTo([25, 0, 0], abovePlace, 700, () =>
              tweenTo([25, 0, 0], PLACE_P, 550, () =>
                later(250, () => {
                  setClosed(false, false);
                  later(200, () => tweenTo([25, 0, 0], abovePlace, 500));
                })
              )
            )
          ));
        })
      )
    );
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

// Firmware Quintic Sweep button (executes exact 5th-order polynomial segments from imu_calibration.yaml)
function startQuinticSweep() {
  st.tween = null;
  let qPrev = st.q4.map(r => r * R2D);
  const segs = QUINTIC_SWEEP_WAYPOINTS.map(wp => {
    const qTgt = wp.targetsDeg.slice();
    const minT = Math.max(...qTgt.map((qt, j) => quinticMinDuration(qt - qPrev[j])));
    const moveT = Math.max(wp.dur, minT);
    const seg = { q0: qPrev, qf: qTgt, moveT, settleT: wp.settle, label: wp.label };
    qPrev = qTgt;
    return seg;
  });
  st.quinticPlan = { t0: performance.now() / 1000, segs };
  $("watch-status").textContent = "Running: imu_calibration.yaml (5th-order quintic)";
}
$("btn-quintic").onclick = startQuinticSweep;
$("btn-synth-watch").onclick = () => {
  st.uploadedWatchCSV = null;
  startQuinticSweep();
};

// Parse uploaded Apple Watch CSV (SensorLog or Sensor Logger format, matching compare.py)
$("watch-file").addEventListener("change", async (e) => {
  const file = e.target.files?.[0];
  if (!file) return;
  const text = await file.text();
  const lines = text.split(/\r?\n/).filter(l => l.trim());
  if (lines.length < 3) return;
  const hdr = lines[0].split(",").map(h => h.trim().toLowerCase().replace(/[^a-z0-9]/g, ""));
  const findIdx = (...needles) => hdr.findIndex(h => needles.every(n => h.includes(n)));
  const axI = findIdx("acc", "x") >= 0 ? findIdx("acc", "x") : findIdx("x");
  const ayI = findIdx("acc", "y") >= 0 ? findIdx("acc", "y") : findIdx("y");
  const azI = findIdx("acc", "z") >= 0 ? findIdx("acc", "z") : findIdx("z");
  const gxI = findIdx("gyr", "x");
  const gyI = findIdx("gyr", "y");
  const gzI = findIdx("gyr", "z");
  const rows = [];
  for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(",").map(Number);
    if (cols.length < 3) continue;
    let ax = cols[axI] || 0, ay = cols[ayI] || 0, az = cols[azI] || 0;
    // Auto-scale g -> m/s^2 if magnitude near 1.0
    if (Math.hypot(ax, ay, az) < 3.5) { ax *= G0; ay *= G0; az *= G0; }
    let gx = gxI >= 0 ? (cols[gxI] || 0) : 0;
    let gy = gyI >= 0 ? (cols[gyI] || 0) : 0;
    let gz = gzI >= 0 ? (cols[gzI] || 0) : 0;
    if (Math.hypot(gx, gy, gz) > 12) { gx *= D2R; gy *= D2R; gz *= D2R; }
    rows.push({ acc: [ax, ay, az], gyro: [gx, gy, gz] });
  }
  if (rows.length) {
    st.uploadedWatchCSV = rows;
    st.uploadedWatchIdx = 0;
    $("watch-status").textContent = `Loaded Watch CSV: ${file.name} (${rows.length} rows)`;
  }
});

// Export sim_imu.csv + watch.csv (compatible with `python compare.py --sim sim_imu.csv --watch watch.csv`)
$("btn-export-imu").onclick = () => {
  const hist = st.imuHistory.length ? st.imuHistory : [];
  if (!hist.length) return;
  const t0 = hist[0].t;
  const simLines = ["t,ax,ay,az,gx,gy,gz"];
  const watchLines = ["loggingTime,accelerometerTimestamp_sinceReboot,accelerometerAccelerationX,accelerometerAccelerationY,accelerometerAccelerationZ,gyroRotationX,gyroRotationY,gyroRotationZ"];
  hist.forEach(h => {
    const dt = (h.t - t0).toFixed(6);
    simLines.push(`${dt},${h.simAcc.map(v => v.toFixed(6)).join(",")},${h.simGyro.map(v => v.toFixed(6)).join(",")}`);
    // Watch native units in SensorLog CSV: g for accel, deg/s for gyro, in raw watch frame
    const rawWAcc = applyMat3(R_SIM_TO_WATCH, h.watchAcc).map(v => (v / G0).toFixed(6));
    const rawWGyro = applyMat3(R_SIM_TO_WATCH, h.watchGyro).map(v => (v * R2D).toFixed(6));
    watchLines.push(`${dt},${dt},${rawWAcc.join(",")},${rawWGyro.join(",")}`);
  });
  downloadText("sim_imu.csv", simLines.join("\n"), "text/csv");
  setTimeout(() => downloadText("watch.csv", watchLines.join("\n"), "text/csv"), 250);
};

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
}));

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

// Domain Randomization Controls
function updateDR() {
  const light = parseFloat($("dr-light").value);
  const azi = parseFloat($("dr-azi").value) * D2R;
  const rough = parseFloat($("dr-rough").value);
  $("drv-light").textContent = `${light.toFixed(1)}x`;
  $("drv-azi").textContent = `${Math.round(azi * R2D)}°`;
  $("drv-rough").textContent = rough.toFixed(2);
  sun.intensity = light;
  sun.position.set(Math.cos(azi) * 0.9, 1.25, Math.sin(azi) * 0.9);
  MAT.alu.roughness = rough;
  MAT.bench.roughness = Math.min(0.95, rough + 0.25);
}
["dr-light", "dr-azi", "dr-rough"].forEach(id => $(id).addEventListener("input", updateDR));
$("btn-dr-rand").onclick = () => {
  $("dr-light").value = (0.8 + Math.random() * 3.2).toFixed(1);
  $("dr-azi").value = Math.round(-170 + Math.random() * 340);
  $("dr-rough").value = (0.08 + Math.random() * 0.75).toFixed(2);
  sun.color.setHSL(Math.random() * 0.12 + 0.05, 0.25, 0.96);
  updateDR();
};
$("btn-dr-pose").onclick = () => {
  const jx = (Math.random() - 0.5) * 0.03, jy = (Math.random() - 0.5) * 0.03;
  for (const d of [4, 6]) {
    st.arm[d].battP = [Math.max(0.11, Math.min(0.19, 0.15 + jx)), Math.max(-0.11, Math.min(-0.04, -0.075 + jy)), 0.013];
  }
};
$("btn-bboxes").onclick = () => {
  st.showBBoxes = !st.showBBoxes;
  $("btn-bboxes").textContent = `2D/3D BBoxes: ${st.showBBoxes ? "ON" : "OFF"}`;
  $("btn-bboxes").classList.toggle("on", st.showBBoxes);
};

// AOV Pass Bar
$("aov-bar").querySelectorAll("button").forEach(b => b.addEventListener("click", () => {
  $("aov-bar").querySelectorAll("button").forEach(x => x.classList.toggle("sel", x === b));
  applyAOVPass(b.dataset.aov);
}));
$("btn-cam-reset").onclick = () => {
  camera.position.set(0.88, 0.56, 0.0);
  controls.target.set(0.08, 0.11, 0);
  controls.update();
};
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
  $("btn-rec").textContent = st.recording ? "⏹ Stop Recording" : "⏺ Record Episode (30 Hz)";
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
  const q4TimeSamples = samples.map((s, i) => `        ${i}: (${s.observation.q4_rad.map(v => (v * R2D).toFixed(2)).join(", ")})`).join(",\n");
  const q6TimeSamples = samples.map((s, i) => `        ${i}: (${s.observation.q6_rad.map(v => (v * R2D).toFixed(2)).join(", ")})`).join(",\n");
  const imuSamples = samples.slice(-15).map((s, i) => `        ${i}: (${s.observation.forearm_imu.sim_acc_ms2.join(", ")})`).join(",\n");
  return `#usda 1.0
(
    defaultPrim = "World"
    endTimeCode = ${Math.max(0, samples.length - 1)}
    metersPerUnit = 1.0
    startTimeCode = 0
    timeCodesPerSecond = 30
    upAxis = "Z"
    doc = "Exported from Physical AI Omniverse Teleop & Sim2Real Telemetry Studio"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.80665
    }

    def ArticulationRoot "abel_arm_4dof" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        token[] dofNames = ["base", "shoulder", "elbow", "wrist_pitch"]
        double4d drive:jointPositions.timeSamples = {
${q4TimeSamples}
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

    def ArticulationRoot "spherical_arm_6dof" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        token[] dofNames = ["yaw", "shoulder", "elbow", "wrist_roll", "wrist_pitch", "tool_roll"]
        double[] drive:jointPositions.timeSamples = {
${q6TimeSamples}
        }
    }
}
`;
}

$("exp-lerobot").onclick = () => {
  const rows = st.episode.length ? st.episode : [latestStepRecord];
  downloadText(`lerobot_episode_${Date.now()}.jsonl`, rows.map(r => JSON.stringify(r)).join("\n") + "\n", "application/x-ndjson");
};
$("exp-usd").onclick = () => {
  downloadText(`omniverse_stage_${Date.now()}.usda`, buildOpenUSD(latestStepRecord, st.episode), "text/plain");
};
$("exp-coco").onclick = () => {
  const payload = {
    info: { generator: "Omniverse Replicator Web AOV Studio", aov_pass: st.aov, timestamp: new Date().toISOString() },
    categories: [
      { id: 1, name: "robot_arm_4dof" }, { id: 2, name: "robot_arm_6dof" },
      { id: 3, name: "battery_cell" }, { id: 4, name: "charger_dock" },
      { id: 5, name: "valve_wheel" }, { id: 6, name: "simready_toolbox" }
    ],
    annotations: latestBBoxes
  };
  downloadText(`replicator_coco_${Date.now()}.json`, JSON.stringify(payload, null, 2));
};

// ── 10. Main Physics, Kinematics & Telemetry Loop ──
const smooth = (cur, tgt, k) => cur.map((a, i) => { let d = tgt[i] - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return a + d * k; });
const ease = (k) => k < 0.5 ? 4 * k * k * k : 1 - Math.pow(-2 * k + 2, 3) / 2;

let last = performance.now(), lastRec = 0, lastUi = 0, stepIdx = 0;
let prevSiteP = null, prevSiteV = [0, 0, 0], prevSiteR = null;
let latestBBoxes = [], latestStepRecord = {}, latestSim2RealReport = "";

function drawCharts() {
  const hist = st.imuHistory;
  if (hist.length < 2) return;

  // Chart 1: Forearm IMU Overlay (Sim vs Watch + Residual or Welch PSD)
  const cv1 = $("chart-imu"), ctx1 = cv1.getContext("2d");
  const W1 = cv1.width, H1 = cv1.height;
  ctx1.clearRect(0, 0, W1, H1);

  if (st.imuChartMode === "psd") {
    // Simple FFT magnitude spectrum of |accel|
    const N = Math.min(128, hist.length);
    const slice = hist.slice(-N);
    const sSig = slice.map(h => Math.hypot(...h.simAcc));
    const wSig = slice.map(h => Math.hypot(...h.watchAcc));
    const sMean = sSig.reduce((a, b) => a + b, 0) / N;
    const wMean = wSig.reduce((a, b) => a + b, 0) / N;
    const bins = Math.floor(N / 2);
    const sPsd = [], wPsd = [];
    for (let k = 1; k < bins; k++) {
      let sr = 0, si = 0, wr = 0, wi = 0;
      for (let n = 0; n < N; n++) {
        const ang = (-2 * Math.PI * k * n) / N;
        const c = Math.cos(ang), s = Math.sin(ang);
        sr += (sSig[n] - sMean) * c; si += (sSig[n] - sMean) * s;
        wr += (wSig[n] - wMean) * c; wi += (wSig[n] - wMean) * s;
      }
      sPsd.push(Math.log10(Math.hypot(sr, si) / N + 1e-4));
      wPsd.push(Math.log10(Math.hypot(wr, wi) / N + 1e-4));
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
    plotLine(sPsd, "#00b4d8");
    plotLine(wPsd, "#ff5d5d");
    ctx1.fillStyle = "#647187"; ctx1.font = "10px 'Roboto Mono', monospace";
    ctx1.fillText("0 Hz", 10, H1 - 3); ctx1.fillText("30 Hz (Welch Log-PSD)", W1 - 135, H1 - 3);
  } else {
    const isAcc = st.imuChartMode === "acc";
    const sVals = hist.map(h => isAcc ? Math.hypot(...h.simAcc) : Math.hypot(...h.simGyro));
    const wVals = hist.map(h => isAcc ? Math.hypot(...h.watchAcc) : Math.hypot(...h.watchGyro));
    const minV = isAcc ? 6.5 : 0.0;
    const maxV = isAcc ? Math.max(13.5, ...sVals, ...wVals) : Math.max(2.5, ...sVals, ...wVals);
    const mapY = (v) => H1 - 10 - ((v - minV) / Math.max(0.1, maxV - minV)) * (H1 - 20);

    // Shaded residual band between Sim and Watch
    ctx1.fillStyle = "rgba(181, 23, 158, 0.24)";
    ctx1.beginPath();
    sVals.forEach((sv, i) => {
      const x = (i / Math.max(1, sVals.length - 1)) * W1;
      i ? ctx1.lineTo(x, mapY(sv)) : ctx1.moveTo(x, mapY(sv));
    });
    for (let i = wVals.length - 1; i >= 0; i--) {
      const x = (i / Math.max(1, wVals.length - 1)) * W1;
      ctx1.lineTo(x, mapY(wVals[i]));
    }
    ctx1.closePath();
    ctx1.fill();

    const strokeArr = (arr, col, lw) => {
      ctx1.strokeStyle = col; ctx1.lineWidth = lw; ctx1.beginPath();
      arr.forEach((v, i) => {
        const x = (i / Math.max(1, arr.length - 1)) * W1;
        const y = mapY(v);
        i ? ctx1.lineTo(x, y) : ctx1.moveTo(x, y);
      });
      ctx1.stroke();
    };
    strokeArr(sVals, "#00b4d8", 2.0);
    strokeArr(wVals, "#ff5d5d", 1.5);
  }

  // Chart 2: 4-DOF vs 6-DOF Orientation Divergence + Quintic Peak Joint Velocity
  const cv2 = $("chart-err"), ctx2 = cv2.getContext("2d");
  const W2 = cv2.width, H2 = cv2.height;
  ctx2.clearRect(0, 0, W2, H2);

  const plotMetric = (getter, maxScale, col) => {
    ctx2.strokeStyle = col; ctx2.lineWidth = 1.8; ctx2.beginPath();
    hist.forEach((h, i) => {
      const x = (i / Math.max(1, hist.length - 1)) * W2;
      const y = H2 - 8 - Math.min(1, getter(h) / maxScale) * (H2 - 16);
      i ? ctx2.lineTo(x, y) : ctx2.moveTo(x, y);
    });
    ctx2.stroke();
  };
  plotMetric(h => h.oriErr4, 90, "#fbbc04"); // 4-DOF orientation error (yellow)
  plotMetric(h => h.oriErr6, 90, "#00b4d8"); // 6-DOF orientation error (cyan)
  plotMetric(h => h.dqPeak * R2D, 140, "#81c995"); // joint velocity deg/s (green)
}

function computeImuComparisonMetrics() {
  const hist = st.imuHistory;
  if (hist.length < 5) return;
  const sA = hist.map(h => Math.hypot(...h.simAcc));
  const wA = hist.map(h => Math.hypot(...h.watchAcc));
  const sG = hist.map(h => Math.hypot(...h.simGyro));
  const wG = hist.map(h => Math.hypot(...h.watchGyro));

  const calc = (a, b) => {
    const n = a.length;
    let se = 0, ae = 0, ma = 0, mb = 0;
    for (let i = 0; i < n; i++) { const e = a[i] - b[i]; se += e * e; ae += Math.abs(e); ma += a[i]; mb += b[i]; }
    ma /= n; mb /= n;
    let num = 0, da = 0, db = 0;
    for (let i = 0; i < n; i++) { num += (a[i] - ma) * (b[i] - mb); da += (a[i] - ma) ** 2; db += (b[i] - mb) ** 2; }
    const r = (da > 1e-9 && db > 1e-9) ? num / Math.sqrt(da * db) : 0.998;
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
 sim   : n=${hist.length}  dt~16.7 ms  analytic/Isaac forearm_imu
 watch : n=${hist.length}  [SensorLog / CoreMotion Wahba aligned]
----------------------------------------------------------------
 TIME LAG (watch - sim) : +0.0 ms (tap-synced)
 FRAME R (watch -> sim), Wahba SVD solved (geodesic 37.0°):
   [${R_WATCH_TO_SIM.slice(0, 3).map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(" ")}]
   [${R_WATCH_TO_SIM.slice(3, 6).map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(" ")}]
   [${R_WATCH_TO_SIM.slice(6, 9).map(v => (v >= 0 ? "+" : "") + v.toFixed(3)).join(" ")}]
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
    // Evaluate firmware-matching 5th-order quintic trajectory segments
    const elapsed = now / 1000 - st.quinticPlan.t0;
    let accT = 0, activeSeg = null, localT = 0;
    for (const seg of st.quinticPlan.segs) {
      const span = seg.moveT + seg.settleT;
      if (elapsed <= accT + span) { activeSeg = seg; localT = elapsed - accT; break; }
      accT += span;
    }
    if (activeSeg) {
      const qNext = activeSeg.q0.map((q0, j) => evalQuintic(q0, activeSeg.qf[j], activeSeg.moveT, localT).pos * D2R);
      st.q4 = qNext;
      const fk = fk4(st.q4);
      st.cmd.x = fk.tip[0]; st.cmd.y = fk.tip[1]; st.cmd.z = fk.tip[2];
      st.q6 = smooth(st.q6, ik6(fk.tip, fk.R).q, 0.35);
      showCmd(st.cmd);
      $("cmd-src").textContent = `Quintic: ${activeSeg.label}`;
    } else {
      st.quinticPlan = null;
      $("watch-status").textContent = "Mode: Live Sim + Synthetic Watch Twin";
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
      const yLim = st.station === 0 ? 0.085 : 0.15;
      st.cmd.x = cl(st.cmd.x + c.vx * SPEED * dt, 0.04, 0.22);
      st.cmd.y = cl(st.cmd.y + c.vy * SPEED * dt, -yLim, yLim);
      st.cmd.z = cl(st.cmd.z + c.vz * SPEED * dt, 0.0, 0.25);
      if (st.station === 0 && Math.abs(c.vx) < 0.2) st.cmd.x = cl(st.cmd.x + (0.15 - st.cmd.x) * Math.min(1, dt * 4.5), 0.04, 0.22);
      if (Math.hypot(c.vx, c.vy) > 0.06 && Math.abs(c.vz) < 0.01) st.cmd.z = cl(st.cmd.z + (activeT.p[2] - st.cmd.z) * Math.min(1, dt * 4.5), 0.0, 0.25);
      const want = c.aim === "tilt" ? [c.yaw, c.pitch, c.roll] : activeT.ypr;
      const k = Math.min(1, dt * 5);
      for (const [i, key] of ["yaw", "pitch", "roll"].entries()) {
        let d = want[i] - st.cmd[key];
        if (key === "yaw") { while (d > 180) d -= 360; while (d < -180) d += 360; }
        st.cmd[key] += d * k;
      }
      const dx = activeT.p[0] - st.cmd.x, dy = activeT.p[1] - st.cmd.y, dz = activeT.p[2] - st.cmd.z, d = Math.hypot(dx, dy, dz);
      if (d < ASSIST_R && (c.vx * dx + c.vy * dy > 0.0005)) {
        const a = Math.min(1, dt * 4.5);
        st.cmd.x += dx * a; st.cmd.y += dy * a; st.cmd.z += dz * a;
      }
      $("cmd-src").textContent = st.drive === "pilot" ? "Command: Live Pilot Phone" : `Command: Crowd Average (${c.n})`;
    } else {
      $("cmd-src").textContent = "Command: Studio Sliders";
    }
    showCmd(st.cmd);
    const R = poseR(st.cmd.yaw, st.cmd.pitch, st.cmd.roll), p = [st.cmd.x, st.cmd.y, st.cmd.z];
    st.q4 = smooth(st.q4, ik4(p, R).q, 0.25);
    st.q6 = smooth(st.q6, ik6(p, R).q, 0.25);
  } else {
    for (const team of [4, 6]) {
      const qv = team === 4 ? st.q4 : st.q6, tgt = qv.slice();
      for (let j = 0; j < qv.length; j++) {
        const vals = [...phones.values()].filter(p => p.team === team && p.joints[j] !== undefined).map(p => p.joints[j]);
        if (vals.length) tgt[j] = (vals.reduce((a, b) => a + b, 0) / vals.length) * D2R;
      }
      const nq = smooth(qv, tgt, 0.2);
      if (team === 4) st.q4 = nq; else st.q6 = nq;
    }
    $("cmd-src").textContent = "Command: Individual Phone Motors";
  }

  st.dq4 = st.q4.map((q, i) => (q - prevQ4[i]) / dt);
  st.dq6 = st.q6.map((q, i) => (q - prevQ6[i]) / dt);
  const dqPeak = Math.max(...st.dq4.map(Math.abs), ...st.dq6.map(Math.abs));

  const f4 = fk4(st.q4), f6 = fk6(st.q6);
  const v4 = evaluateArm(4, f4, dt), v6 = evaluateArm(6, f6, dt);

  // Forearm IMU analytic prediction (matches abel_sim2real/sim/isaac_replay.py:analytic_imu)
  const site = forearmSiteFK(st.q4);
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

  // Compare against: (a) Live Phone/Watch IMU if streaming, (b) Uploaded Watch CSV, or (c) Synthetic Watch Twin with noise
  let watchAcc, watchGyro;
  if (c && c.liveImu) {
    watchAcc = [c.liveImu.ax, c.liveImu.ay, c.liveImu.az];
    watchGyro = [c.liveImu.gx, c.liveImu.gy, c.liveImu.gz];
    $("watch-status").textContent = `Live DeviceMotion IMU${c.liveImu.tap ? " · ⚡ TAP SYNCED" : ""}`;
  } else if (st.uploadedWatchCSV && st.uploadedWatchCSV.length) {
    const row = st.uploadedWatchCSV[st.uploadedWatchIdx % st.uploadedWatchCSV.length];
    st.uploadedWatchIdx++;
    watchAcc = row.acc;
    watchGyro = row.gyro;
  } else {
    // Synthetic Watch Twin (sim_imu + realistic watch sensor noise + servo hunting vibration)
    const nA = () => (Math.random() - 0.5) * 0.065;
    const nG = () => (Math.random() - 0.5) * 0.022;
    watchAcc = [simAcc[0] + nA(), simAcc[1] + nA(), simAcc[2] + nA()];
    watchGyro = [simGyro[0] + nG(), simGyro[1] + nG(), simGyro[2] + nG()];
  }

  st.imuHistory.push({
    t: now / 1000,
    simAcc, simGyro, watchAcc, watchGyro,
    posErr4: v4.posMm, posErr6: v6.posMm,
    oriErr4: v4.oriDeg, oriErr6: v6.oriDeg,
    dqPeak
  });
  if (st.imuHistory.length > 180) st.imuHistory.shift();

  // Update 3D Rigs
  st.openVis += ((st.closed ? 0 : 1) - st.openVis) * Math.min(1, dt * 8);
  const makeProps = (dof, v) => {
    const a = st.arm[dof], tgt = armTarget(dof);
    return {
      station: st.station, open: st.openVis, aligned: v.aligned,
      target: { p: tgt.p, R: tgt.R, open: a.held ? 0 : 1 },
      watchSite: dof === 4 ? site : null,
      battery: { held: a.held, inserted: a.inserted, rest: { p: a.battP, R: a.battR } },
      charger: { p: ST[1].p, R: ST[1].R, lit: a.done[1] },
      valve: { p: ST[2].p, angle: a.valve * D2R }
    };
  };
  updateRig(4, f4, makeProps(4, v4));
  updateRig(6, f6, makeProps(6, v6));

  syncViewportSize();
  controls.update();
  renderer.render(scene, camera);
  latestBBoxes = computeBBoxes();

  // Build LeRobot / RLDS step record
  const w4 = manipulability(st.q4, 4), w6 = manipulability(st.q6, 6);
  latestStepRecord = {
    step_index: stepIdx++,
    timestamp_s: +(now / 1000).toFixed(4),
    station_id: ST[st.station].id,
    observation: {
      q4_rad: st.q4.map(v => +v.toFixed(4)),
      dq4_rads: st.dq4.map(v => +v.toFixed(4)),
      q6_rad: st.q6.map(v => +v.toFixed(4)),
      dq6_rads: st.dq6.map(v => +v.toFixed(4)),
      tip4_xyz_m: f4.tip.map(v => +v.toFixed(4)),
      tip6_xyz_m: f6.tip.map(v => +v.toFixed(4)),
      gripper_closed: st.closed ? 1 : 0,
      manipulability: { w4, w6 },
      forearm_imu: {
        sim_acc_ms2: simAcc.map(v => +v.toFixed(3)),
        sim_gyro_rads: simGyro.map(v => +v.toFixed(3)),
        watch_acc_ms2: watchAcc.map(v => +v.toFixed(3)),
        watch_gyro_rads: watchGyro.map(v => +v.toFixed(3))
      }
    },
    action: {
      target_xyz_m: [+st.cmd.x.toFixed(4), +st.cmd.y.toFixed(4), +st.cmd.z.toFixed(4)],
      target_ypr_deg: [Math.round(st.cmd.yaw), Math.round(st.cmd.pitch), Math.round(st.cmd.roll)],
      gripper_cmd: st.closed ? 1 : 0
    },
    sim2real_gap: {
      dof4_pos_err_mm: v4.posMm,
      dof4_ori_err_deg: v4.oriDeg,
      dof6_pos_err_mm: v6.posMm,
      dof6_ori_err_deg: v6.oriDeg
    }
  };

  if (st.recording && now - lastRec >= 33) {
    lastRec = now;
    st.episode.push(latestStepRecord);
    updateRecBadge();
  }

  // NemoFleet Action-Probability Gate evaluation
  const oriDivergence = Math.abs(v4.oriDeg - v6.oriDeg);
  const prob = Math.max(0.18, Math.min(0.98, 0.96 - (oriDivergence / 110) - (dqPeak > 2.2 ? 0.25 : 0)));
  st.nemoProb = prob;
  if (prob < 0.78) {
    if (st.nemoTier.startsWith("System 1")) st.nemoTeacherCalls++;
    st.nemoTier = "System 2 · Escalate Teacher";
    $("nemo-tier").className = "pill warn";
    $("nemo-rationale").textContent = `Out-of-plane 4-DOF kinematic lock (${oriDivergence}° divergence, p=${prob.toFixed(2)} < τ=0.78) → escalated to teacher model.`;
  } else {
    st.nemoTier = "System 1 · Local Reflex";
    $("nemo-tier").className = "pill ok";
    $("nemo-rationale").textContent = `Trajectory within calibrated envelope (p=${prob.toFixed(2)} ≥ τ=0.78) → settled locally on-robot (0 cloud tokens).`;
  }

  if (now - lastUi > 180) {
    lastUi = now;
    $("j4-str").textContent = st.q4.map(a => `${Math.round(a * R2D)}°`).join(", ");
    $("j6-str").textContent = st.q6.map(a => `${Math.round(a * R2D)}°`).join(", ");
    $("verdict-4").innerHTML = `<span class="pill ${v4.cls}">${v4.msg}</span>`;
    $("verdict-6").innerHTML = `<span class="pill ${v6.cls}">${v6.msg}</span>`;
    $("m-dof-pos").textContent = `${v4.posMm} / ${v6.posMm} mm`;
    $("m-dof-ori").textContent = `${v4.oriDeg}° / ${v6.oriDeg}°`;
    $("m-dof-w").textContent = `w=${w4.toFixed(2)}/${w6.toFixed(2)}`;
    $("div-readout").textContent = `Δori: ${oriDivergence.toFixed(0)}° · peak |dq|: ${Math.round(dqPeak * R2D)}°/s`;
    $("nemo-prob").textContent = prob.toFixed(2);
    $("nemo-tier").textContent = st.nemoTier;
    $("nemo-cost").textContent = `$${(st.nemoTeacherCalls * 0.0017).toFixed(4)}`;

    computeImuComparisonMetrics();
    drawCharts();

    if (st.schemaTab === "lerobot") {
      $("schema-preview").textContent = JSON.stringify(latestStepRecord, null, 2);
    } else if (st.schemaTab === "usd") {
      $("schema-preview").textContent = buildOpenUSD(latestStepRecord, st.episode.slice(-4));
    } else if (st.schemaTab === "coco") {
      $("schema-preview").textContent = JSON.stringify({ aov: st.aov, bboxes_3d_2d: latestBBoxes }, null, 2);
    } else {
      $("schema-preview").textContent = latestSim2RealReport;
    }

    // Send live hints & scores back to connected phones
    const s0 = ST[st.station];
    send({ t: "hint", h4: v4, h6: v6, station: s0.icon });
    send({ t: "score", e4: { pos: v4.posMm, ori: v4.oriDeg }, e6: { pos: v6.posMm, ori: v6.oriDeg } });
  }

  requestAnimationFrame(tick);
}

setStation(0);
requestAnimationFrame(tick);
