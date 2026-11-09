# Operon — Agent Handoff Document (`HANDOFF.md`)

**Last Updated:** 2026-11-09 PST
**Author / Owner:** Gretchen Boria, PhD
**Workspace:** `/Users/dr.gretchenboria/Operon`
**GitHub Repository:** `https://github.com/gretchenboria/Operon.git` (`main`)

---

## 0. State of play — read this first

| | |
|---|---|
| **`main` HEAD** | MuJoCo 3.x rigid-body contact physics on all three stations, cache version `?v=24` |
| **Deployed to Cloud Run** | revision `operon-00041-c4q`, commit `decdd72`, cache version `?v=22` |
| **Therefore** | **production is ~4 commits behind local `main` and has none of the physics work.** Deploying is the single biggest outstanding action. |
| **Working tree** | clean; `main` is in sync with `origin/main` |
| **Regression suite** | `./test/physics/run-all.sh --browser` — 11/11 passing at HEAD |

### The physics commits, in order

| commit | what it did |
|---|---|
| `e1a904d` | Brought MuJoCo 3.x WASM into the browser studio; Station 0 grasp/release became real |
| `7cf6e9a` | Raised grip to ~17 N; fixed a false "dropped" report during re-orientation |
| `448dc8e` | Made the charger dock and valve tasks physical instead of scripted |
| `1c6adef` | Corrected the measured reliability figure for the scripted dock insertion |

### Two things that need an owner decision

1. **Deploy.** Nothing since `decdd72` is live. See §1 for the command. The `.wasm` is served out of `node_modules`, so the container needs `mujoco` as a production dependency — it is one, and the `Dockerfile` already runs `npm install --omit=dev`, so a normal `--source .` deploy picks it up. Verify `/vendor/mujoco/mujoco.wasm` returns `200 application/wasm` after deploying.
2. **The Station 1 scene moved.** Making the charger dock physically reachable required re-angling it to face upward *and* relocating it from `[0.10, 0.13, 0.12]` to `[0.10, 0.090, 0.040]`. That is a visible change to the workcell layout. The reasoning is in §3.4; if the original placement matters more than a physical insertion, revert `ST[1]` and set `physCellOn()` back to Station 0 only.

---

## 1. Live Production & Cloud Infrastructure

- **Google Cloud Run Service URL:** `https://operon-421717890494.us-central1.run.app`
  - **Service Name:** `operon`
  - **GCP Project ID:** `gen-lang-client-0221638641` ("Astro")
  - **Region:** `us-central1`
  - **Billing Account:** `01A844-CEA6E2-BAF517` (~$300 trial credit account — **WATCH CREDITS CAREFULLY!** Do NOT spin up paid GPU Compute Engine VMs unless explicitly instructed; verify `gcloud compute instances list --project=gen-lang-client-0221638641` shows `0` running instances).
  - **Approved GPU Quotas in `us-central1`:** `NVIDIA_L4_GPUS: 1.0`, `PREEMPTIBLE_NVIDIA_L4_GPUS: 1.0` (currently 0 instances, $0/hr).
- **Deploy Command:**
  ```zsh
  gcloud run deploy operon --source . --region us-central1 --project gen-lang-client-0221638641 --allow-unauthenticated --quiet
  ```
- **Untracked local file to NEVER commit:** `Screen Recording 2026-11-08 at 4.35.49 PM.mov`
- **Local dev:** `PORT=8841 node server.mjs` then `http://127.0.0.1:8841/teleop-studio.html`

---

## 2. MuJoCo 3.x Rigid-Body Contact Physics

The studio no longer fakes manipulation. It runs the **official Google DeepMind MuJoCo WebAssembly bindings** (npm [`mujoco@3.1.16`](https://github.com/google-deepmind/mujoco/tree/main/wasm), Apache-2.0) in the browser. **There is no MuJoCo 4** — 3.x is the current line; don't go looking for a 4.x package.

### 2.1 Files

- **`public/models/operon_station0.xml`** — the MJCF scene. Compiled **once per arm rig** (two independent `mjModel`/`mjData`), so the two rigs can never contact each other and each keeps its own object state.
- **`public/mujoco-physics.js`** — WASM loader plus a per-arm `ArmSim` wrapper: `setArmTarget`, `setGripTarget`, `advance`, `read`, `placeCell`, `resetCell`, `setStation`, `setValveTarget`, `recover`, `dispose`. Caches every Embind handle once (they are WASM-heap objects, **not** garbage collected). Fails safe — if the WASM or the model will not load, `available=false` and the studio stays on the analytic path.
- **`public/teleop-studio.js`** — integration. Key symbols: `physArmOn()`, `physCellOn()`, `physValveOn()`, `physWaitArrival()`, `physCellGraspYaw()`, `runPoseSequence()`, `runPhysicalDockInsertion()`, `runPhysicalValveTurn()`, `blendR()`, `st.cmdR`, `st.phys`.
- **`server.mjs`** — serves the 8.2 MB `.wasm` from `node_modules/mujoco` at `/vendor/mujoco/` with `application/wasm` (needed for streaming compilation) and `Cache-Control: immutable`. **The `.wasm` is deliberately not committed**; it comes from the pinned dependency.

### 2.2 How the arm is driven

Both arms run on **position actuators**, not by writing `qpos`. The analytic `ik6` result is the *servo setpoint*; what renders is what the solver achieved under gravity, link inertia, joint damping and contact.

- Left rig tracks the ideal IK; right rig tracks the **firmware quintic streamer** (`st.q6Real`). Sim-vs-real therefore compares two genuinely dynamic arms rather than two kinematic chains.
- The MJCF chain reproduces `fk6()` from `public/demos/arms3d.js` **to under a nanometre** (worst case 0.00 nm position, 6.66e-16 orientation), and collapses exactly onto `fk4()` with `wrist_roll` and `tool_roll` held at 0. The physical and analytic paths are interchangeable, which is what makes the `Kinematic` toggle in the Contact Physics card a true A/B.
- Cost is ~1.5 ms/frame for both arms at the 0.5 ms timestep (~9 % of a 60 fps budget).

### 2.3 Measured outcomes

| | before | after |
|---|---|---|
| grasp | distance heuristic + proximity snap | pad friction; `held` from `touch` sensors |
| jaws | binary eased snap, closed *through* the cell | stop at the cell's own width (22.8 mm pad gap on a 24 mm cell) |
| release | cell teleported to the destination pad | **falls under gravity** (67 mm in 0.12 s) |
| charger dock | cell teleported into the slot | lowered into a cradle, retained after the arm withdraws |
| valve | angle inferred from tool roll | wound 0→91° by contact, **no stem actuator** |
| RMS TCP jerk (pick & place) | 43.1 m/s³ | **29.2 m/s³ (−32 %)** |
| RMS TCP jerk (live wrist tilt) | 108 m/s³ | **47.1 m/s³ (−56 %)** |
| peak TCP jerk (hard step) | 62 217 m/s³ | **3 835 m/s³ (−94 %)** |
| peak TCP speed | 8.88 m/s (impossible) | 1.32 m/s |
| forearm IMU | finite-difference estimate | real MuJoCo `accelerometer` / `gyro` sensor |
| render fidelity | — | rendered cell pose == solved pose to **0.00 µm** over a full cycle |

---

## 3. Load-bearing details — do not "simplify" these

Each of these cost real debugging time. They look arbitrary and are not.

### 3.1 Collision bitmasks
Arm `contype=1 conaffinity=6`, environment `2/5`, cell `4/3`. Without them the gripper housing self-collides with the forearm capsule and **physically jams the wrist at ~48° of tracking error**, which presents as "the arm won't reach" rather than as a collision.

### 3.2 `PHYS_GRASP_H = 0.011`
Finger pads reach 14 mm below the TCP. Grasping at the cell's centre drives the fingertips into the 1 mm locating pad; one pad then reads a ~48 N crush load *against the table*, the jaws jam open, and the grasp fails on the lift. **This only shows on the right-hand rig**, which follows the firmware streamer (1° deadband ⇒ several mm of tool error). The ideal-IK left rig hides it, so the Node tests passed while the real studio failed — it was caught only by the headless-browser test.

### 3.3 Grip force and the `held` flag
`grip_act` runs at `kp=1800` (~17 N clamp). At `kp=900` the clamp was ~9 N, weak enough that a carried cell slid 9 mm between the pads during a re-orientation. `held` uses the **total** pad load `(padL + padR) > 0.4 N`, not both pads individually: re-orienting shifts the cell onto one pad and momentarily unloads the other, which the earlier test reported as a drop while the cell was still 2 mm from the TCP.

### 3.4 Station 1 — three defects that had to be fixed together
Fixing any one alone still fails.

- **a. It faced downward.** Outward normal `(0, 0.906, -0.423)` put **+4.15 m/s² of gravity along the opening** and ejected every released cell. Now `(0, 0.423, 0.906)` — −8.89 m/s² *into* the pocket, with the remaining 4.15 m/s² of slide taken by the +y end wall.
- **b. Body frame ≠ approach frame.** `ST[1].R`'s +z is the dock's **outward normal**, but a tool's +z is its **approach axis**. Aiming the arm at the body frame pointed the tool backwards and put the wrist centre 60 mm *inside* the dock structure. Stations now expose `app`; `s.tgtR = s.app || s.R` is what `armTarget()`/`evaluateArm()` use, while `s.R` still draws the prop.
- **c. It was out of reach.** At the authored height the **wrist centre** — not the TCP — fell outside the 200 mm envelope; the commanded pose ended up 50–108 mm from anything achievable with the forearm fouling the dock. Relocated to `[0.10, 0.090, 0.040]`, where standoffs of 12–40 mm solve at 0.175–0.193 m.

The cradle interior is 60 × 64 mm so the finger arms (46.5 mm outer span while holding a 24 mm cell) can enter, with side rails low enough to clear them. Five boxes, because MuJoCo geoms are convex — a "slot" has to be walls.

### 3.5 Station 1 — the insertion path is staged on purpose
Lift → re-orient **in place** → traverse **above** the dock → descend. Blending position and orientation together walks the arm through poses it cannot track (ends 50 mm short); a straight line from the safe point to the dock clips the cradle and strips the cell out of the jaws.

### 3.6 Station 2 — no actuator on the valve stem
The arm engages the handwheel **hub** and rolls the tool about the wheel axis; every degree comes from contact against the stem's own 0.085 N·m frictionloss (~0.14 N·m breakaway).

- The roll **must** command `R` directly via `st.cmdR`. Routed through yaw/pitch/roll the same motion stalls at 36° and unwinds, because `yprOf()` cannot represent a clean roll about world +X.
- The roll is **closed-loop on the wheel's solved angle** (lead the measured angle by ~14°). Open-loop the tool outruns the wheel about 3× and most of the rotation is wasted as slip; closing the loop took the turn from 27 s to 8 s.
- **Gripping the rim or a spoke does not work.** The 11 mm rim and 6.4 mm spokes are too small relative to the hand; the fingers jam on the spokes (16 contacts, 327 N on one pad). Hub engagement is the approach that works.

### 3.7 Station props are parked when inactive
`ArmSim.setStation()` moves the inactive stations' bodies 5 m below the table. This is **required, not cosmetic**: the renderer hides inactive props, and leaving them solid makes the arm clip the charger dock mid-traverse during pick-and-place. **Any test that builds the model directly instead of through `ArmSim` must park them too** — `validate_mjcf.mjs` does this explicitly and will fail if you remove it.

### 3.8 `jawGapMm` is the pad-to-pad gap
Not finger-body separation. The pads close to 3.5 mm; the finger bodies are 18 mm apart at full close. An earlier version reported the latter and made a perfectly good 24 mm grasp look like a failure.

### 3.9 Coordinate conventions
`T3(x, y, z) = (x, z, -y)` — the three.js scene is **Y-up**, the arm is **Z-up**, and a rig's display offset lands on three's **z**, not y. `window.__operonRenderProbe()` is a read-only hook returning the rendered vs solved cell pose and jaw aperture; it already does this conversion.

---

## 4. Known gap: the scripted Station 1 insertion

**It completes 7 of 8 consecutive runs.** Everything else in the suite is deterministic.

- **The physics is not the problem.** `dock_retention.mjs` and `dock_insertion.mjs` pass every time, and the browser run seats the cell 4.5 mm from the slot centre when it works.
- **Symptom of the failure:** verdict reads `"Aligned in Slot -> Click Release"` — the arm arrives at the dock but the cell is not seated, i.e. it was lost or badly held somewhere in the carry.
- **Teleoperated insertion is unaffected.** This is specific to the scripted Auto sequence.

### What was tried and rejected
Compensating for the cell sliding in the jaws by measuring its offset from the TCP before the final descent and aiming the **cell** (rather than the tool) at the slot. **Made it worse — 1/6 — and was reverted.** The slip is real and measurable (11 mm → 2 mm from the TCP during the 115° re-orientation) but it is evidently not what decides success. Don't re-try this without new evidence.

### The next thing to try (designed, written, never tested)
Add a **seating verification with retry** at the end of `runPhysicalDockInsertion()`. The sequence already verifies arrival and verifies the clamp; nothing verifies the outcome, so a run that quietly loses the cell still reports success. Give the function an `attempt = 1` parameter, and in the completion callback of the final `runPoseSequence`:

```js
const r = st.phys.readR;
const seated = r && !r.held && r.cellVel < 0.02 && Math.hypot(
  r.cellP[0] - CHARGER_SLOT_P[0],
  r.cellP[1] - CHARGER_SLOT_P[1],
  r.cellP[2] - CHARGER_SLOT_P[2]) < 0.032;
if (!seated && attempt < 3) later(600, () => runPhysicalDockInsertion(attempt + 1));
```

Re-running the whole insertion is cheap because the cell is wherever physics left it and the sequence re-reads its pose anyway. If failures are independent this takes 7/8 to roughly 63/64. **This was written and then reverted unverified — treat it as a proposal, not a fix, and measure it.**

### ⚠ Measure on an idle machine
Two separate rounds of much worse numbers (1/6 and 3/6) turned out to be **infrastructure, not physics**:
- `/tmp` filled with headless-Chrome profiles (1.8 GB) on a disk that was already at 99 %.
- The test server died part-way through a loop, so later runs were hitting nothing.

Always health-check the server between runs and clear `/tmp/cr-prof-*`. `run-all.sh` does the check and the cleanup; a bare loop does not.

---

## 5. Test suite — `test/physics/`

```zsh
./test/physics/run-all.sh              # 7 Node suites, deterministic, no browser
PORT=8841 node server.mjs &
./test/physics/run-all.sh --browser    # + 4 headless-Chrome suites  (11 total)
OPERON_URL=http://host:port ./test/physics/run-all.sh --browser
```

| file | what it proves |
|---|---|
| `validate_mjcf.mjs` | MJCF chain == `fk6()`/`fk4()`; gravity sag; grasp, lift, release |
| `module_api.mjs` | the public `mujoco-physics.js` API end to end (22 checks) |
| `grasp_random_yaw.mjs` | 8/8 randomised cell orientations grasped and lifted |
| `dock_retention.mjs` | the cradle holds a cell under gravity for 7.5 s |
| `dock_insertion.mjs` | full carry → seat → release → withdraw, cell stays docked |
| `valve_turn.mjs` | the arm winds the handwheel past 90° with no stem actuator |
| `smoothness.mjs` | TCP jerk, analytic vs MuJoCo, over three trajectory types |
| `browser_station0.mjs` | the real page: grasp cycle, physics panel readouts |
| `browser_stations12.mjs` | the real page: charger and valve tasks complete |
| `browser_render_fidelity.mjs` | rendered pose == solved pose to 0.00 µm |
| `browser_fallback.mjs` | the `Kinematic` toggle and switching back |

**The browser suites are the ones that catch integration bugs.** Several defects passed every Node test and failed only in Chrome, because the Node tests drive the ideal-IK arm while the studio's right rig follows the lagging firmware stream.

---

## 6. Pre-existing architecture (unchanged by the physics work)

### 6.1 `server.mjs` — room relay and the authoritative pose sanitizer
HTTP + WebSocket relay (`/api/room/:code/ws`, `/poll`, `/imu`) with three roles: `deck` (desktop studio), `phone` (Apple Watch / iPhone), `isaac` (Shadow PC / Omniverse bridge, 256 KB frame limit).

**`Room.prototype.toDecks` is a safety filter, not a convenience.** Apple Watch WebKit has no reload button or URL bar, so when a new Cloud Run revision deploys the Watch reconnects its WebSocket while still running the **old cached `phone.js` in Watch RAM**. Older `phone.js` (`185c477`..`3a2f9be`) used a frame-to-frame 3D difference `da3D` with thresholds of 0.20–0.95 m/s², which fired false grabs on any wrist tilt — rotating a 1 g gravity vector by 5.5° already produces ≈0.95 m/s². So the server:

1. Preserves full `v.yaw` / `v.pitch` responsiveness (2.4× gain) while forcing `v.roll = 0` and `v.squeeze = 0`, so the wrist never twists on its own.
2. Strips `v.grabToggle` / `v.grabSeq` unless it is an explicit button tap (`v.gesture` starting with `"Button"`) **or** the raw IMU shows a rotation-invariant norm impulse:
   $$\text{aDelta} = \left|\sqrt{a_x^2 + a_y^2 + a_z^2} - 9.80665\right| > 1.75\ \text{m/s}^2 \quad\text{while}\quad \|\boldsymbol{\omega}\| < 1.90\ \text{rad/s}$$
3. Pure rotation preserves $\|\mathbf a\|$ (aDelta < 0.35), so tilting to steer never triggers a grab, while a deliberate tendon clench spikes well past 1.75.

### 6.2 `public/phone.js` / `phone.html`
Apple Watch / iPhone controller. `deskGain = 2.4`, zero roll-twist output (`twistDeg = 0`), roll-to-yaw steering 0.45, and the same rotation-invariant impulse detector (`aDelta > 1.75 && gMag < 1.90`, 950 ms debounce).

### 6.3 `sim/shadow_rtx_bridge.py`
Omniverse Isaac Sim RTX + PhysX WebSocket bridge (with a `--local-sim` $0 Mac fallback). Streams 20 Hz base64 JPEG frames plus PhysX `IMUSensor` telemetry into `#rtx-stream-overlay` via `role=isaac`. When connected it **outranks** the MuJoCo forearm IMU (see the sensor priority in `tick()`).

### 6.4 Rendering
WebGL2 PBR with soft shadows — **not** a path tracer. RTX frames come from the Omniverse bridge above. What is guaranteed is that the pixels are the solver's state (§2.3, last row).

---

## 7. Tuning guide

### Watch tendon-clench sensitivity
Lower the `aDelta` threshold in **three places together**: `server.mjs` (`toDecks`), `public/phone.js`, and `public/teleop-studio.js` (currently 1.75 / 1.75 / 1.65) to e.g. 1.20–1.40 m/s².
**Never** go below 0.85 m/s², and **never** use raw `da3D` or `pose.gx` with thresholds under 1.2 — normal wrist tilt rotates the gravity vector and will cause false auto-grasps.

### Tilt reach and speed
Adjust the `targetY` (`/30`) and `targetZ` (`/28`) divisors and `kp` (14.5) in `tick()`. Because the Watch caches `phone.js` in RAM until the app is force-closed, **always apply gain or gesture changes in `server.mjs` (`toDecks`) and `public/teleop-studio.js`**, not only in `public/phone.js`.

### Physics tuning
- Grip force: `grip_act` `kp` in the MJCF (1800 ≈ 17 N). Raising it increases pad compression on the cell.
- Arm compliance: per-joint `kp`/`kv`/`forcerange` on the `<position>` actuators.
- Contact hardness: `solref`/`solimp` on the relevant geom class. The elliptic friction cone with `impratio=10` is what makes grasping work at all — the default pyramidal cone lets the cell shear out from between the pads.

### ⚠ Always bump the cache version
`?v=NN` in **both** `public/index.html` and `public/teleop-studio.html`, plus the `mujoco-physics.js` and model-URL query strings inside `teleop-studio.js`. Currently `?v=24`. Stale cached JS has bitten this project repeatedly.

---

## 8. Subagents spawned in earlier sessions

1. **`omniverse-rtx-materials-expert`** (`53f3c564-453d-4065-9f06-761a825a6b40`) — Omniverse RTX & OpenUSD materials. Upgraded `sim/shadow_rtx_bridge.py` with a `/World/Looks` `UsdPreviewSurface` library, 3-point `UsdLux` lighting, `carb.settings` `/rtx/*` configuration, and 6-DOF dual-arm USD articulation.
2. **`wrist-tendon-imu-expert`** (`4bd7ab2e-fa9d-4227-b8cd-b04339106a51`) — biomechanical wrist/tendon IMU teleop. Found the two root causes of persistent auto-grasping: Watch WebKit caching old `phone.js` across deploys, and `da3D` measuring gravity-vector rotation rather than true linear acceleration.
