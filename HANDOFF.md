# Operon — Agent Handoff Document (`HANDOFF.md`)

**Last Updated:** 2026-11-09 PST  
**Author / Owner:** Gretchen Boria, PhD  
**Workspace:** `/Users/dr.gretchenboria/Operon`  
**GitHub Repository:** `https://github.com/gretchenboria/Operon.git` (`main` branch)  
**Latest Live Cloud Run Revision:** `operon-00041-c4q` (commit `decdd72`, cache version `?v=22`)  
**Local HEAD (not yet deployed):** MuJoCo 3.x contact physics on all three stations, cache version `?v=24`

---

## 1. Live Production & Cloud Infrastructure

- **Google Cloud Run Service URL:** `https://operon-421717890494.us-central1.run.app`
  - **Service Name:** `operon`
  - **GCP Project ID:** `gen-lang-client-0221638641` ("Astro")
  - **Region:** `us-central1`
  - **Billing Account:** `01A844-CEA6E2-BAF517` (~$300 trial credit account — **WATCH CREDITS CAREFULLY!** Do NOT spin up paid GPU Compute Engine VMs unless explicitly instructed by the user; verify `gcloud compute instances list --project=gen-lang-client-0221638641` shows `0` running instances).
  - **Approved GPU Quotas in `us-central1`:** `NVIDIA_L4_GPUS: 1.0`, `PREEMPTIBLE_NVIDIA_L4_GPUS: 1.0` (currently 0 instances running, $0/hr).
- **Deploy Command:**
  ```zsh
  gcloud run deploy operon --source . --region us-central1 --project gen-lang-client-0221638641 --allow-unauthenticated --quiet
  ```
- **Untracked Local File to NEVER Commit:**
  - `/Users/dr.gretchenboria/Operon/Screen Recording 2026-11-08 at 4.35.49 PM.mov`

---

## 1b. MuJoCo 3.x Rigid-Body Contact Physics (NEW, `?v=24`)

The studio no longer fakes the grasp. It runs the **official Google DeepMind MuJoCo WebAssembly bindings** (npm [`mujoco@3.1.16`](https://github.com/google-deepmind/mujoco/tree/main/wasm), Apache-2.0) in the browser. There is **no MuJoCo 4** — 3.x is the current line.

- **[`public/models/operon_station0.xml`](file:///Users/dr.gretchenboria/Operon/public/models/operon_station0.xml)** — the MJCF scene. One instance is compiled **per arm rig** (two independent `mjModel`/`mjData`), so the rigs can never contact each other.
  - The chain reproduces `fk6()` from `arms3d.js` **to under a nanometre** (verified: worst case 0.00 nm position, 6.66e-16 orientation). A 4-DOF arm is the same chain with `wrist_roll` and `tool_roll` held at 0, which reproduces `fk4()` exactly.
  - Arm is driven by **position actuators** (`kp` 16–150, `forcerange` 6–42 N·m). The analytic `ik6` result is the *servo setpoint*; what renders is what the solver achieved.
  - **Collision bitmasks are load-bearing:** arm `contype=1 conaffinity=6`, environment `2/5`, cell `4/3`. Without them the gripper housing self-collides with the forearm capsule and **physically jams the wrist at ~48° of tracking error**. Do not remove.
  - Grip: two finger slides coupled by an `<equality>` joint constraint (single-drive 2F-85 linkage), one `position` actuator, `forcerange ±85 N`.
  - Cell: Ø24 mm × 54 mm, 45 g **free body**, `condim=6`, elliptic friction cone, `impratio=10`. Held by friction only. Rest quat `Rz(-25°)·Rx(90°)` lays its axis parallel to the hand's Y so the jaws close across the curved wall.
- **[`public/mujoco-physics.js`](file:///Users/dr.gretchenboria/Operon/public/mujoco-physics.js)** — loader + per-arm `ArmSim` wrapper (`setArmTarget`, `setGripTarget`, `advance`, `read`, `placeCell`, `resetCell`, `recover`, `dispose`). Caches every Embind handle once (they are WASM heap objects, not GC'd). Fails safe: `available=false` keeps the studio on the analytic path.
- **`server.mjs`** serves the 8.2 MB `.wasm` from `node_modules/mujoco` at `/vendor/mujoco/` with `application/wasm` (streaming compile) and `Cache-Control: immutable`. It is **not** committed to the repo.

### What this fixed
| | before | after |
|---|---|---|
| grasp | distance heuristic + proximity snap | pad friction; `held` from `touch` sensors |
| charger dock | cell teleported into the slot | lowered into a cradle, retained after withdrawal |
| valve | angle inferred from tool roll | wound 0→91° by contact, no stem actuator |
| jaws | binary 0/1 eased snap, closed *through* the cell | stop at the cell's width, 0.4 mm pad compression |
| release | cell teleported to the destination pad | **falls under gravity** (67 mm in 0.12 s) |
| RMS TCP jerk (pick & place) | 43.1 m/s³ | **29.2 m/s³ (−32 %)** |
| RMS TCP jerk (live wrist tilt) | 108 m/s³ | **47.1 m/s³ (−56 %)** |
| peak TCP jerk (hard step) | 62 217 m/s³ | **3 835 m/s³ (−94 %)** |
| peak TCP speed | 8.88 m/s (impossible) | 1.32 m/s |
| forearm IMU | finite-difference estimate | real MuJoCo `accelerometer`/`gyro` sensor |

### Gotchas for the next agent
1. **`PHYS_GRASP_H = 0.011` is not arbitrary.** Finger pads reach 14 mm below the TCP. Grasping at the cell's centre drives the fingertips into the 1 mm locating pad; one pad then reads a ~48 N crush load against the table, the jaws jam open, and the grasp fails on the lift. This only shows up on the **right-hand rig**, which follows the firmware quintic streamer (1° deadband ⇒ several mm of tool error) — the ideal-IK left rig hides the bug. It was caught only by the headless-browser test, never by the Node tests.
2. **Clamp on arrival, never on a timer.** `physWaitArrival()` polls the achieved TCP. A fixed `setTimeout` closes the jaws while the firmware-streamed arm is still travelling.
3. **`physCellGraspYaw()`** derives the approach yaw from the cell's solved axis (column 2 of `cellR`). Needed for `Perturb Cell`; a fixed −25° approach catches an end cap on a randomised cell (verified 8/8 random yaws grasp and lift).
4. **All three station tasks are physics-driven now.** Station props are **parked below the table when inactive** (`ArmSim.setStation`) — required, not cosmetic: the renderer hides inactive props, and leaving them solid makes the arm clip the charger dock mid-traverse. Any test that builds the model directly instead of through `ArmSim` must park them too.
   - **Station 1 was redesigned, and the scene layout changed.** The dock now faces UP (normal `(0, 0.423, 0.906)`) and sits at `p = [0.10, 0.090, 0.040]`. Three separate defects had to be fixed together, and fixing only one at a time still fails:
     a. Facing down, gravity put +4.15 m/s² along the opening and ejected every released cell.
     b. `ST[1].R`'s +z is the dock's OUTWARD normal while a tool's +z is its APPROACH axis, so aiming the arm at the body frame put the wrist 60 mm inside the dock. Stations now expose `app` for the approach frame; `s.tgtR = s.app || s.R` is what `armTarget`/`evaluateArm` use, while `s.R` still draws the prop.
     c. At the authored height the wrist centre fell outside the 200 mm reach — the TCP is not what has to be reachable, the wrist is.
   - **The insertion path is staged on purpose.** Lift → re-orient IN PLACE → traverse ABOVE the dock → descend. Blending position and orientation together walks the arm through poses it cannot track (50 mm short); a straight line from the safe point to the dock clips the cradle and strips the cell out of the jaws.
   - **Station 2 has no actuator on the stem.** The arm engages the hub and rolls the tool about the wheel axis. This must command R directly (`st.cmdR`) — routed through yaw/pitch/roll the same motion only reached 36° before unwinding, because `yprOf()` cannot represent a clean roll about world +X. Gripping the 11 mm rim or a 6.4 mm spoke does NOT work: the hand is too large for a 62 mm wheel and jams on the spokes.
   - **Known gap: the scripted Station 1 insertion completes ~3 runs in 4.** The physics is not the problem — `retain.mjs` and `dock3.mjs` pass deterministically, and the headless-browser run seats the cell 4.5 mm from the slot when it works. The variance is in the scripted approach: if the initial clamp seats poorly the carry delivers a badly-held cell and the retries do not always recover. Next thing to try is gating the carry on a measured grip quality (pad load AND cell-to-TCP distance stable over several frames) rather than the single `held` check, and/or re-grasping from a known pose instead of wherever the cell ended up. Teleoperated insertion is unaffected.
   - Scripted sequences **verify, never assume**: `physWaitArrival` before clamping, and the dock sequence re-reads the cell pose after the approach and retries the clamp up to 3× if the pads did not take it. A fixed timer clamps while the firmware-streamed arm is still travelling.

5. **Grip force and the `held` flag.** `grip_act` runs at `kp=1800` (~17 N clamp); `kp=900` gave only ~9 N and a carried cell slid between the pads during a re-orientation. `held` uses the **total** pad load `(padL + padR) > 0.4 N`, not both pads individually — re-orienting shifts the cell onto one pad and momentarily unloads the other, which the old test reported as a drop while the cell was still 2 mm from the TCP.
6. **`jawGapMm` is the pad-to-pad gap, not finger-body separation.** The pads close to 3.5 mm; the finger bodies are 18 mm apart at full close. An earlier version reported the latter and made a correct 24 mm grasp look like a failure.
7. Cost is ~1.5 ms/frame for both arms at the 0.5 ms timestep (~9 % of a 60 fps budget).
8. `window.__operonRenderProbe()` is a read-only hook returning the rendered vs solved cell pose and jaw aperture. Remember `T3(x,y,z) = (x, z, -y)`: the scene is Y-up, the arm is Z-up, and the rig's display offset lands on three's **z**.

### Test harnesses (scratchpad, not committed)
`validate.mjs` (MJCF vs `fk6`, 13 checks) · `module_test.mjs` (module API, 22 checks) · `cdp_test.mjs` (real Chrome over DevTools Protocol, drives the page and reads the physics panel) · `smooth.mjs` (jerk comparison). The CDP test is the one that catches integration bugs.

---

## 2. Architecture & Key Files (Current State in `?v=22` / `decdd72`)

1. **[`server.mjs`](file:///Users/dr.gretchenboria/Operon/server.mjs)** — Node.js HTTP + WebSocket room relay (`/api/room/:code/ws`, `/api/room/:code/poll`, `/api/room/:code/imu`).
   - Supports three roles: `deck` (Desktop 3D Studio), `phone` (Apple Watch / iPhone controller), and `isaac` (Shadow PC / Omniverse RTX + PhysX bridge, 256 KB frame limit).
   - **Critical Server-Side Pose Sanitizer (`Room.prototype.toDecks`, lines 126–165):**
     - **Why it exists:** Apple Watch WebKit (opened via QR or SMS link to `/phone.html?room=OPERON`) has **no browser reload button or URL bar**. When a new Cloud Run revision deploys, the Watch's existing WebSocket automatically reconnects (`setTimeout(connect, ...)` in `phone.js`) while still executing the **old cached `phone.js` in Watch RAM**.
     - Earlier versions of `phone.js` (`185c477`..`3a2f9be`) used frame-to-frame 3D vector difference `da3D = hypot(ax - prevAx, ...)` with low thresholds (`0.20–0.95 m/s²`), which fired false `grabSeq` / `grabToggle` events whenever the user tilted their wrist (because rotating a $1g$ gravity vector by just $5.5^\circ$ produces $\text{da3D} = 9.81 \cdot \Delta\theta \approx 0.95\text{ m/s}^2$).
     - Therefore, `toDecks(msg)` in `server.mjs` acts as the **authoritative server-side filter**:
       1. Preserves full `100%` `v.yaw` and `v.pitch` responsiveness (`2.4x` gain) while forcing `v.roll = 0` and `v.squeeze = 0` so the 6-DOF robot wrist never twists on its own.
       2. Strips any `v.grabToggle` / `v.grabSeq` coming from the Watch unless it is an explicit UI button tap (`v.gesture` starting with `"Button"`) OR the raw IMU stream in `v.ax, v.ay, v.az` exhibits a **rotation-invariant 3D norm impulse**:
          $$\text{aDelta} = \left|\sqrt{a_x^2 + a_y^2 + a_z^2} - 9.80665\right| > 1.75\text{ m/s}^2 \quad \text{while} \quad \|\boldsymbol{\omega}\| < 1.90\text{ rad/s}$$
       3. Because pure 3D rotation of the wrist preserves $\|\mathbf{a}\| \approx 9.80665\text{ m/s}^2$ ($\text{aDelta} < 0.35\text{ m/s}^2$), tilting the wrist to steer **never** triggers a false grab, whereas a deliberate forearm tendon clench / hand grasp impulse produces a radial mechanomyographic (MMG) + linear spike ($\text{aDelta} > 1.75\text{ m/s}^2$).

2. **[`public/teleop-studio.js`](file:///Users/dr.gretchenboria/Operon/public/teleop-studio.js)** — Desktop 3D Dual-Arm Kinematic & Sim-to-Real Workbench (`index.html` / `teleop-studio.html`, `?v=22`).
   - **No Passive Proximity Magnet in `evaluateArm()` (lines 1180–1205):**
     - `evaluateArm()` has **no** passive `if (aligned && closed) { st.arm[4].held = true; ... }` block, so passing near the battery cell never sucks it into the gripper like a magnet.
   - **Authoritative `onLive` Pose Handler (lines 1321–1365):**
     - Initializes `p.lastGrabSeq` on the first packet from each Watch so connecting/refreshing never triggers a false grab.
     - Verifies `aDelta > 1.65 m/s² && gMag < 1.95 rad/s` (or explicit `"Button"` / `"Tendon"` gesture) before calling `triggerSmartGraspOrRelease()`.
     - Keeps full `2.4x` `yaw`/`pitch` responsiveness and locks `roll = 0`.
   - **Pure In-Place Grasp & Release During Live Tilt (`triggerSmartGraspOrRelease`, lines 1441–1495):**
     - When `crowdPose() !== null` (`liveTilt`), grasping or releasing **never** runs `tweenTo` or `st.holdCrowd` (which previously hijacked the arm away from the user's live tilt).
     - When `!isHolding` on Station 0 (Battery Pick & Place), a grasp gesture only picks up the battery if the gripper is positioned over the cell (`dxy < 0.068 m`); if triggered in empty space away from the cell (`dxy >= 0.068 m`), it keeps the jaws open (`setClosed(false, false)`) so the jaws are never pre-closed before reaching the cell.
   - **Fast, Untwisted 1:1 Tilt Steering (`tick()`, lines 2660–2690):**
     - Keeps `targetX` centered on the pad line (`0.150 m`), maps `c.yaw` (`(c.yaw / 30) * 0.122 * gainScale`) to `targetY` (left/right between Pick Pad and Place Pad) and `c.pitch` (`0.080 + (c.pitch / 28) * 0.084 * gainScale`) to `targetZ` (up/down height) with `kp = Math.min(1, dt * 14.5)`, and sets `wantYPR = [c.yaw * 0.40, 0, 0]` so the 6-DOF spherical wrist (`q4, q5, q6`) stays level (`pitch = 0, roll = 0`) and never twists on its own.

3. **[`public/phone.js`](file:///Users/dr.gretchenboria/Operon/public/phone.js) & [`public/phone.html`](file:///Users/dr.gretchenboria/Operon/public/phone.html)** — Apple Watch / iPhone Web Controller (`?v=22`).
   - Uses responsive `deskGain = 2.4`, zero roll-twist output (`twistDeg = 0`), balanced roll-to-yaw steering (`0.45`), and the rotation-invariant forearm tendon impulse detector (`aDelta > 1.75 && gMag < 1.90` with `950 ms` debounce).

4. **[`sim/shadow_rtx_bridge.py`](file:///Users/dr.gretchenboria/Operon/sim/shadow_rtx_bridge.py)** — Omniverse Isaac Sim RTX + PhysX WebSocket Bridge (plus `--local-sim` $0 Mac fallback).
   - Streams live 20 Hz base64 JPEG frames + PhysX `IMUSensor` telemetry into `#rtx-stream-overlay` on the Studio dashboard via `wss://operon-421717890494.us-central1.run.app/api/room/OPERON/ws?role=isaac`.

---

## 3. Subagents Spawned in This Session

1. **`omniverse-rtx-materials-expert`** (`53f3c564-453d-4065-9f06-761a825a6b40`)
   - Role: Omniverse RTX & OpenUSD Materials Architect.
   - Upgraded `sim/shadow_rtx_bridge.py` with `/World/Looks` `UsdPreviewSurface` material library, 3-point `UsdLux` lighting, `carb.settings` `/rtx/*` configuration, and 6-DOF dual-arm USD articulation.
2. **`wrist-tendon-imu-expert`** (`4bd7ab2e-fa9d-4227-b8cd-b04339106a51`)
   - Role: Biomechanical Wrist & Tendon IMU Teleop Specialist.
   - Identified the two root causes of persistent auto-grasping:
     1. Apple Watch WebKit keeping old cached `phone.js` in memory across Cloud Run deploys (solved by moving authoritative `aDelta` filtering into `server.mjs` `toDecks()` and `public/teleop-studio.js`).
     2. `da3D` measuring gravity vector rotation ($9.81 \cdot \Delta\theta$) during normal wrist tilt, whereas `aDelta = |hypot(ax,ay,az) - 9.80665|` is mathematically rotation-invariant.

---

## 4. Next Steps / Tuning Guide for Next Agent

- **If the user wants the tendon clench to be MORE sensitive (easier to trigger):**
  - Lower the `aDelta` threshold in **three places** together (currently `1.75` in `server.mjs` line 151 and `public/phone.js` line 445, and `1.65` in `public/teleop-studio.js` line 1338) to e.g. `1.20–1.40 m/s²`.
  - **Important:** Never lower `aDelta` below `0.85 m/s²`, and **never** use raw `da3D` or `pose.gx` (gyro roll) with low thresholds (`< 1.2`), as normal wrist tilting rotates the gravity vector and will cause false auto-grasps.
- **If the user wants to adjust tilt reach/speed:**
  - Adjust `targetY` (`/ 30`) or `targetZ` (`/ 28`) divisors and `kp` (`14.5`) in [`public/teleop-studio.js`](file:///Users/dr.gretchenboria/Operon/public/teleop-studio.js#L2680-L2688). Because the Watch caches `phone.js` in RAM until the Watch app is force-closed, **always apply any gain or gesture changes in `server.mjs` (`toDecks`) and `public/teleop-studio.js`**, not just in `public/phone.js`!
