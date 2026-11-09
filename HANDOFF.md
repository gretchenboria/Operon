# Operon — Agent Handoff Document (`HANDOFF.md`)

**Last Updated:** 2026-11-08 22:36 PST  
**Author / Owner:** Gretchen Boria, PhD  
**Workspace:** `/Users/dr.gretchenboria/Operon`  
**GitHub Repository:** `https://github.com/gretchenboria/Operon.git` (`main` branch)

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

## 2. Architecture & Key Files

1. **[`server.mjs`](file:///Users/dr.gretchenboria/Operon/server.mjs)** — Node.js HTTP + WebSocket room relay (`/api/room/:code/ws`, `/api/room/:code/poll`, `/api/room/:code/imu`).
   - Supports three roles: `deck` (Desktop 3D Studio), `phone` (Apple Watch / iPhone controller), and `isaac` (Shadow PC / Omniverse RTX + PhysX bridge, 256 KB frame limit).
   - **Critical Server-Side Pose Sanitizer (`Room.prototype.toDecks`, lines 126–170):**
     - **Why it exists:** Apple Watch WebKit (opened via QR or SMS link to `/phone.html?room=OPERON`) has **no browser reload button or URL bar**. When a new Cloud Run revision deploys, the Watch's existing WebSocket automatically reconnects (`setTimeout(connect, ...)` in `phone.js`) while still executing the **old cached `phone.js` in Watch RAM**!
     - Earlier versions of `phone.js` (`185c477`..`3a2f9be`) used frame-to-frame 3D vector difference `da3D = hypot(ax - prevAx, ...)` with low thresholds (`0.20–0.95 m/s²`), which fired false `grabSeq` / `grabToggle` events whenever the user tilted their wrist (because rotating a $1g$ gravity vector by just $5.5^\circ$ produces $\text{da3D} = 9.81 \cdot \Delta\theta \approx 0.95\text{ m/s}^2$).
     - Therefore, `toDecks(msg)` in `server.mjs` acts as the **authoritative server-side filter**:
       1. Normalizes any legacy `2.4x` / `3.2x` Watch gain down to `1.5x` and forces `v.roll = 0` so the 6-DOF robot wrist never twists on its own.
       2. Strips any `v.grabToggle` / `v.grabSeq` coming from the Watch unless it is an explicit UI button tap (`v.gesture` starting with `"Button"`) OR the raw IMU stream in `v.ax, v.ay, v.az` exhibits a **rotation-invariant 3D norm impulse**:
          $$\text{aDelta} = \left|\sqrt{a_x^2 + a_y^2 + a_z^2} - 9.80665\right| > 1.75\text{ m/s}^2 \quad \text{while} \quad \|\boldsymbol{\omega}\| < 1.90\text{ rad/s}$$
       3. Because pure 3D rotation of the wrist preserves $\|\mathbf{a}\| \approx 9.80665\text{ m/s}^2$ ($\text{aDelta} < 0.35\text{ m/s}^2$), tilting the wrist to steer **never** triggers a false grab, whereas a deliberate forearm tendon clench / hand grasp impulse produces a radial mechanomyographic (MMG) + linear spike ($\text{aDelta} > 1.75\text{ m/s}^2$).

2. **[`public/teleop-studio.js`](file:///Users/dr.gretchenboria/Operon/public/teleop-studio.js)** — Desktop 3D Dual-Arm Kinematic & Sim-to-Real Workbench (`index.html` / `teleop-studio.html`).
   - **No Passive Proximity Magnet in `evaluateArm()` (lines 1180–1205):**
     - Previously, `evaluateArm()` had `if (aligned && closed) { st.arm[4].held = true; st.arm[6].held = true; }` running at 60 Hz, which caused the battery cell to get sucked into the gripper like a magnet if the jaws were already closed when passing near the cell. This has been **completely removed**.
   - **Authoritative `onLive` Pose Handler (lines 1321–1370):**
     - Initializes `p.lastGrabSeq` on the first packet from each Watch so connecting/refreshing never triggers a false grab.
     - Verifies `aDelta > 1.65 m/s² && gMag < 1.95 rad/s` (or explicit `"Button"` / `"Tendon"` gesture) before calling `triggerSmartGraspOrRelease()`.
     - Normalizes incoming `yaw`/`pitch` to `1.5x` gain and locks `roll = 0`.
   - **Pure In-Place Grasp & Release During Live Tilt (`triggerSmartGraspOrRelease`, lines 1441–1495):**
     - When `crowdPose() !== null` (`liveTilt`), grasping or releasing **never** runs `tweenTo` or `st.holdCrowd` (which previously hijacked the arm away from the user's live tilt).
     - When `!isHolding` on Station 0 (Battery Pick & Place), a grasp gesture only picks up the battery if the gripper is positioned over the cell (`dxy < 0.068 m`); if triggered in empty space away from the cell (`dxy >= 0.068 m`), it keeps the jaws open (`setClosed(false, false)`) so the jaws are never pre-closed before reaching the cell.
   - **Untwisted 1:1 Tilt Steering (`tick()`, lines 2662–2688):**
     - Keeps `targetX` centered on the pad line (`0.150 m`), maps `c.yaw` to `targetY` (left/right between Pick Pad and Place Pad) and `c.pitch` to `targetZ` (up/down height), and sets `wantYPR = [c.yaw * 0.35, 0, 0]` so the 6-DOF spherical wrist (`q4, q5, q6`) stays level (`pitch = 0, roll = 0`) and never twists on its own.

3. **[`public/phone.js`](file:///Users/dr.gretchenboria/Operon/public/phone.js) & [`public/phone.html`](file:///Users/dr.gretchenboria/Operon/public/phone.html)** — Apple Watch / iPhone Web Controller (`?v=21`).
   - Uses 1:1 calm gain (`deskGain = 1.5`), zero roll-twist output (`twistDeg = 0`), minimal roll-to-yaw cross-coupling (`0.22`), and the rotation-invariant forearm tendon impulse detector (`aDelta > 1.75 && gMag < 1.90` with `950 ms` debounce).

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
     1. Apple Watch WebKit keeping old cached `phone.js` in memory across Cloud Run deploys (solved by moving authoritative `aDelta` filtering and angle normalization into `server.mjs` `toDecks()` and `public/teleop-studio.js`).
     2. `da3D` measuring gravity vector rotation ($9.81 \cdot \Delta\theta$) during normal wrist tilt, whereas `aDelta = |hypot(ax,ay,az) - 9.80665|` is mathematically rotation-invariant.

---

## 4. Next Steps / Tuning Guide for Next Agent

- **If the user wants the tendon clench to be MORE sensitive (easier to trigger):**
  - Lower the `aDelta` threshold in **three places** together (currently `1.75` in `server.mjs` line 156 and `public/phone.js` line 445, and `1.65` in `public/teleop-studio.js` line 1338) to e.g. `1.25–1.40 m/s²`.
  - **Important:** Never lower `aDelta` below `0.85 m/s²`, and **never** use raw `da3D` or `pose.gx` (gyro roll) with low thresholds (`< 1.2`), as normal wrist tilting rotates the gravity vector and will cause false auto-grasps.
- **If the user wants to adjust tilt reach/speed:**
  - Adjust `gainScale` or `targetY` / `targetZ` divisors in [`public/teleop-studio.js`](file:///Users/dr.gretchenboria/Operon/public/teleop-studio.js#L2662-L2672) and the `scale` factor in [`server.mjs`](file:///Users/dr.gretchenboria/Operon/server.mjs#L139) (`toDecks`). Because the Watch caches `phone.js` in RAM until the Watch app is force-closed, **always apply any gain or gesture changes in `server.mjs` (`toDecks`) and `public/teleop-studio.js`**, not just in `public/phone.js`!
