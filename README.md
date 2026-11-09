# Operon: 6-DOF Sim-to-Real Kinematic & IMU Calibration Workbench

Operon is an open, non-commercial engineering workbench for comparing an ideal **6-DOF Simulated UR5e** against a **6-DOF Real / IMU-Conditioned UR5e Twin** side by side (with an optional **6-DOF Single** view and a **4-DOF vs 6-DOF** kinematic constraint comparison mode).

## Technical Value Proposition: What Operon Actually Does (and Does Not Do)

### What Operon Does
1. **Side-by-Side 6-DOF Sim vs. 6-DOF Hardware Streamer Comparison**:
   - **Left Rig (6-DOF Ideal Sim)**: Solves exact analytical inverse kinematics ($q_1 \dots q_6$) and zero-noise forward kinematics at 60 Hz.
   - **Right Rig (6-DOF Hardware Twin)**: Implements the exact `AbelArmHardwareInterface` open-loop segment streamer (`command_deadband_deg = 1.0`, `min_send_interval_ms = 100`, `segment_time_ms = 150` on-device 5th-order quintic interpolation, and integer servo degree quantization $\text{servo\_deg} = \text{round}(\text{clamp}(90 + q \cdot 180/\pi, 0, 180))$) so you can measure position lag ($\text{mm}$) and orientation residual ($^\circ$).
   - **Comparison Modes**: Switch between **6D Sim vs 6D Real** (default), **Single 6-DOF**, or **4-DOF vs 6-DOF** (planar wrist lock vs. spherical wrist).
2. **Canonical Firmware YAML Trajectory Replay & Forearm 6-Axis IMU Calibration (`sim_imu.csv` vs `watch.csv`)**:
   - Loads the real `config/trajectories/*.yaml` files (`imu_calibration.yaml`, `wave.yaml`, `nod_yes.yaml`) via `/api/trajectories` using the exact firmware servo-90 convention (`home: [90, 90, 90, 90]`, `URDF_rad = (servo_deg - 90) * pi / 180`, and `duration_ms` clamped to `quintic.min_duration()`).
   - Computes analytical proper acceleration $\mathbf{a}_{\text{site}} = R^T(\ddot{\mathbf{p}} + \mathbf{g}_{\text{up}})$ and body angular velocity $\boldsymbol{\omega} = (R^T \dot{R})^\vee$ at the forearm sensor site (`sim/isaac_replay.py:forearm_fk`).
   - When a real `SensorLog` or `Sensor Logger` CSV is loaded or a live `DeviceMotion` stream is active, solves the 3x3 Kabsch/Wahba SVD rotation matrix $R_{\text{watch}\to\text{sim}}$ (`compare.py:kabsch`), reporting live **RMSE**, **MAE**, **Pearson $r$**, and **Welch PSD**. When no Watch input is connected, no synthetic noise is fabricated (`N/A` is reported).
3. **1:1 Right-Handed Teleop & Continuous Spherical Wrist IK**:
   - Dual-branch $ZYZ$ wrist decomposition (`ik6` in `public/demos/arms3d.js`) selects the shortest angular path $(q_4, q_5, q_6)$ relative to the previous step so the wrist never flips $180^\circ$ at $q_5 \approx 0$.
   - Stream live 6-axis `DeviceMotion` / `DeviceOrientation` from an iPhone or Apple Watch over WebSockets/HTTP, or upload a recorded `SensorLog` / `Sensor Logger` CSV.
4. **Standard Robotics Exports**:
   - Export time-synchronized `sim_imu.csv` + `watch.csv`, HuggingFace LeRobot episodes (`.jsonl`), OpenUSD articulation stages (`.usda`), and perception AOV bounding boxes (`.json`).

5. **MuJoCo 3.x Rigid-Body Contact Physics (in-browser)**:
   - Runs the official Google DeepMind MuJoCo WebAssembly bindings (npm [`mujoco`](https://github.com/google-deepmind/mujoco/tree/main/wasm), Apache-2.0) directly in the studio, one independent `mjModel`/`mjData` per arm rig, at a $0.5\,\text{ms}$ timestep (~$1.5\,\text{ms}$/frame for both arms).
   - Both arms are driven by **position actuators**, not by writing `qpos`: the analytic `ik6` solution becomes the *servo setpoint*, and what is rendered is what the solver achieved under real gravity, link inertia, joint damping and contact. The left rig tracks the ideal IK, the right rig tracks the firmware quintic streamer, so sim-vs-real compares two genuinely dynamic arms.
   - The Ø24 mm × 54 mm cell is a **free rigid body held by friction alone** between two nitrile pads (`condim=6`, elliptic friction cone, `impratio=10`). The jaws stop on the cell at its own width and compress the pad into it; on release the cell **falls under gravity** and settles. There is no welding, no tool parenting, no teleport-to-destination and no proximity magnet.
   - "Holding" is ground truth from the pad `touch` sensors, not a distance heuristic. Jaw gap (the true pad-to-pad opening), grip force, per-pad normal load, TCP force/torque and contact count are surfaced live and recorded into every LeRobot step under a `physics` block.
   - The approach yaw is derived from the cell's own solved axis, so a randomly perturbed cell is still gripped across its curved wall rather than on an end cap (verified: 8/8 random poses grasped and lifted).
   - Smoothness, measured as TCP jerk, improves on **every** trajectory type, not just step inputs: RMS jerk −32 % on the full scripted pick-and-place, −56 % on continuous live wrist-tilt steering, −93 % on a hard step command.
   - The MJCF chain in `public/models/operon_station0.xml` reproduces `fk6()` from `public/demos/arms3d.js` **to under a nanometre**, so the physical and analytic paths are interchangeable; a `Kinematic` toggle switches back for A/B comparison, and the studio falls back to it automatically if the WASM cannot load.

### Engineering Reality & Scope Limitations
- Operon is a **kinematic, trajectory, and inertial calibration workbench** that now also runs a real frictional rigid-body contact solver in the browser (MuJoCo 3.x via WebAssembly) for the arm dynamics and for all three station tasks.
- **All three stations are physically simulated.** Both arm chains at every station (gravity, link inertia, joint damping, torque-limited position servos, self-collision excluded by design), the gripper's coupled finger drive, and every workcell prop as solid collision geometry. Only the active station's props stay collidable; the rest are parked, so the arm never hits geometry the renderer is hiding.
  - **Station 0 — pick & place:** the cell is grasped, carried, slipped and dropped by pad friction alone.
  - **Station 1 — charger dock:** the cell is carried to the dock and lowered into a cradle that **retains it by gravity and the pocket walls**, not by the gripper. After the arm withdraws it stays seated 4.5 mm from the slot centre. *Caveat: the scripted insertion completed 7 of 8 consecutive headless-browser runs. The physics is deterministic — insertion and retention pass every time in isolation — and the residual failure is in the scripted approach, when the initial clamp seats poorly and the retry does not recover. Teleoperated insertion is unaffected.*
  - **Station 2 — valve:** there is **no actuator on the valve stem**. The arm engages the handwheel hub, clamps, and winds it 0 → 91° purely through contact, against the stem's own 0.14 N·m breakaway friction; it holds ~95° after release.
- **Three physics-driven corrections** were needed to stations that were never physically valid, because the old kinematic path teleported objects into place and so never had to be reachable:
  1. The charger faced partly **downward** — its outward normal put +4.15 m/s² of gravity along the opening and ejected every released cell. It now faces up at a 25° compound tilt (−8.89 m/s² into the pocket).
  2. A station's **body frame and approach frame are now separate** (`app`). `ST[1].R`'s +z is the dock's outward normal, but a tool's +z is its approach axis, so aiming the arm at the body frame put the wrist centre 60 mm *inside* the dock.
  3. The dock was **out of reach** and had to be relocated to `[0.10, 0.090, 0.040]`; the wrist centre, not the TCP, is what must stay inside the 200 mm envelope.
- **Rendering:** WebGL2 PBR with soft shadows, not a path tracer; RTX frames come from the optional `sim/shadow_rtx_bridge.py` Omniverse bridge. What *is* guaranteed is that the pixels are the solver's state: an automated check over a full pick-and-place compares the rendered cell transform and jaw aperture against MuJoCo's own and finds them identical to 0.00 µm.
- Its practical utility is **pre-hardware verification**: validating coordinate frame transforms ($R_{\text{watch}\to\text{sim}}$), catching spherical wrist singularities ($q_5 \to 0$), measuring teleop latency, verifying 5th-order polynomial trajectory limits ($\dot{q}_{\max} = 120^\circ/\text{s}$, $\ddot{q}_{\max} = 200^\circ/\text{s}^2$) using a \$0 wearable IMU, and now sanity-checking whether a grasp is mechanically plausible at all — before running on a physical manipulator.

## Local Usage

```bash
npm install
npm start
# Open http://localhost:8080
```

## License

Released under the **Personal Testing & Non-Commercial Research License** (see [LICENSE](./LICENSE)). Free for personal testing, academic study, and non-commercial research.
