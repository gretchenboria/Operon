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

### Engineering Reality & Scope Limitations
- Operon is a **kinematic, trajectory, and inertial calibration workbench**, not a full rigid-body contact solver (it does not run MuJoCo/PhysX frictional contact dynamics in the browser).
- Its practical utility is **pre-hardware verification**: validating coordinate frame transforms ($R_{\text{watch}\to\text{sim}}$), catching spherical wrist singularities ($q_5 \to 0$), measuring teleop latency, and verifying 5th-order polynomial trajectory limits ($\dot{q}_{\max} = 120^\circ/\text{s}$, $\ddot{q}_{\max} = 200^\circ/\text{s}^2$) using a $0 wearable IMU before running on a physical manipulator.

## Local Usage

```bash
npm install
npm start
# Open http://localhost:8080
```

## License

Released under the **Personal Testing & Non-Commercial Research License** (see [LICENSE](./LICENSE)). Free for personal testing, academic study, and non-commercial research.
