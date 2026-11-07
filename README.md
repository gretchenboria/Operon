# Operon · Omniverse RTX Teleop & Sim-to-Real Telemetry Studio

**Compare real/watch telemetry vs. simulation in the browser — even before you have physical robot hardware.**

Operon unifies three pieces of the Physical AI stack into a zero-GPU-burn cloud web studio:

1. **Phone & Apple Watch Teleop + Live IMU Relay (`public/phone.js`, `server.mjs`)**:
   - Scan a QR code from any phone or load an Apple Watch **SensorLog / Sensor Logger** `.csv` (`⌚ Synth Watch Run` or `📂 Load Watch CSV`).
   - Streams 6-DOF pose commands + live `DeviceMotion` accelerometer ($\text{m/s}^2$ with gravity reaction) and gyroscope ($\text{rad/s}$) + physical tap-sync fiducials ($|\mathbf{a}| > 16.5\text{ m/s}^2$) over WebSockets.
2. **Firmware Quintic Contract & Forearm IMU Digital Twin (`public/teleop-studio.js`)**:
   - Implements the exact 5th-order firmware quintic trajectory generator (`MAX_VELOCITY = 120°/s`, `MAX_ACCELERATION = 200°/s²`, $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$) and `imu_calibration.yaml` waypoints from `abel_sim2real`.
   - Predicts analytic forearm IMU at `/World/abel_arm/forearm_link/forearm_imu` ($\mathbf{a}_{\text{site}} = R^T(\ddot{\mathbf{p}} + \mathbf{g}_{\text{up}})$, $\Omega = R^T \dot{R}$), aligns frames via Wahba/SVD ($37^\circ$ strap mount), and computes live **RMSE**, **MAE**, **Pearson $r$**, and **Welch log-PSD**.
3. **Omniverse Replicator AOVs, NemoFleet Gate & Dataset Export**:
   - Multi-pass viewport: **RTX Lit (PBR)**, **Linear Depth ($Z$)**, **World Surface Normals**, **Semantic ID**, **Instance ID**, plus live **2D/3D Bounding Boxes** and Domain Randomization.
   - **NemoFleet System 1 / System 2 Action-Probability Gate** ($\tau = 0.78$, teacher escalation cost tracking at $\$0.0017$/call).
   - One-click export of **LeRobot Episodes (`.jsonl`)**, **OpenUSD Scene + TimeSamples (`.usda`)** with `PhysicsArticulationRootAPI` and `IsaacImuSensor`, **Replicator COCO (`.json`)**, and `compare.py`-compatible `sim_imu.csv` + `watch.csv`.

## Cloud Cost Guardrails (`$0` Idle / Free-Tier Cloud Run)

Operon is configured to run on Cloud Run with strict cost caps:
- `--min-instances 0` (scales to zero immediately when idle — `$0.00` idle cost)
- `--max-instances 1` (hard cap on scaling)
- `--cpu 1 --memory 512Mi` (fits inside the Cloud Run free tier; all RTX/PBR & Replicator AOV shaders run on the client's WebGL2 GPU)

```bash
npm run deploy
```
