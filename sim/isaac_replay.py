#!/usr/bin/env python3
"""Replay a shared joint trajectory and log a simulated forearm IMU.

This is the **Shadow PC (Isaac Sim)** half of the sim-to-real loop. It drives a
4DOF arm USD through the *exact* firmware quintic trajectory (shared contract in
``config/trajectories/*.yaml``) and records a simulated IMU at the forearm link
-- the same spot where an Apple Watch is strapped on the real arm. The resulting
``sim_imu.csv`` is later compared against the watch recording by ``compare.py``.

Two execution modes
-------------------
* ``--analytic``  : pure-numpy forward-kinematics IMU predictor. **Runs anywhere**
                    (Mac, no Isaac, no GPU). Useful to sanity-check the pipeline
                    and to generate reference IMU traces for testing ``compare.py``.
* (default)       : full NVIDIA Isaac Sim physics replay. Must run inside the
                    Isaac Sim Python environment on the Shadow PC Windows VM.
                    **UNTESTED-PENDING-VM** -- written from the 4.5 docs; see the
                    version notes throughout ``run_isaac()``.

Both modes emit the identical CSV schema::

    t,ax,ay,az,gx,gy,gz

    t          seconds, 0 = start of trajectory (see "Time sync" below)
    ax,ay,az   linear acceleration / specific force, m/s^2, sensor frame
    gx,gy,gz   angular velocity, rad/s, sensor (body) frame

Time sync
---------
The **real-arm watch tap** is the time-sync marker: the operator taps the watch
hard right before the run and that impulse spike is what ``compare.py`` aligns
on. We deliberately do **not** fake a tap in sim -- an external hand-tap is not
reproducible by the joints, and injecting a synthetic spike would only pollute
the trace. Instead, sim ``t=0`` is defined as the start of the trajectory, and
``compare.py`` shifts the watch stream so its tap lands at sim ``t=0``. (If a
synthetic in-sim marker is ever wanted, the cleanest option is a short, sharp,
documented single-joint jab held at home *before* segment 1 -- add it as an extra
leading waypoint in the YAML so BOTH sides execute it, rather than special-casing
it here.)

Shared contract reused (do NOT reinvent the trajectory math):
  * ``quintic.py``    -- Quintic, min_duration  (firmware-matching 5th-order)
  * ``trajectory.py`` -- load(path), sample(traj, dt) -> (t, Q) with Q (N,4) deg
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

# Make the sibling shared-contract modules importable regardless of CWD.
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

import trajectory as traj_mod  # noqa: E402  (load, sample)

_REPO_ROOT = _THIS_DIR.parent
_DEFAULT_TRAJ = _REPO_ROOT / "config" / "trajectories" / "imu_calibration.yaml"

GRAVITY = 9.80665  # m/s^2, standard gravity (world up = +Z)


# ---------------------------------------------------------------------------
# Prim-path / kinematics configuration.
#
# Reconciled with `assets/operon_arm.usda` (generated from
# `public/models/operon_station0.xml` by `sim/mjcf_to_usd.py` and verified to
# 0.000 nm against MuJoCo in `test/usd/verify_against_mujoco.py`).
# CLI flags override the prim-path entries; link lengths / FK signs are here.
# ---------------------------------------------------------------------------
CONFIG = {
    # --- Isaac prim paths (override via CLI) ------------------------------
    "articulation_root": "/World/pedestal",
    # The 4 joint prim names in servo order 0..3 matching `assets/operon_arm.usda`
    "joint_names": ["yaw", "shoulder", "elbow", "finger_r"],
    # Prim path of the IMU sensor created as a child of the forearm_imu site
    "imu_prim_path": "/World/pedestal/base/upper_arm/forearm/forearm_imu/imu_sensor",

    # --- servo-degrees -> USD-radians mapping -----------------------------
    # The YAML/firmware speak servo degrees with 90 deg = mechanical home/centre.
    # `assets/operon_arm.usda` defines joint zero at upright neutral, so
    # offset_deg = -90 maps 90 deg servo -> 0 rad USD.
    "joint_offset_deg": [-90.0, -90.0, -90.0, -90.0],
    "joint_sign":       [1.0, 1.0, 1.0, 1.0],

    # --- analytic FK link geometry (meters, matching operon_station0.xml) --
    "base_height":    0.090,  # world floor -> shoulder pitch axis (pedestal 0.055 + base 0.035)
    "upper_arm_len":  0.100,  # shoulder axis -> elbow axis
    "forearm_len":    0.100,  # elbow axis -> watch/IMU site on the forearm

    # --- analytic FK joint axes -------------------------------------------
    # base  : yaw about world +Z. shoulder/elbow : pitch about the (yawed) Y.
    "base_yaw_sign":       1.0,
    "shoulder_pitch_sign": 1.0,
    "elbow_pitch_sign":    1.0,
}


# ---------------------------------------------------------------------------
# Shared trajectory loading.
# ---------------------------------------------------------------------------
def load_trajectory(traj_path: Path, dt: float):
    """Return (t, Q_deg) using the shared firmware-quintic contract.

    Q_deg has shape (N, 4), joint angles in DEGREES, servo order [base, shoulder,
    elbow, gripper]. The move durations are clamped up to the limit-respecting
    minimum inside ``trajectory.load`` exactly as the ESP32 firmware does.
    """
    traj = traj_mod.load(traj_path)
    t, Q_deg = traj_mod.sample(traj, dt)
    if Q_deg.shape[1] != 4:
        raise ValueError(
            f"expected 4 joints, got {Q_deg.shape[1]} from {traj_path}")
    print(f"[traj] '{traj.name}': {len(t)} steps @ {dt*1000:.1f}ms "
          f"({t[-1]:.2f}s), Q{Q_deg.shape}")
    return t, Q_deg


def servo_deg_to_urdf_rad(Q_deg: np.ndarray) -> np.ndarray:
    """Map servo degrees -> URDF joint radians per the CONFIG offset/sign."""
    sign = np.asarray(CONFIG["joint_sign"], dtype=float)
    off = np.asarray(CONFIG["joint_offset_deg"], dtype=float)
    return np.radians(Q_deg * sign + off)


# ---------------------------------------------------------------------------
# Isaac-independent ANALYTIC predictor (runs on the Mac, no Isaac).
# ---------------------------------------------------------------------------
def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def forearm_fk(Q_deg: np.ndarray):
    """Forward kinematics of the forearm/watch site from joint angles.

    Simple 3-link planar-in-a-yawing-plane model (gripper does NOT move the
    forearm, so it is ignored here):

        base     q0 -> yaw   about world +Z
        shoulder q1 -> pitch about the yawed Y axis (upper arm)
        elbow    q2 -> pitch about the yawed Y axis (forearm)

    Home (servo 90,90,90) -> yaw=0, pitches=0 -> arm points straight up (+Z),
    site frame == world frame. Returns:
        p : (N,3) world position of the site, meters
        R : (N,3,3) world<-site rotation per step
    """
    bh = CONFIG["base_height"]
    L1 = CONFIG["upper_arm_len"]
    L2 = CONFIG["forearm_len"]

    # Servo-90 = home -> work in "delta from home" radians for the FK angles.
    yaw = np.radians((Q_deg[:, 0] - 90.0) * CONFIG["base_yaw_sign"])
    ph1 = np.radians((Q_deg[:, 1] - 90.0) * CONFIG["shoulder_pitch_sign"])
    ph2 = np.radians((Q_deg[:, 2] - 90.0) * CONFIG["elbow_pitch_sign"])

    n = Q_deg.shape[0]
    p = np.zeros((n, 3))
    R = np.zeros((n, 3, 3))
    for i in range(n):
        # In-plane (x forward, z up) position before applying base yaw.
        x_local = L1 * math.sin(ph1[i]) + L2 * math.sin(ph1[i] + ph2[i])
        z_local = bh + L1 * math.cos(ph1[i]) + L2 * math.cos(ph1[i] + ph2[i])
        Rz = _rot_z(yaw[i])
        p[i] = Rz @ np.array([x_local, 0.0, z_local])
        # Site orientation: yaw then total forearm pitch.
        R[i] = Rz @ _rot_y(ph1[i] + ph2[i])
    return p, R


def analytic_imu(t: np.ndarray, Q_deg: np.ndarray):
    """Predict the forearm IMU (specific force + gyro) without Isaac.

    1. FK -> site world position p(t) and orientation R(t).
    2. Double finite-difference p(t) for proper (world) linear acceleration,
       then add the gravity reaction: an accelerometer at rest reads +g along
       world up (specific force f = a_world - g_vec, g_vec = (0,0,-g)). Express
       in the site frame:  a_meas_site = R^T (a_world + g_up).
    3. Differentiate R(t) for body-frame angular velocity:
       Omega_body = R^T Rdot (skew-symmetric) -> gyro = (Oz y, Ox z, Oy x).
    """
    p, R = forearm_fk(Q_deg)
    t = np.asarray(t, dtype=float)

    # Linear: world acceleration via two passes of np.gradient (handles the
    # tiny non-uniformities at segment boundaries by using the actual t array).
    vel = np.gradient(p, t, axis=0, edge_order=2)
    acc_world = np.gradient(vel, t, axis=0, edge_order=2)
    g_up = np.array([0.0, 0.0, GRAVITY])  # reaction the sensor feels at rest
    # a_meas_world = a_world + g_up ; rotate into each step's site frame.
    a_meas_world = acc_world + g_up
    a_site = np.einsum("nji,nj->ni", R, a_meas_world)  # R^T @ v  (transpose)

    # Angular: Rdot via finite diff, body rate Omega = R^T Rdot.
    Rdot = np.gradient(R, t, axis=0, edge_order=2)
    gyro = np.zeros((len(t), 3))
    for i in range(len(t)):
        Omega = R[i].T @ Rdot[i]
        # vee() of the skew-symmetric part (numerically symmetrize first).
        Omega = 0.5 * (Omega - Omega.T)
        gyro[i] = np.array([Omega[2, 1], Omega[0, 2], Omega[1, 0]])

    return a_site, gyro


def run_analytic(args):
    t, Q_deg = load_trajectory(args.traj, args.dt)
    accel, gyro = analytic_imu(t, Q_deg)
    write_csv(args.out, t, accel, gyro)

    # Quick at-rest gravity sanity report (first sample is a held-home baseline).
    rest_mag = float(np.linalg.norm(accel[0]))
    print(f"[analytic] wrote {args.out}")
    print(f"[analytic] |accel| at t=0 (rest) = {rest_mag:.4f} m/s^2 "
          f"(expect ~{GRAVITY:.3f})")
    print(f"[analytic] finite values: "
          f"{bool(np.all(np.isfinite(accel)) and np.all(np.isfinite(gyro)))}")
    return 0


# ---------------------------------------------------------------------------
# CSV writer (shared by both modes).
# ---------------------------------------------------------------------------
def write_csv(out_path: Path, t, accel, gyro):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "ax", "ay", "az", "gx", "gy", "gz"])
        for i in range(len(t)):
            w.writerow([f"{t[i]:.6f}",
                        f"{accel[i, 0]:.6f}", f"{accel[i, 1]:.6f}",
                        f"{accel[i, 2]:.6f}",
                        f"{gyro[i, 0]:.6f}", f"{gyro[i, 1]:.6f}",
                        f"{gyro[i, 2]:.6f}"])


# ---------------------------------------------------------------------------
# Full ISAAC SIM replay  (Shadow PC VM only -- UNTESTED-PENDING-VM).
#
# Targeted version: Isaac Sim 4.5 (also annotated for 5.x / 6.0 deltas).
#
# Import paths verified against the 4.5 docs:
#   from isaacsim import SimulationApp                 # MUST be first
#   from isaacsim.core.api import World
#   from isaacsim.core.utils.stage import open_stage
#   from isaacsim.core.prims import SingleArticulation
#   from isaacsim.core.utils.types import ArticulationAction
#   from isaacsim.sensors.physics import IMUSensor
#
# Version deltas:
#   * <=4.1  used the `omni.isaac.*` prefix (e.g. omni.isaac.core,
#     omni.isaac.sensor.IMUSensor, omni.isaac.core.articulations.Articulation).
#     4.5 renamed everything to the `isaacsim.*` prefix.
#   * 6.0 deprecates `isaacsim.sensors.physics` in favour of
#     `isaacsim.sensors.experimental.physics` (IMU/IMUSensor with an
#     IMU.create(...) factory and get_data()/get_sensor_reading() instead of
#     get_current_frame()). See run_isaac() inline notes.
# ---------------------------------------------------------------------------
def run_isaac(args):
    # --- 1. Launch the app FIRST (before any omni/isaacsim.core import) -----
    from isaacsim import SimulationApp  # noqa: E402
    sim_cfg = {"headless": bool(args.headless)}
    simulation_app = SimulationApp(sim_cfg)

    exit_code = 0
    try:
        # --- 2. Now safe to import the core/sensor extensions --------------
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import open_stage
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.physics import IMUSensor
        # 6.0+ alternative:
        #   from isaacsim.sensors.experimental.physics import IMU, IMUSensor

        # --- 3. Open the arm USD stage ------------------------------------
        usd_path = str(Path(args.usd).resolve())
        print(f"[isaac] opening stage: {usd_path}")
        if not open_stage(usd_path):
            raise RuntimeError(f"failed to open USD stage: {usd_path}")

        # --- 4. World drives physics at our trajectory dt (200Hz default) -
        # physics_dt = args.dt makes one world.step() advance exactly one
        # trajectory sample, so we read the IMU once per sampled row.
        world = World(physics_dt=args.dt,
                      rendering_dt=args.dt,
                      stage_units_in_meters=1.0)

        # --- 5. Articulation (the arm) ------------------------------------
        # SingleArticulation wraps a single articulated robot at a prim path.
        # (4.1 and earlier: isaacsim.core.articulations.Articulation /
        #  omni.isaac.core.articulations.Articulation.)
        arm = SingleArticulation(prim_path=args.articulation_root,
                                 name="abel_arm")

        # --- 6. IMU sensor on the forearm / watch site --------------------
        # Created as a new prim; translation/orientation are the offset from
        # the PARENT prim (the forearm link). Zero offset => sits exactly at
        # the watch-mount site prim. filter_size=1 => raw, unfiltered samples
        # (TODO: match whatever smoothing the Apple Watch applies).
        imu = IMUSensor(
            prim_path=args.imu_prim_path,
            name="forearm_imu",
            frequency=int(round(1.0 / args.dt)),  # or dt=args.dt (not both)
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # w,x,y,z identity
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )

        # --- 7. Reset, then initialise handles (physics view exists now) --
        world.reset()
        arm.initialize()
        # IMUSensor needs no explicit initialize(): it activates on PLAY.

        # Map our servo-ordered joint names -> articulation DOF indices once.
        dof_names = list(arm.dof_names)
        try:
            joint_idx = np.array(
                [dof_names.index(j) for j in args.joint_names], dtype=int)
        except ValueError as e:
            raise RuntimeError(
                f"joint name not found in articulation DOFs {dof_names}: {e}\n"
                f"Fix CONFIG['joint_names'] / --joint-names to match the USD.")
        print(f"[isaac] DOF names={dof_names}  using indices={joint_idx.tolist()} "
              f"for {args.joint_names}")

        # --- 8. Load + sample the shared trajectory -----------------------
        t, Q_deg = load_trajectory(args.traj, args.dt)
        Q_rad = servo_deg_to_urdf_rad(Q_deg)  # (N,4) URDF radians

        # Pre-position at the first row and let physics settle one beat so the
        # IMU baseline at t=0 is a true at-rest reading.
        first = ArticulationAction(joint_positions=Q_rad[0],
                                   joint_indices=joint_idx)
        arm.apply_action(first)
        for _ in range(5):
            world.step(render=not args.headless)

        # --- 9. Replay: per step set targets, read IMU --------------------
        accel = np.zeros((len(t), 3))
        gyro = np.zeros((len(t), 3))
        render = not args.headless
        for i in range(len(t)):
            action = ArticulationAction(joint_positions=Q_rad[i],
                                        joint_indices=joint_idx)
            arm.apply_action(action)
            world.step(render=render)

            frame = imu.get_current_frame()  # dict: lin_acc, ang_vel, ...
            # 6.0+ alternative:
            #   d = imu.get_data(); lin_acc=d['linear_acceleration'];
            #   ang_vel=d['angular_velocity']
            accel[i] = np.asarray(frame["lin_acc"], dtype=float)
            gyro[i] = np.asarray(frame["ang_vel"], dtype=float)

        write_csv(args.out, t, accel, gyro)
        rest_mag = float(np.linalg.norm(accel[0]))
        print(f"[isaac] wrote {args.out}  ({len(t)} rows)")
        print(f"[isaac] |accel| at t=0 = {rest_mag:.4f} m/s^2 "
              f"(expect ~{GRAVITY:.3f} at rest)")
    except Exception as e:  # noqa: BLE001 -- ensure the app always closes
        print(f"[isaac] ERROR: {e}", file=sys.stderr)
        exit_code = 1
        raise
    finally:
        simulation_app.close()
    return exit_code


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        description="Replay a joint trajectory and log a simulated forearm IMU "
                    "(Isaac Sim, or --analytic numpy predictor).")
    p.add_argument("--traj", type=Path, default=_DEFAULT_TRAJ,
                   help="trajectory YAML (default: imu_calibration.yaml)")
    p.add_argument("--usd", type=Path, default=_REPO_ROOT / "assets" / "operon_arm.usda",
                   help="arm USD stage (default: assets/operon_arm.usda)")
    p.add_argument("--out", type=Path, default=Path("outputs/sim_imu.csv"),
                   help="output IMU CSV (default: outputs/sim_imu.csv)")
    p.add_argument("--dt", type=float, default=0.005,
                   help="physics/sample timestep s (default 0.005 = 200Hz)")
    p.add_argument("--headless", action="store_true",
                   help="run Isaac headless (no viewport)")
    p.add_argument("--analytic", action="store_true",
                   help="Isaac-free numpy FK IMU predictor (runs on the Mac)")
    # Prim-path overrides (Isaac mode). Defaults come from CONFIG (all TODO).
    p.add_argument("--articulation-root", default=CONFIG["articulation_root"],
                   help="articulation root prim path")
    p.add_argument("--imu-prim-path", default=CONFIG["imu_prim_path"],
                   help="IMU sensor prim path (child of forearm/watch site)")
    p.add_argument("--joint-names", nargs=4, default=CONFIG["joint_names"],
                   metavar=("BASE", "SHOULDER", "ELBOW", "GRIPPER"),
                   help="4 DOF names in servo order")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.analytic:
        return run_analytic(args)
    return run_isaac(args)


if __name__ == "__main__":
    raise SystemExit(main())
