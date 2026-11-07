#!/usr/bin/env python3
"""compare.py -- sim-vs-watch IMU comparison for the 4DOF Abel-Arm sim2real loop.

WHAT THIS DOES
==============
The same joint trajectory (config/trajectories/*.yaml, expanded by the firmware
quintic in sim/quintic.py + sim/trajectory.py) is run two ways:

  * Isaac Sim replays it and logs an ``IMUSensor`` at the watch mounting site
    -> ``sim_imu.csv``  (columns: t,ax,ay,az,gx,gy,gz).
  * The REAL ESP32 arm runs it with an Apple Watch strapped to the forearm,
    logging accel + gyro -> ``watch.csv`` (an app export; schema varies).

This script normalises units + frames + time, then reports RMSE / MAE / Pearson
between the two IMU streams and saves overlay plots. That number is the
sim-to-real gap.

------------------------------------------------------------------------------
CONVENTIONS (read before trusting any number)
------------------------------------------------------------------------------
ACCELERATION SIGN / GRAVITY
  Isaac's IMUSensor reports *proper acceleration* (a.k.a. specific force):
  a_proper = a_world - g. At rest it therefore reads ~ +9.81 m/s^2 along the
  site's "up" axis (the reaction/support acceleration), NOT 0. We treat this
  "raw, gravity-included" convention as the reference on BOTH sides.

  Apple CoreMotion gives several accel flavours:
    - accelerometerAccelerationX/Y/Z : RAW accelerometer, in g, INCLUDES gravity
      (this is the watch analogue of Isaac's proper acceleration). PREFERRED.
    - motionUserAccelerationX/Y/Z    : user accel, gravity REMOVED, in g.
    - motionGravityX/Y/Z             : gravity unit vector, in g.
  CoreMotion identity (same frame, same g units):
      raw_accelerometer = userAcceleration + gravity
  So if a watch export only has user + gravity, we reconstruct
      raw = user + gravity   (controllable via --gravity-mode / --gravity-sign).
  This mirrors how this project has historically turned a watch accel/gravity
  vector into a pose in Abel-Arm/abel-voice-cli/src/server.rs:291-350
  (pitch = atan2(y, sqrt(x^2+z^2)), roll = atan2(-x, z) on the gravity vector) --
  i.e. the project already assumes a gravity-included accel vector defines "down".

UNITS
  Target SI: accel m/s^2, gyro rad/s.
    - accel in g  -> * 9.80665     (1 g = 9.80665 m/s^2)
    - gyro in deg/s -> * pi/180
  Auto-detection heuristics (override with --accel-unit / --gyro-unit):
    - accel: median |a| over a still window ~1.0 => g ; ~9.81 => m/s^2.
    - gyro : 95th-pct |w| > ~20 => almost certainly deg/s (this arm tops out at
      ~120 deg/s = ~2.1 rad/s; hundreds of deg/s in rad/s would be absurd).

FRAMES
  The watch axis convention (CoreMotion device frame) is NOT the URDF IMU-site
  frame Isaac logs in. We resolve the constant rotation R (watch -> sim) two ways:
    1. --axis-map "tok,tok,tok": a signed axis permutation, e.g. "-y,x,z" means
       sim_x = -watch_y, sim_y = +watch_x, sim_z = +watch_z. Builds an exact
       3x3 with entries in {-1,0,1}.
    2. AUTO (default, Kabsch/SVD): during a STILL window both sides see the same
       gravity direction; during the single-joint SWEEP both see the same
       dominant rotation axis (from the gyro). Two corresponding unit-vector
       pairs (+ their cross product) fully determine R. The gyro-axis sign
       ambiguity is fixed by correlating the time-aligned projections.
  ASSUMPTION: R is a single rigid rotation, constant for the whole recording
  (the watch does not move on the arm mid-run).

TIME
  Both recordings should start from a sharp TAP on the watch (a big accel spike).
  (a) We detect the first spike above threshold on each stream -> coarse lag.
  (b) We refine sub-sample by maximising the correlation of the gyro-magnitude
      (gravity/rotation-invariant) profiles over a fine lag scan.
  Reported ``lag`` is watch_clock - sim_clock: a watch sample at time t_w
  corresponds to sim time t_w - lag.

------------------------------------------------------------------------------
SUPPORTED WATCH FORMATS
------------------------------------------------------------------------------
  * "sensorlog"    : SensorLog (wide CSV, CamelCase headers such as
                     accelerometerAccelerationX, gyroRotationX,
                     motionUserAccelerationX, motionGravityX,
                     accelerometerTimestamp_sinceReboot / loggingTime).
  * "sensorlogger" : "Sensor Logger" by tszheichoi (long format with a
                     ``sensor`` column + x,y,z + seconds_elapsed/time(ns), OR a
                     single per-sensor file). Accel in m/s^2, gyro in rad/s.
  * --watch-format auto  : sniff the header and pick one.
  Column auto-detection is fuzzy; everything is overridable on the CLI
  (--watch-accel-cols / --watch-gyro-cols / --watch-gravity-cols /
   --watch-user-accel-cols / --watch-time-col).

------------------------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------------------------
  Requires numpy. scipy / matplotlib are OPTIONAL: scipy is used if importable
  (nothing critical depends on it -- pure-numpy fallbacks exist), and plotting
  degrades to a no-op with a warning if matplotlib is missing.

Run the offline self-test (no real data, no Isaac, no hardware):
    python compare.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import tempfile
from dataclasses import dataclass, field

import numpy as np

# ---- optional scipy (everything has a numpy fallback) -----------------------
try:
    import scipy  # noqa: F401
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

G0 = 9.80665  # standard gravity, m/s^2
DEG2RAD = math.pi / 180.0


# =============================================================================
# Data container
# =============================================================================
@dataclass
class IMU:
    """A uniform-or-not IMU record. acc/gyro are (N,3) in SI (m/s^2, rad/s)."""
    t: np.ndarray            # (N,) seconds from 0 (each stream's own clock)
    acc: np.ndarray          # (N,3) proper acceleration, gravity INCLUDED
    gyro: np.ndarray         # (N,3) angular velocity, rad/s
    name: str = "imu"
    meta: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return len(self.t)

    def acc_mag(self) -> np.ndarray:
        return np.linalg.norm(self.acc, axis=1)

    def gyro_mag(self) -> np.ndarray:
        return np.linalg.norm(self.gyro, axis=1)


# =============================================================================
# Low-level CSV reader (stdlib only; no pandas dependency)
# =============================================================================
def read_table(path):
    """Read a CSV into (headers, columns) where columns maps name -> np.ndarray
    of float (NaN where a cell is non-numeric / blank)."""
    with open(path, "r", newline="") as fh:
        reader = csv.reader(fh)
        rows = [r for r in reader if r and any(c.strip() for c in r)]
    if not rows:
        raise ValueError(f"{path}: empty CSV")
    headers = [h.strip() for h in rows[0]]
    ncol = len(headers)
    data = [[] for _ in range(ncol)]
    for r in rows[1:]:
        for i in range(ncol):
            cell = r[i].strip() if i < len(r) else ""
            try:
                data[i].append(float(cell))
            except (ValueError, TypeError):
                data[i].append(math.nan)
    cols = {headers[i]: np.asarray(data[i], dtype=float) for i in range(ncol)}
    # also keep the raw string column for the categorical "sensor" field
    raw_str = {}
    for i in range(ncol):
        raw_str[headers[i]] = [r[i].strip() if i < len(r) else "" for r in rows[1:]]
    return headers, cols, raw_str


def _norm(s: str) -> str:
    return "".join(ch for ch in s.lower() if ch.isalnum())


def find_col(headers, *needles, axis=None, avoid=()):
    """Fuzzy-find a header containing all `needles` (and axis suffix if given),
    not containing any `avoid` token. Returns the header string or None."""
    nax = _norm(axis) if axis else None
    best = None
    for h in headers:
        nh = _norm(h)
        if any(_norm(a) in nh for a in avoid):
            continue
        if not all(_norm(nd) in nh for nd in needles):
            continue
        if nax is not None and not nh.endswith(nax):
            # accept axis appearing as a distinct suffix token
            if not nh.endswith(nax):
                continue
        return h
    return best


def _triplet(headers, base_needles, avoid=()):
    """Find an x/y/z triplet for a sensor; return (cx,cy,cz) headers or None."""
    out = []
    for ax in ("x", "y", "z"):
        c = find_col(headers, *base_needles, axis=ax, avoid=avoid)
        if c is None:
            return None
        out.append(c)
    return tuple(out)


# =============================================================================
# Parsers
# =============================================================================
def parse_sim(path, name="sim", accel_includes_gravity=True):
    """Parse Isaac-logged sim_imu.csv (t,ax,ay,az,gx,gy,gz)."""
    headers, cols, _ = read_table(path)
    tcol = find_col(headers, "t") or headers[0]
    # be forgiving about exact accel/gyro names
    def axis_col(kind, ax):
        for nd in ([kind] if kind != "a" else ["a"]):
            pass
        c = (find_col(headers, kind, axis=ax)
             or find_col(headers, ("accel" if kind == "a" else "gyro"), axis=ax))
        return c
    ax = [find_col(headers, "a", axis=a) or find_col(headers, "accel", axis=a)
          for a in "xyz"]
    gx = [find_col(headers, "g", axis=a) or find_col(headers, "gyro", axis=a)
          for a in "xyz"]
    if any(c is None for c in ax + gx):
        raise ValueError(f"{path}: could not find ax/ay/az/gx/gy/gz columns "
                         f"(headers={headers})")
    t = cols[tcol]
    acc = np.column_stack([cols[c] for c in ax])
    gyro = np.column_stack([cols[c] for c in gx])
    m = np.isfinite(t) & np.all(np.isfinite(acc), axis=1) & np.all(np.isfinite(gyro), axis=1)
    t, acc, gyro = t[m], acc[m], gyro[m]
    t = t - t[0]
    meta = {"accel_includes_gravity": accel_includes_gravity, "source": "sim"}
    if not accel_includes_gravity:
        meta["note"] = "sim accel was declared gravity-free; not adding gravity."
    return IMU(t, acc, gyro, name=name, meta=meta)


def detect_watch_format(headers):
    nh = [_norm(h) for h in headers]
    joined = " ".join(nh)
    if "secondselapsed" in joined or "sensor" in nh:
        return "sensorlogger"
    if "accelerometeraccelerationx" in joined or "gyrorotationx" in joined \
       or "motionuseraccelerationx" in joined:
        return "sensorlog"
    # heuristic fallback
    if any(h in ("x", "y", "z") for h in nh):
        return "sensorlogger"
    return "sensorlog"


def _detect_accel_unit(acc, gyro, still_mask, override):
    if override == "g":
        return G0, "g (forced)"
    if override == "ms2":
        return 1.0, "m/s^2 (forced)"
    mag = np.linalg.norm(acc, axis=1)
    ref = mag[still_mask] if still_mask is not None and still_mask.any() else mag
    med = float(np.nanmedian(ref))
    if med < 3.0:          # ~1.0 -> g
        return G0, f"g  (auto: still |a| median={med:.3f})"
    return 1.0, f"m/s^2  (auto: still |a| median={med:.3f})"


def _detect_gyro_unit(gyro, override):
    if override == "degps":
        return DEG2RAD, "deg/s (forced)"
    if override == "radps":
        return 1.0, "rad/s (forced)"
    mag = np.linalg.norm(gyro, axis=1)
    p95 = float(np.nanpercentile(mag, 95)) if mag.size else 0.0
    if p95 > 20.0:
        return DEG2RAD, f"deg/s  (auto: p95 |w|={p95:.1f})"
    return 1.0, f"rad/s  (auto: p95 |w|={p95:.3f})"


def parse_watch(path, fmt="auto", name="watch", gravity_mode="auto",
                gravity_sign=1.0, accel_unit="auto", gyro_unit="auto",
                overrides=None):
    overrides = overrides or {}
    headers, cols, raw_str = read_table(path)
    if fmt == "auto":
        fmt = detect_watch_format(headers)

    if fmt == "sensorlogger":
        t, acc_raw, gyro_raw, acc_kind = _parse_sensorlogger(
            headers, cols, raw_str, gravity_mode, gravity_sign, overrides)
    else:
        t, acc_raw, gyro_raw, acc_kind = _parse_sensorlog(
            headers, cols, gravity_mode, gravity_sign, overrides)

    # clean
    m = np.isfinite(t) & np.all(np.isfinite(acc_raw), axis=1) & np.all(np.isfinite(gyro_raw), axis=1)
    t, acc_raw, gyro_raw = t[m], acc_raw[m], gyro_raw[m]
    order = np.argsort(t)
    t, acc_raw, gyro_raw = t[order], acc_raw[order], gyro_raw[order]
    t = t - t[0]

    # rough still window (first 25% by time) for unit auto-detect
    still = t < (t[0] + 0.25 * (t[-1] - t[0])) if t.size > 4 else np.ones(t.shape, bool)

    a_scale, a_desc = _detect_accel_unit(acc_raw, gyro_raw, still, accel_unit)
    g_scale, g_desc = _detect_gyro_unit(gyro_raw, gyro_unit)
    acc = acc_raw * a_scale
    gyro = gyro_raw * g_scale

    meta = {
        "format": fmt, "accel_unit": a_desc, "gyro_unit": g_desc,
        "accel_kind": acc_kind, "accel_includes_gravity": True,
        "source": "watch",
    }
    return IMU(t, acc, gyro, name=name, meta=meta)


def _parse_sensorlog(headers, cols, gravity_mode, gravity_sign, overrides):
    """Wide SensorLog CSV."""
    # time
    tcol = overrides.get("time")
    if tcol is None:
        tcol = (find_col(headers, "accelerometertimestamp")
                or find_col(headers, "motiontimestamp")
                or find_col(headers, "loggingtime")
                or find_col(headers, "timestamp")
                or find_col(headers, "time"))
    if tcol is None:
        raise ValueError(f"sensorlog: no time column found in {headers}")
    t = cols[tcol].astype(float)

    # gyro: prefer raw gyroRotation, fall back to motionRotationRate
    gtrip = (_explicit(overrides.get("gyro"), cols)
             or _triplet(headers, ("gyrorotation",))
             or _triplet(headers, ("rotationrate",))
             or _triplet(headers, ("gyro",)))
    if gtrip is None:
        raise ValueError(f"sensorlog: no gyro triplet found in {headers}")
    gyro = _stack(gtrip, cols)

    # accel
    acc, kind = _resolve_watch_accel_sensorlog(
        headers, cols, gravity_mode, gravity_sign, overrides)
    return t, acc, gyro, kind


def _resolve_watch_accel_sensorlog(headers, cols, gravity_mode, gravity_sign, overrides):
    raw_tr = (_explicit(overrides.get("accel"), cols)
              or _triplet(headers, ("accelerometeracceleration",))
              or _triplet(headers, ("accelerometer",), avoid=("timestamp",)))
    user_tr = (_explicit(overrides.get("user_accel"), cols)
               or _triplet(headers, ("motionuseracceleration",))
               or _triplet(headers, ("useracceleration",)))
    grav_tr = (_explicit(overrides.get("gravity"), cols)
               or _triplet(headers, ("motiongravity",))
               or _triplet(headers, ("gravity",)))

    want = gravity_mode
    if want == "auto":
        if raw_tr is not None:
            want = "raw"
        elif user_tr is not None and grav_tr is not None:
            want = "reconstruct"
        else:
            want = "raw"  # last resort

    if want == "raw" and raw_tr is not None:
        return _stack(raw_tr, cols), "raw accelerometer (gravity included)"
    if want == "reconstruct" and user_tr is not None and grav_tr is not None:
        acc = _stack(user_tr, cols) + gravity_sign * _stack(grav_tr, cols)
        return acc, "reconstructed raw = userAccel + gravity"
    # fallbacks
    if raw_tr is not None:
        return _stack(raw_tr, cols), "raw accelerometer (fallback)"
    if user_tr is not None and grav_tr is not None:
        acc = _stack(user_tr, cols) + gravity_sign * _stack(grav_tr, cols)
        return acc, "reconstructed raw = userAccel + gravity (fallback)"
    if user_tr is not None:
        return _stack(user_tr, cols), "WARNING: userAcceleration only (gravity-free!)"
    raise ValueError(f"sensorlog: no usable accel columns in {headers}")


def _parse_sensorlogger(headers, cols, raw_str, gravity_mode, gravity_sign, overrides):
    """Sensor Logger (tszheichoi). Long format with a `sensor` column, or a
    single per-sensor file."""
    sensor_col = None
    for h in headers:
        if _norm(h) == "sensor":
            sensor_col = h
            break

    def time_of(idx=None):
        tcol = overrides.get("time") or find_col(headers, "secondselapsed") \
            or find_col(headers, "time")
        tt = cols[tcol].astype(float)
        # if it looks like ns-since-epoch, convert to seconds
        if np.nanmax(np.abs(tt)) > 1e12:
            tt = tt / 1e9
        return tt, tcol

    if sensor_col is not None:
        sens = [s for s in raw_str[sensor_col]]
        tt, _ = time_of()
        xcol = find_col(headers, "x") or "x"
        ycol = find_col(headers, "y") or "y"
        zcol = find_col(headers, "z") or "z"
        X, Y, Z = cols[xcol], cols[ycol], cols[zcol]

        def grab(*names):
            want = {_norm(n) for n in names}
            idx = np.array([i for i, s in enumerate(sens) if _norm(s) in want], dtype=int)
            return idx

        acc_idx = grab("Accelerometer", "RawAccelerometer")
        lin_idx = grab("LinearAccelerometer", "LinearAcceleration", "UserAcceleration")
        grav_idx = grab("Gravity")
        gyro_idx = grab("Gyroscope", "Gyro", "RotationRate")
        if gyro_idx.size == 0:
            raise ValueError("sensorlogger: no Gyroscope rows found")

        gyro_t = tt[gyro_idx]
        gyro = np.column_stack([X[gyro_idx], Y[gyro_idx], Z[gyro_idx]])

        # accel: prefer the gravity-included Accelerometer sensor
        want = gravity_mode
        if want == "auto":
            want = "raw" if acc_idx.size else (
                "reconstruct" if (lin_idx.size and grav_idx.size) else "raw")
        if want == "raw" and acc_idx.size:
            base_t = tt[acc_idx]
            acc = np.column_stack([X[acc_idx], Y[acc_idx], Z[acc_idx]])
            kind = "Accelerometer (gravity included)"
        elif lin_idx.size and grav_idx.size:
            base_t = tt[lin_idx]
            lin = np.column_stack([X[lin_idx], Y[lin_idx], Z[lin_idx]])
            gv = np.column_stack([
                np.interp(base_t, tt[grav_idx], X[grav_idx]),
                np.interp(base_t, tt[grav_idx], Y[grav_idx]),
                np.interp(base_t, tt[grav_idx], Z[grav_idx])])
            acc = lin + gravity_sign * gv
            kind = "reconstructed raw = Linear + Gravity"
        elif acc_idx.size:
            base_t = tt[acc_idx]
            acc = np.column_stack([X[acc_idx], Y[acc_idx], Z[acc_idx]])
            kind = "Accelerometer (fallback)"
        else:
            raise ValueError("sensorlogger: no usable accel rows")

        # unify gyro onto the accel timeline
        gyro_on_acc = np.column_stack([
            np.interp(base_t, gyro_t, gyro[:, 0]),
            np.interp(base_t, gyro_t, gyro[:, 1]),
            np.interp(base_t, gyro_t, gyro[:, 2])])
        return base_t, acc, gyro_on_acc, kind

    # single-file (already one sensor) -> need x/y/z + time; treat as ambiguous:
    tt, _ = time_of()
    xyz = _triplet(headers, ()) or (find_col(headers, "x"), find_col(headers, "y"),
                                    find_col(headers, "z"))
    if any(c is None for c in xyz):
        raise ValueError(f"sensorlogger single-file: need x/y/z columns ({headers})")
    vec = _stack(xyz, cols)
    # without a sensor label we cannot know accel vs gyro; require overrides
    raise ValueError("sensorlogger single-file without a `sensor` column is "
                     "ambiguous; pass the combined export or use sensorlog/overrides.")


def _explicit(spec, cols):
    if not spec:
        return None
    names = [s.strip() for s in spec.split(",")]
    if len(names) != 3 or any(n not in cols for n in names):
        return None
    return tuple(names)


def _stack(triplet, cols):
    return np.column_stack([cols[c] for c in triplet]).astype(float)


# =============================================================================
# Rotation helpers (numpy-only; scipy not required)
# =============================================================================
def rodrigues(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-15)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def rotation_geodesic_deg(R1, R2):
    """Angle of R1 R2^T, in degrees -- 0 means identical rotations."""
    R = R1 @ R2.T
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def kabsch(P, Q, w=None):
    """Best rotation R s.t. R @ p_i ~ q_i. P,Q: (n,3) direction vectors."""
    P = np.asarray(P, float)
    Q = np.asarray(Q, float)
    if w is None:
        w = np.ones(len(P))
    w = np.asarray(w, float)[:, None]
    H = (P * w).T @ Q                      # 3x3
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    return R


def axis_map_to_R(spec):
    """'-y,x,z' -> 3x3 mapping watch->sim (sim_axis_i = sign*watch_axis_j)."""
    toks = [s.strip().lower() for s in spec.split(",")]
    if len(toks) != 3:
        raise ValueError(f"--axis-map needs 3 comma tokens, got {spec!r}")
    idx = {"x": 0, "y": 1, "z": 2}
    R = np.zeros((3, 3))
    for i, tok in enumerate(toks):
        sign = 1.0
        if tok[0] in "+-":
            sign = -1.0 if tok[0] == "-" else 1.0
            tok = tok[1:]
        if tok not in idx:
            raise ValueError(f"--axis-map bad token {toks[i]!r}")
        R[i, idx[tok]] = sign
    return R


def apply_R(imu, R):
    imu.acc = (R @ imu.acc.T).T
    imu.gyro = (R @ imu.gyro.T).T
    return imu


# =============================================================================
# Still / motion window detection
# =============================================================================
def still_window(imu, gyro_thresh=0.15, min_dur=0.25):
    """Earliest contiguous span where |gyro| < thresh for >= min_dur. Returns
    boolean mask. Falls back to the quietest fixed fraction."""
    gm = imu.gyro_mag()
    quiet = gm < gyro_thresh
    t = imu.t
    best = None
    i = 0
    n = len(t)
    while i < n:
        if quiet[i]:
            j = i
            while j + 1 < n and quiet[j + 1]:
                j += 1
            if t[j] - t[i] >= min_dur:
                best = (i, j)
                break
            i = j + 1
        else:
            i += 1
    if best is None:
        # quietest 20% by gyro magnitude near the start
        k = max(3, int(0.2 * n))
        mask = np.zeros(n, bool)
        mask[:k] = True
        return mask
    mask = np.zeros(n, bool)
    mask[best[0]:best[1] + 1] = True
    return mask


def motion_window(imu, gyro_thresh=0.3):
    gm = imu.gyro_mag()
    mask = gm > gyro_thresh
    if not mask.any():
        # top 30% by gyro magnitude
        thr = np.nanpercentile(gm, 70)
        mask = gm >= thr
    return mask


# =============================================================================
# Time alignment
# =============================================================================
def detect_tap(imu, thresh=6.0):
    """Time of first accel-magnitude deviation from the rest baseline > thresh
    (m/s^2). Returns float seconds or None."""
    mag = imu.acc_mag()
    n = len(mag)
    base = np.median(mag[:max(3, n // 10)])
    dev = np.abs(mag - base)
    idx = np.argmax(dev > thresh)
    if dev[idx] > thresh:
        return float(imu.t[idx])
    return None


def _signal_on_grid(imu, grid):
    """Sample a rotation/gravity-invariant alignment signal on `grid` (in the
    imu's own clock). Uses gyro magnitude; falls back to accel-magnitude
    deviation if the gyro is essentially flat."""
    gm = imu.gyro_mag()
    if np.nanmax(gm) - np.nanmin(gm) < 1e-3:
        am = imu.acc_mag()
        gm = np.abs(am - np.median(am))
    return np.interp(grid, imu.t, gm, left=0.0, right=0.0)


def estimate_lag(sim, watch, tap_thresh=6.0, dt=None):
    """Return (lag, info). lag = watch_clock - sim_clock; a watch sample at t_w
    matches sim time t_w - lag."""
    if dt is None:
        dt = float(np.median(np.diff(sim.t)))
        if not np.isfinite(dt) or dt <= 0:
            dt = 0.005

    tap_s = detect_tap(sim, tap_thresh)
    tap_w = detect_tap(watch, tap_thresh)
    coarse = (tap_w - tap_s) if (tap_s is not None and tap_w is not None) else 0.0

    # build sim signal on a uniform grid
    g0, g1 = sim.t[0], sim.t[-1]
    grid = np.arange(g0, g1, dt)
    sig_s = _signal_on_grid(sim, grid)
    sig_s = sig_s - sig_s.mean()

    def corr(lag):
        # sample the watch signal at watch-time = (sim grid) + lag
        wt = grid + lag
        valid = (wt >= watch.t[0]) & (wt <= watch.t[-1])
        if valid.sum() < max(8, 0.2 * len(grid)):
            return -np.inf
        sw = np.interp(wt[valid], watch.t, watch.gyro_mag())
        ss = sig_s[valid]
        sw = sw - sw.mean()
        denom = (np.linalg.norm(ss) * np.linalg.norm(sw))
        if denom < 1e-12:
            return -np.inf
        return float(ss @ sw / denom)

    # coarse scan around the tap estimate, then fine refine
    coarse_best, best_c = coarse, -np.inf
    for lag in np.arange(coarse - 0.6, coarse + 0.6 + 1e-9, dt):
        c = corr(lag)
        if c > best_c:
            best_c, coarse_best = c, lag
    fine_best, best_f = coarse_best, best_c
    for lag in np.arange(coarse_best - 2 * dt, coarse_best + 2 * dt + 1e-9, dt / 16.0):
        c = corr(lag)
        if c > best_f:
            best_f, fine_best = c, lag

    info = {"tap_sim": tap_s, "tap_watch": tap_w, "coarse_lag": coarse,
            "lag": fine_best, "corr": best_f, "dt": dt}
    return fine_best, info


# =============================================================================
# Frame estimation (auto)
# =============================================================================
def estimate_frame(sim, watch, lag, gyro_still=0.15, gyro_motion=0.3):
    """Estimate R (watch->sim) from gravity (still) + dominant gyro axis (sweep).
    Returns (R, info)."""
    # --- gravity pair from each side's still window ---
    sm = still_window(sim, gyro_still)
    wm = still_window(watch, gyro_still)
    g_sim = np.nanmean(sim.acc[sm], axis=0)
    g_w = np.nanmean(watch.acc[wm], axis=0)
    g_sim_u = g_sim / (np.linalg.norm(g_sim) + 1e-15)
    g_w_u = g_w / (np.linalg.norm(g_w) + 1e-15)

    # --- gyro-axis pair from the (time-aligned) motion window ---
    mm = motion_window(sim, gyro_motion)
    t_mot = sim.t[mm]
    G_sim = sim.gyro[mm]
    # watch gyro sampled at the same physical instants (watch time = sim + lag)
    wt = t_mot + lag
    G_w = np.column_stack([
        np.interp(wt, watch.t, watch.gyro[:, 0]),
        np.interp(wt, watch.t, watch.gyro[:, 1]),
        np.interp(wt, watch.t, watch.gyro[:, 2])])

    # Robust full-rotation solve (Wahba): R @ watch_v ~ sim_v holds for EVERY
    # gravity, accel, and gyro sample. As the arm rotates, the gravity reaction
    # sweeps across the (rotating) sensor frame, so accel-during-motion is a rich,
    # clean direction set; gyro adds the rest. Stacking all time-aligned pairs and
    # solving once stays well-conditioned for multi-axis trajectories (base about
    # Z, shoulder/elbow about Y), where no single rotation axis dominates.
    A_sim = sim.acc[mm]
    A_w = np.column_stack([
        np.interp(wt, watch.t, watch.acc[:, k]) for k in range(3)])
    accel_energy = float(np.sum(A_sim ** 2)) + 1e-9
    gyro_energy = float(np.sum(G_sim ** 2)) + 1e-9
    gscale = math.sqrt(accel_energy / gyro_energy)   # balance gyro vs accel energy
    ganchor = math.sqrt(accel_energy)                 # gravity anchor weight
    P = np.vstack([g_w_u[None, :] * ganchor, A_w, G_w * gscale])      # watch
    Q = np.vstack([g_sim_u[None, :] * ganchor, A_sim, G_sim * gscale])  # sim
    R = kabsch(P, Q)

    info = {
        "gravity_sim": g_sim_u, "gravity_watch": g_w_u,
        "n_gyro_pairs": int(len(G_sim)),
        "still_n_sim": int(sm.sum()), "still_n_watch": int(wm.sum()),
        "motion_n": int(mm.sum()),
    }
    return R, info


# =============================================================================
# Resample + metrics
# =============================================================================
# Half-width (s) of the window around the sync tap excluded from tracking
# metrics. The tap is only a time fiducial; its razor-sharp impulse would
# otherwise dominate RMSE without reflecting trajectory-tracking quality.
TAP_EXCLUDE_S = 0.12


def resample_common(sim, watch, lag, dt=None):
    """Interpolate both streams onto a common uniform grid (sim clock) over the
    overlap. Returns (grid, sim_acc, sim_gyro, watch_acc, watch_gyro)."""
    if dt is None:
        dt = float(np.median(np.diff(sim.t)))
        if not np.isfinite(dt) or dt <= 0:
            dt = 0.005
    lo = max(sim.t[0], watch.t[0] - lag)
    hi = min(sim.t[-1], watch.t[-1] - lag)
    if hi <= lo:
        raise ValueError("no temporal overlap between sim and watch after alignment")
    grid = np.arange(lo, hi, dt)

    def interp3(t, V):
        return np.column_stack([np.interp(grid, t, V[:, i]) for i in range(3)])

    sa = interp3(sim.t, sim.acc)
    sg = interp3(sim.t, sim.gyro)
    wt = grid + lag
    wa = np.column_stack([np.interp(wt, watch.t, watch.acc[:, i]) for i in range(3)])
    wg = np.column_stack([np.interp(wt, watch.t, watch.gyro[:, i]) for i in range(3)])
    return grid, sa, sg, wa, wg


def _metrics(a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    err = a - b
    rmse = float(np.sqrt(np.mean(err ** 2)))
    mae = float(np.mean(np.abs(err)))
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        r = float("nan")
    else:
        r = float(np.corrcoef(a, b)[0, 1])
    return rmse, mae, r


def compute_metrics(grid, sa, sg, wa, wg):
    rows = []
    labels_acc = ["acc_x", "acc_y", "acc_z"]
    labels_gyro = ["gyro_x", "gyro_y", "gyro_z"]
    for i, lab in enumerate(labels_acc):
        rows.append((lab, "m/s^2") + _metrics(sa[:, i], wa[:, i]))
    rows.append(("acc_mag", "m/s^2") + _metrics(
        np.linalg.norm(sa, axis=1), np.linalg.norm(wa, axis=1)))
    for i, lab in enumerate(labels_gyro):
        rows.append((lab, "rad/s") + _metrics(sg[:, i], wg[:, i]))
    rows.append(("gyro_mag", "rad/s") + _metrics(
        np.linalg.norm(sg, axis=1), np.linalg.norm(wg, axis=1)))
    return rows


def print_summary(rows, lag_info, frame_R, frame_info, sim, watch):
    print("\n" + "=" * 64)
    print("SIM vs WATCH  IMU comparison")
    print("=" * 64)
    print(f" sim   : n={sim.n}  dt~{np.median(np.diff(sim.t))*1e3:.2f} ms  "
          f"span={sim.t[-1]-sim.t[0]:.2f}s")
    print(f" watch : n={watch.n}  span={watch.t[-1]-watch.t[0]:.2f}s  "
          f"[{watch.meta.get('format')}]")
    print(f"         accel: {watch.meta.get('accel_kind')}  "
          f"unit={watch.meta.get('accel_unit')}")
    print(f"         gyro : unit={watch.meta.get('gyro_unit')}")
    print("-" * 64)
    print(f" TIME LAG (watch - sim) : {lag_info['lag']*1e3:+.1f} ms   "
          f"(corr={lag_info['corr']:.3f})")
    if lag_info.get("tap_sim") is not None:
        print(f"   taps  sim={lag_info['tap_sim']:.3f}s  "
              f"watch={lag_info['tap_watch']:.3f}s")
    print("-" * 64)
    if frame_info is not None:
        print(" FRAME R (watch -> sim), auto-estimated:")
    else:
        print(" FRAME R (watch -> sim):")
    for r in frame_R:
        print("   [{:+.3f} {:+.3f} {:+.3f}]".format(*r))
    print("-" * 64)
    hdr = f"{'channel':9s} {'unit':6s} {'RMSE':>10s} {'MAE':>10s} {'Pearson':>9s}"
    print(hdr)
    print("-" * 64)
    for name, unit, rmse, mae, r in rows:
        print(f"{name:9s} {unit:6s} {rmse:10.4f} {mae:10.4f} {r:9.3f}")
    print("=" * 64 + "\n")


# =============================================================================
# Plots (optional)
# =============================================================================
# Vivid palette so sim vs watch pop; spectrograms use a rainbow "spectrum" map.
_C_SIM = "#00B4D8"      # cyan
_C_WATCH = "#FF5D5D"    # coral
_C_RESID = "#B5179E"    # magenta
_SPEC_CMAP = "turbo"    # perceptual rainbow colormap


def _psd(x, fs):
    """Power spectral density: Welch via scipy if present, else FFT periodogram."""
    x = np.asarray(x, float)
    x = x - np.nanmean(x)
    n = len(x)
    if n < 8:
        return np.array([0.0]), np.array([0.0])
    try:
        from scipy.signal import welch
        f, p = welch(x, fs=fs, nperseg=min(n, 1024))
        return f, p
    except Exception:
        win = np.hanning(n)
        X = np.fft.rfft(x * win)
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        p = (np.abs(X) ** 2) / (fs * np.sum(win ** 2))
        return f, p


def make_plots(grid, sa, sg, wa, wg, outdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[plots] matplotlib unavailable ({e}); skipping plots.")
        return []
    try:
        plt.style.use("seaborn-v0_8-darkgrid")
    except Exception:
        pass
    os.makedirs(outdir, exist_ok=True)
    saved = []
    dt = float(np.median(np.diff(grid))) if len(grid) > 1 else 0.005
    fs = 1.0 / dt if dt > 0 else 200.0

    sm_a, wm_a = np.linalg.norm(sa, axis=1), np.linalg.norm(wa, axis=1)
    sm_g, wm_g = np.linalg.norm(sg, axis=1), np.linalg.norm(wg, axis=1)

    # ---- time-domain overlays: colored sim vs watch + residual band --------
    def overlay(fname, title, S, W, labels, unit):
        fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
        for i in range(3):
            axes[i].plot(grid, S[:, i], color=_C_SIM, lw=1.4, label="sim")
            axes[i].plot(grid, W[:, i], color=_C_WATCH, lw=1.1, alpha=0.85,
                         label="watch")
            axes[i].fill_between(grid, S[:, i], W[:, i], color=_C_RESID,
                                 alpha=0.18, lw=0, label="residual")
            axes[i].set_ylabel(f"{labels[i]} ({unit})")
        axes[0].legend(loc="upper right", ncol=3, fontsize=8)
        axes[3].plot(grid, np.linalg.norm(S, axis=1), color=_C_SIM, lw=1.4,
                     label="sim |·|")
        axes[3].plot(grid, np.linalg.norm(W, axis=1), color=_C_WATCH, lw=1.1,
                     alpha=0.85, label="watch |·|")
        axes[3].set_ylabel(f"magnitude ({unit})")
        axes[3].set_xlabel("time (s, sim clock)")
        axes[3].legend(loc="upper right", fontsize=8)
        fig.suptitle(title, fontsize=13, fontweight="bold")
        fig.tight_layout()
        p = os.path.join(outdir, fname)
        fig.savefig(p, dpi=120)
        plt.close(fig)
        saved.append(p)

    overlay("accel_overlay.png", "Acceleration: sim vs watch (m/s²)",
            sa, wa, ["ax", "ay", "az"], "m/s²")
    overlay("gyro_overlay.png", "Angular velocity: sim vs watch (rad/s)",
            sg, wg, ["gx", "gy", "gz"], "rad/s")

    # ---- power spectra: sim vs watch (log-y) -------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for ax, (label, s_sig, w_sig, unit) in zip(axes, [
            ("accel |·|", sm_a, wm_a, "m/s²"),
            ("gyro |·|", sm_g, wm_g, "rad/s")]):
        f1, p1 = _psd(s_sig, fs)
        f2, p2 = _psd(w_sig, fs)
        ax.semilogy(f1, p1 + 1e-18, color=_C_SIM, lw=1.7, label="sim")
        ax.semilogy(f2, p2 + 1e-18, color=_C_WATCH, lw=1.3, alpha=0.85,
                    label="watch")
        ax.set_title(f"PSD — {label}")
        ax.set_xlabel("frequency (Hz)")
        ax.set_ylabel(f"PSD ({unit}²/Hz)")
        ax.set_xlim(0, min(fs / 2, 50))
        ax.legend()
    fig.suptitle("Power spectra: sim vs watch", fontsize=13, fontweight="bold")
    fig.tight_layout()
    p = os.path.join(outdir, "psd.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    saved.append(p)

    # ---- spectrograms: time-frequency, rainbow colormap --------------------
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    nfft = 256 if len(grid) > 300 else max(32, len(grid) // 8)

    def spectro(ax, sig, title):
        _, _, _, im = ax.specgram(sig - np.mean(sig), NFFT=nfft, Fs=fs,
                                  noverlap=int(nfft * 0.8), cmap=_SPEC_CMAP,
                                  mode="magnitude")
        ax.set_title(title)
        ax.set_ylim(0, min(fs / 2, 40))
        ax.set_xlabel("time (s)")
        ax.set_ylabel("freq (Hz)")
        return im

    im_a = spectro(axes[0, 0], sm_a, "sim — accel |·|")
    spectro(axes[0, 1], wm_a, "watch — accel |·|")
    im_g = spectro(axes[1, 0], sm_g, "sim — gyro |·|")
    spectro(axes[1, 1], wm_g, "watch — gyro |·|")
    fig.colorbar(im_a, ax=axes[0, :], shrink=0.85, label="magnitude")
    fig.colorbar(im_g, ax=axes[1, :], shrink=0.85, label="magnitude")
    fig.suptitle("Spectrograms: sim vs watch", fontsize=13, fontweight="bold")
    p = os.path.join(outdir, "spectrogram.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    saved.append(p)

    for p in saved:
        print(f"[plots] wrote {p}")
    return saved


# =============================================================================
# Driver
# =============================================================================
def run_pipeline(args):
    sim = parse_sim(args.sim, accel_includes_gravity=not args.sim_accel_gravity_free)
    watch = parse_watch(
        args.watch, fmt=args.watch_format, gravity_mode=args.gravity_mode,
        gravity_sign=args.gravity_sign, accel_unit=args.accel_unit,
        gyro_unit=args.gyro_unit,
        overrides={
            "accel": args.watch_accel_cols, "gyro": args.watch_gyro_cols,
            "gravity": args.watch_gravity_cols,
            "user_accel": args.watch_user_accel_cols,
            "time": args.watch_time_col,
        })

    lag, lag_info = estimate_lag(sim, watch, tap_thresh=args.tap_threshold)

    frame_info = None
    if args.axis_map:
        R = axis_map_to_R(args.axis_map)
    elif args.no_auto_frame:
        R = np.eye(3)
    else:
        R, frame_info = estimate_frame(sim, watch, lag,
                                       gyro_still=args.still_gyro_thresh,
                                       gyro_motion=args.motion_gyro_thresh)
    apply_R(watch, R)

    grid, sa, sg, wa, wg = resample_common(sim, watch, lag)
    # Drop the sync-tap window from tracking metrics (see TAP_EXCLUDE_S).
    mmask = np.ones(len(grid), bool)
    tap_sim = lag_info.get("tap_sim")
    if tap_sim is not None:
        mmask &= np.abs(grid - tap_sim) > TAP_EXCLUDE_S
    if mmask.sum() < 8:
        mmask = np.ones(len(grid), bool)
    rows = compute_metrics(grid[mmask], sa[mmask], sg[mmask], wa[mmask], wg[mmask])
    print_summary(rows, lag_info, R, frame_info, sim, watch)

    if not args.no_plots:
        make_plots(grid, sa, sg, wa, wg, args.outdir)

    return {"lag": lag, "lag_info": lag_info, "R": R, "frame_info": frame_info,
            "rows": rows, "grid": grid}


# =============================================================================
# Self-test (no real data)
# =============================================================================
def _synth_sim(dt=0.005, dur=6.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, dur, dt)
    n = len(t)
    acc = np.zeros((n, 3))
    gyro = np.zeros((n, 3))
    acc[:, 2] = G0  # proper accel: gravity reaction "up" at rest

    # tap impulse at ~0.5 s (sharp accel spike, a few samples wide)
    ti = int(0.5 / dt)
    acc[ti:ti + 3, 0] += 45.0

    # single-joint sweep 1.0 .. 4.0 s: dominant rotation axis, bell profile
    axis = np.array([0.2, 0.9, 0.35])
    axis = axis / np.linalg.norm(axis)
    t0, t1 = 1.0, 4.0
    w = np.zeros(n)
    m = (t >= t0) & (t <= t1)
    tau = (t[m] - t0) / (t1 - t0)
    # quintic-velocity-like bell (rest-to-rest), one signed direction
    w[m] = 2.0 * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    gyro += np.outer(w, axis)
    # a little tangential accel during the move so accel also carries signal
    acc[m, 0] += 1.5 * np.sin(2 * math.pi * 0.8 * (t[m] - t0))
    acc[m, 1] += 1.0 * np.cos(2 * math.pi * 0.5 * (t[m] - t0))

    acc += rng.normal(0, 0.01, acc.shape)
    gyro += rng.normal(0, 0.002, gyro.shape)
    return IMU(t, acc, gyro, name="sim")


def _write_sim_csv(imu, path):
    with open(path, "w", newline="") as fh:
        wcsv = csv.writer(fh)
        wcsv.writerow(["t", "ax", "ay", "az", "gx", "gy", "gz"])
        for i in range(imu.n):
            wcsv.writerow([f"{imu.t[i]:.6f}",
                           *[f"{v:.6f}" for v in imu.acc[i]],
                           *[f"{v:.6f}" for v in imu.gyro[i]]])


def _write_watch_sensorlog_csv(t, acc_g, gyro_degps, path):
    """Emit a SensorLog-style wide CSV: raw accelerometer (g, gravity incl) +
    gyroRotation (deg/s)."""
    with open(path, "w", newline="") as fh:
        wcsv = csv.writer(fh)
        wcsv.writerow([
            "loggingTime", "accelerometerTimestamp_sinceReboot",
            "accelerometerAccelerationX", "accelerometerAccelerationY",
            "accelerometerAccelerationZ",
            "gyroRotationX", "gyroRotationY", "gyroRotationZ"])
        for i in range(len(t)):
            wcsv.writerow([f"{t[i]:.6f}", f"{t[i]:.6f}",
                           *[f"{v:.6f}" for v in acc_g[i]],
                           *[f"{v:.6f}" for v in gyro_degps[i]]])


def selftest():
    print("[selftest] generating synthetic sim IMU ...")
    sim = _synth_sim()

    # ---- known ground-truth transform applied to make the fake watch ----
    R_true = rodrigues([0.3, -0.7, 0.6], math.radians(37.0))  # watch->sim
    tau_true = 0.370      # recoverable lag = the watch's extra lead-in (s)
    rng = np.random.default_rng(7)

    # The real watch is already recording (held still) BEFORE the arm trajectory
    # begins, so it carries extra lead-in. The parser re-bases each clock to its
    # own first sample (see parse_watch: t -= t[0]), which erases any constant
    # absolute offset — so the *recoverable* lag is exactly this lead-in, marked
    # by the shared sync tap. Model it by prepending a still block to the watch.
    Rt = R_true.T                          # watch = R_true^T @ sim
    dt = float(np.median(np.diff(sim.t)))
    lead_n = int(round(tau_true / dt))
    g_watch = Rt @ np.array([0.0, 0.0, G0])   # gravity reaction, watch frame, at rest
    lead_acc = np.tile(g_watch, (lead_n, 1))
    lead_gyro = np.zeros((lead_n, 3))
    w_acc_si = np.vstack([lead_acc, (Rt @ sim.acc.T).T])
    w_gyro_si = np.vstack([lead_gyro, (Rt @ sim.gyro.T).T])
    # add measurement noise (in SI), then change units to g + deg/s
    w_acc_si = w_acc_si + rng.normal(0, 0.03, w_acc_si.shape)
    w_gyro_si = w_gyro_si + rng.normal(0, 0.01, w_gyro_si.shape)
    w_acc_g = w_acc_si / G0
    w_gyro_degps = w_gyro_si / DEG2RAD
    w_t = np.arange(w_acc_si.shape[0]) * dt   # watch's own continuous clock

    tmp = tempfile.mkdtemp(prefix="abel_selftest_")
    sim_path = os.path.join(tmp, "sim_imu.csv")
    watch_path = os.path.join(tmp, "watch.csv")
    _write_sim_csv(sim, sim_path)
    _write_watch_sensorlog_csv(w_t, w_acc_g, w_gyro_degps, watch_path)
    print(f"[selftest] sim   -> {sim_path}")
    print(f"[selftest] watch -> {watch_path}")

    class A:  # arg shim
        pass
    a = A()
    a.sim = sim_path
    a.watch = watch_path
    a.watch_format = "auto"
    a.gravity_mode = "auto"
    a.gravity_sign = 1.0
    a.accel_unit = "auto"
    a.gyro_unit = "auto"
    a.sim_accel_gravity_free = False
    a.axis_map = None
    a.no_auto_frame = False
    a.still_gyro_thresh = 0.15
    a.motion_gyro_thresh = 0.3
    a.tap_threshold = 6.0
    a.watch_accel_cols = a.watch_gyro_cols = a.watch_gravity_cols = None
    a.watch_user_accel_cols = a.watch_time_col = None
    a.outdir = os.path.join(tmp, "outputs")
    a.no_plots = not _can_plot()

    out = run_pipeline(a)

    # ---- assertions ----
    lag_err = abs(out["lag"] - tau_true)
    rot_err = rotation_geodesic_deg(out["R"], R_true)
    print(f"[selftest] recovered lag = {out['lag']*1e3:.1f} ms "
          f"(true {tau_true*1e3:.1f} ms, err {lag_err*1e3:.1f} ms)")
    print(f"[selftest] recovered frame error = {rot_err:.2f} deg")

    # accel/gyro RMSE should be small (units & frame correctly recovered)
    rmse = {r[0]: r[2] for r in out["rows"]}
    ok = True
    msgs = []
    if lag_err > 0.01:
        ok = False; msgs.append(f"lag error {lag_err*1e3:.1f} ms > 10 ms")
    if rot_err > 5.0:
        ok = False; msgs.append(f"frame error {rot_err:.2f} deg > 5 deg")
    if rmse["gyro_mag"] > 0.05:
        ok = False; msgs.append(f"gyro_mag RMSE {rmse['gyro_mag']:.3f} > 0.05")
    if rmse["acc_mag"] > 0.3:
        ok = False; msgs.append(f"acc_mag RMSE {rmse['acc_mag']:.3f} > 0.3")

    if ok:
        print("\n[selftest] PASS  -- recovered rotation, lag, units within tolerance.")
        return 0
    print("\n[selftest] FAIL:")
    for m in msgs:
        print("   - " + m)
    return 1


def _can_plot():
    try:
        import matplotlib  # noqa: F401
        return True
    except Exception:
        return False


# =============================================================================
# CLI
# =============================================================================
def build_parser():
    p = argparse.ArgumentParser(
        description="Compare Isaac-sim IMU vs Apple-Watch IMU for the Abel-Arm "
                    "sim2real loop.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--sim", help="sim_imu.csv from Isaac (t,ax,ay,az,gx,gy,gz)")
    p.add_argument("--watch", help="watch CSV export (SensorLog / Sensor Logger)")
    p.add_argument("--watch-format", choices=["auto", "sensorlog", "sensorlogger"],
                   default="auto")
    p.add_argument("--accel-unit", choices=["auto", "g", "ms2"], default="auto")
    p.add_argument("--gyro-unit", choices=["auto", "degps", "radps"], default="auto")
    p.add_argument("--gravity-mode", choices=["auto", "raw", "reconstruct"],
                   default="auto",
                   help="raw=use accelerometer incl gravity; "
                        "reconstruct=userAccel+gravity")
    p.add_argument("--gravity-sign", type=float, default=1.0,
                   help="sign on gravity when reconstructing raw = user + s*gravity")
    p.add_argument("--sim-accel-gravity-free", action="store_true",
                   help="declare the sim accel as gravity-removed (default: not)")
    p.add_argument("--axis-map", default=None,
                   help="explicit watch->sim signed axis perm, e.g. '-y,x,z'. "
                        "If omitted, the rotation is auto-estimated.")
    p.add_argument("--no-auto-frame", action="store_true",
                   help="skip auto frame estimation; use identity (with no axis-map)")
    p.add_argument("--still-gyro-thresh", type=float, default=0.15)
    p.add_argument("--motion-gyro-thresh", type=float, default=0.3)
    p.add_argument("--tap-threshold", type=float, default=6.0,
                   help="accel-magnitude deviation (m/s^2) counted as the start tap")
    # column overrides
    p.add_argument("--watch-accel-cols", default=None, help="ax,ay,az header names")
    p.add_argument("--watch-gyro-cols", default=None, help="gx,gy,gz header names")
    p.add_argument("--watch-gravity-cols", default=None)
    p.add_argument("--watch-user-accel-cols", default=None)
    p.add_argument("--watch-time-col", default=None)
    p.add_argument("--outdir",
                   default="/Users/dr.gretchenboria/ROS/abel_sim2real/outputs")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--selftest", action="store_true",
                   help="run the offline synthetic self-test and exit")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.sim or not args.watch:
        print("error: --sim and --watch are required (or use --selftest)",
              file=sys.stderr)
        return 2
    run_pipeline(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
