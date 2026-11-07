"""Firmware-matching quintic (5th-order) trajectory.

This is the *shared contract* between the Isaac Sim replay (Shadow PC) and the
real ESP32 arm (Mac). The ESP32 firmware generates this exact trajectory
on-device; we reproduce the identical math here so the simulator drives its
joints through the same motion the real servos execute.

Ported verbatim from Abel-Arm/firmware/Abel_Arm_Firmware.ino:79-100.

Boundary conditions (point-to-point, rest-to-rest):
    q(0)=q0, q(T)=qf, v(0)=v(T)=0, a(0)=a(T)=0

    q(t) = q0 + d*(10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5),   d = qf - q0

Coefficients (firmware form, t in absolute seconds):
    a0 = q0,  a1 = 0,  a2 = 0
    a3 = 10*d / T^3
    a4 = -15*d / T^4
    a5 = 6*d / T^5
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

# Physical limits — must match the firmware constants
# (Abel_Arm_Firmware.ino:32-34). Angles in degrees, time in seconds.
MAX_VELOCITY = 120.0      # deg/s
MAX_ACCELERATION = 200.0  # deg/s^2
MAX_JERK = 600.0          # deg/s^3  (informational; quintic is C2, jerk finite)

# Peak-value factors for this rest-to-rest quintic, derived analytically:
#   v(t)   = 30*d/T * (tau*(1-tau))^2        -> peak 1.875*d/T   at tau=0.5
#   a(t)   = 60*d/T^2 * (tau - 3*tau^2 + ..) -> peak 5.7735*d/T^2 at tau=0.2113
_V_PEAK_FACTOR = 1.875
_A_PEAK_FACTOR = 10.0 / math.sqrt(3.0)  # ~= 5.7735


@dataclass
class Quintic:
    """A single-joint rest-to-rest quintic segment, q0 -> qf over duration T."""

    q0: float
    qf: float
    T: float  # seconds

    @property
    def delta(self) -> float:
        return self.qf - self.q0

    def position(self, t):
        t = np.clip(np.asarray(t, dtype=float), 0.0, self.T)
        tau = t / self.T
        return self.q0 + self.delta * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)

    def velocity(self, t):
        t = np.clip(np.asarray(t, dtype=float), 0.0, self.T)
        tau = t / self.T
        return (self.delta / self.T) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)

    def acceleration(self, t):
        t = np.clip(np.asarray(t, dtype=float), 0.0, self.T)
        tau = t / self.T
        return (self.delta / self.T**2) * (60 * tau - 180 * tau**2 + 120 * tau**3)

    def peak_velocity(self) -> float:
        return _V_PEAK_FACTOR * abs(self.delta) / self.T

    def peak_acceleration(self) -> float:
        return _A_PEAK_FACTOR * abs(self.delta) / self.T**2


def min_duration(delta_deg: float,
                 max_vel: float = MAX_VELOCITY,
                 max_acc: float = MAX_ACCELERATION) -> float:
    """Shortest duration (s) that keeps a move within velocity & accel limits.

    Mirrors the firmware's calculateOptimalDuration intent: the quintic's peak
    velocity and acceleration scale as 1.875*d/T and 5.7735*d/T^2, so we invert
    both limits and take the binding (longer) one.
    """
    d = abs(delta_deg)
    if d == 0.0:
        return 0.0
    t_vel = _V_PEAK_FACTOR * d / max_vel
    t_acc = math.sqrt(_A_PEAK_FACTOR * d / max_acc)
    return max(t_vel, t_acc)


def sample_multi_joint(q_start, q_end, duration_s: float, dt: float):
    """Sample a coordinated N-joint move on a common clock.

    All joints start and finish together over the same ``duration_s`` (the
    firmware drives each servo with its own quintic but the host commands them
    with a shared T, so they are time-synchronised). Returns (t, Q) where Q has
    shape (len(t), N) of joint angles in degrees.
    """
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)
    n_steps = max(1, int(round(duration_s / dt)))
    t = np.linspace(0.0, duration_s, n_steps + 1)
    segs = [Quintic(q0, qf, duration_s) for q0, qf in zip(q_start, q_end)]
    Q = np.stack([seg.position(t) for seg in segs], axis=1)
    return t, Q


if __name__ == "__main__":
    # Quick self-check against the analytic peak factors.
    seg = Quintic(q0=90.0, qf=0.0, T=1.0)
    ts = np.linspace(0, 1, 2001)
    v = seg.velocity(ts)
    a = seg.acceleration(ts)
    print(f"delta={seg.delta:+.1f} deg over T={seg.T}s")
    print(f"  peak |v| numeric={np.max(np.abs(v)):.4f}  analytic={seg.peak_velocity():.4f}")
    print(f"  peak |a| numeric={np.max(np.abs(a)):.4f}  analytic={seg.peak_acceleration():.4f}")
    print(f"  min_duration(90deg)={min_duration(90):.4f}s "
          f"(vel-bound={_V_PEAK_FACTOR*90/MAX_VELOCITY:.4f}, "
          f"acc-bound={math.sqrt(_A_PEAK_FACTOR*90/MAX_ACCELERATION):.4f})")
