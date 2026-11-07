"""Shared trajectory loader + sampler.

Parses a config/trajectories/*.yaml waypoint file and expands it into a single
time-synchronised joint-angle signal using the firmware quintic. Used by BOTH
sides of the loop:
  * Isaac replay (Shadow) -> drives sim joints, logs the IMU.
  * ROS goal-builder (Mac) -> turns waypoints into a joint_trajectory goal.

Each waypoint move duration is clamped UP to quintic.min_duration() exactly as
the ESP32 firmware does (calculateOptimalDuration), so the simulator and the
real arm execute identical motion even when a YAML asks for a too-short move.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from quintic import Quintic, min_duration


@dataclass
class Trajectory:
    name: str
    joint_order: list
    home: list
    # Expanded plan: per segment -> (q_start, q_end, move_T_s, settle_s)
    segments: list = field(default_factory=list)

    @property
    def n_joints(self) -> int:
        return len(self.joint_order)

    @property
    def total_duration(self) -> float:
        return sum(T + s for _, _, T, s in self.segments)


def load(path) -> Trajectory:
    data = yaml.safe_load(Path(path).read_text())
    joint_order = data["joint_order"]
    home = [float(x) for x in data["home"]]

    segments = []
    q_prev = list(home)
    for wp in data["waypoints"]:
        q_target = [float(x) for x in wp["targets"]]
        commanded = float(wp.get("duration_ms", 0)) / 1000.0
        settle = float(wp.get("settle_ms", 0)) / 1000.0
        # Clamp move duration up to the limit-respecting minimum across all
        # joints (the binding joint sets T, exactly like the firmware).
        needed = max(min_duration(qt - q0) for q0, qt in zip(q_prev, q_target))
        move_T = max(commanded, needed)
        segments.append((list(q_prev), q_target, move_T, settle))
        q_prev = q_target

    return Trajectory(name=data.get("name", Path(path).stem),
                      joint_order=joint_order, home=home, segments=segments)


def sample(traj: Trajectory, dt: float):
    """Expand the whole trajectory to (t, Q) at fixed dt.

    Q has shape (len(t), n_joints), angles in degrees. Settle/dwell holds the
    last target (zero velocity), giving a clean IMU baseline between moves.
    """
    ts, Qs = [], []
    clock = 0.0
    for q0, qf, move_T, settle in traj.segments:
        n = max(1, int(round(move_T / dt)))
        local = np.linspace(0.0, move_T, n + 1)
        segs = [Quintic(a, b, move_T) for a, b in zip(q0, qf)]
        Q = np.stack([s.position(local) for s in segs], axis=1)
        ts.append(clock + local)
        Qs.append(Q)
        clock += move_T
        if settle > 0:
            n_s = max(1, int(round(settle / dt)))
            local_s = np.linspace(0.0, settle, n_s + 1)[1:]
            ts.append(clock + local_s)
            Qs.append(np.repeat(np.asarray(qf)[None, :], len(local_s), axis=0))
            clock += settle
    t = np.concatenate(ts)
    Q = np.concatenate(Qs, axis=0)
    # De-duplicate the shared boundary samples.
    keep = np.concatenate([[True], np.diff(t) > 1e-9])
    return t[keep], Q[keep]


if __name__ == "__main__":
    import sys

    here = Path(__file__).resolve().parent.parent
    default = here / "config" / "trajectories" / "imu_calibration.yaml"
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    traj = load(path)

    print(f"Trajectory '{traj.name}'  joints={traj.joint_order}")
    print(f"  home={traj.home}  total={traj.total_duration:.2f}s "
          f"({len(traj.segments)} segments)")
    for i, (q0, qf, T, s) in enumerate(traj.segments, 1):
        moved = [j for j, (a, b) in enumerate(zip(q0, qf)) if abs(b - a) > 1e-6]
        names = ", ".join(f"{traj.joint_order[j]} {q0[j]:.0f}->{qf[j]:.0f}"
                          for j in moved) or "hold"
        print(f"  {i:2d}. move {T:5.2f}s + settle {s:4.2f}s   [{names}]")

    t, Q = sample(traj, dt=0.005)
    print(f"  sampled {len(t)} steps @5ms -> shape {Q.shape}, "
          f"final={Q[-1].tolist()}")
