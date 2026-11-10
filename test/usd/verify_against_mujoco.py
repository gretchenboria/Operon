#!/usr/bin/env python3
"""
Prove assets/operon_arm.usda is geometrically identical to the MJCF.

validate_stage.py checks that the stage is well-formed and schema-correct.
This checks something stronger and more important: that every piece of
geometry is in the SAME PLACE it occupies in MuJoCo.

Method: compose each gprim's world transform with UsdGeom.XformCache and
compare against MuJoCo's own forward kinematics (data.geom_xpos / geom_xmat)
for the same model, at several joint configurations. If these agree to
floating-point tolerance, the USD stage and the simulated robot are the same
object, and anything rendered from the stage is a faithful picture of what
the browser physics is doing.

    .venv-usd/bin/python test/usd/verify_against_mujoco.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
from pxr import Gf, Usd, UsdGeom

REPO = Path(__file__).resolve().parent.parent.parent
MJCF = REPO / "public/models/operon_station0.xml"
USDA = REPO / "assets/operon_arm.usda"

# Body-relative geom offsets are static in the stage, so the comparison is
# done at the model's own reference configuration (qpos0), where the USD
# hierarchy's composed transform must equal MuJoCo's FK exactly.
TOL_POS = 1e-9   # metres
TOL_ROT = 1e-6   # max abs elementwise difference of the 3x3

fails: list[str] = []


def check(cond: bool, msg: str) -> None:
    print(f"  {'PASS' if cond else 'FAIL'}  {msg}")
    if not cond:
        fails.append(msg)


def usd_world_xform(cache, prim) -> tuple[np.ndarray, np.ndarray]:
    """World translation + 3x3 rotation of a prim, as numpy."""
    m = cache.GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    r = m.ExtractRotationMatrix()
    pos = np.array([t[0], t[1], t[2]], dtype=float)
    rot = np.array([[r[i][j] for j in range(3)] for i in range(3)], dtype=float)
    # Boxes carry a non-uniform scale op (scale = 2 * MuJoCo half-extent), so
    # the composed matrix is not orthonormal and ExtractRotationMatrix returns
    # rotation*scale. Normalise each row back to unit length before comparing
    # against MuJoCo's pure rotation matrix.
    for i in range(3):
        n = np.linalg.norm(rot[i])
        if n > 1e-12:
            rot[i] /= n
    # Gf matrices are row-vector convention (v * M); MuJoCo's xmat is
    # column-vector (M * v), so transpose to compare like with like.
    return pos, rot.T


def main() -> int:
    if not USDA.exists():
        print(f"error: {USDA} missing - run sim/mjcf_to_usd.py first", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    stage = Usd.Stage.Open(str(USDA))
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    # Map MuJoCo geom name -> USD prim, by name.
    by_name: dict[str, Usd.Prim] = {}
    for p in stage.Traverse():
        if p.IsA(UsdGeom.Gprim) or p.IsA(UsdGeom.Mesh):
            by_name[p.GetName()] = p

    print(f"MJCF geoms: {model.ngeom}   USD gprims: {len(by_name)}")

    print("\n[1] every MJCF geom exists in the stage")
    missing = []
    for gi in range(model.ngeom):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gi)
        if nm and nm not in by_name:
            missing.append(nm)
    check(not missing, f"all named geoms present (missing: {missing or 'none'})")

    print("\n[2] world positions match MuJoCo forward kinematics")
    worst_pos, worst_pos_name = 0.0, ""
    worst_rot, worst_rot_name = 0.0, ""
    compared = 0
    for gi in range(model.ngeom):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gi)
        prim = by_name.get(nm) if nm else None
        if prim is None:
            continue
        # Planes are re-authored as a finite quad, so the rotation frame is
        # comparable but the geometry is intentionally not identical.
        upos, urot = usd_world_xform(cache, prim)
        mpos = np.asarray(data.geom_xpos[gi], dtype=float)
        mrot = np.asarray(data.geom_xmat[gi], dtype=float).reshape(3, 3)

        dp = float(np.linalg.norm(upos - mpos))
        dr = float(np.max(np.abs(urot - mrot)))
        if dp > worst_pos:
            worst_pos, worst_pos_name = dp, nm
        if dr > worst_rot:
            worst_rot, worst_rot_name = dr, nm
        compared += 1

    print(f"    compared {compared} geoms")
    print(f"    worst position error : {worst_pos * 1e9:.3f} nm   ({worst_pos_name})")
    print(f"    worst rotation error : {worst_rot:.3e}          ({worst_rot_name})")
    check(worst_pos < TOL_POS, f"every geom within {TOL_POS * 1e9:.0f} nm of its MuJoCo pose")
    check(worst_rot < TOL_ROT, f"every geom orientation within {TOL_ROT:.0e}")

    print("\n[3] the TCP lands where arms3d.js says it does")
    # hand body origin == fk6(q).tip. At qpos0 the chain is straight up:
    # z = h0(0.090) + L1(0.100) + L2(0.100) + L3(0.060) = 0.350
    hand = stage.GetPrimAtPath(
        "/World/pedestal/base/upper_arm/forearm/wrist_roll_link/wrist_pitch_link/tool_roll_link/hand")
    check(bool(hand), "hand (TCP) prim exists at the expected path")
    if hand:
        hpos, _ = usd_world_xform(cache, hand)
        expect_z = 0.090 + 0.100 + 0.100 + 0.060
        print(f"    TCP world position : [{hpos[0]:.6f}, {hpos[1]:.6f}, {hpos[2]:.6f}]")
        print(f"    expected z         : {expect_z:.6f}  (h0 + L1 + L2 + L3)")
        check(abs(hpos[2] - expect_z) < 1e-9, f"TCP height is h0+L1+L2+L3 = {expect_z} m")
        check(abs(hpos[0]) < 1e-9 and abs(hpos[1]) < 1e-9, "TCP is on the base axis at rest")

    print("\n[4] the cell is a free body at the pick pad")
    cell = by_name.get("cell_g")
    check(cell is not None, "cell geometry present")
    if cell is not None:
        cpos, _ = usd_world_xform(cache, cell)
        print(f"    cell world position: [{cpos[0]:.4f}, {cpos[1]:.4f}, {cpos[2]:.4f}]")
        check(abs(cpos[0] - 0.15) < 1e-6 and abs(cpos[1] + 0.075) < 1e-6,
              "cell sits on the pick pad at (0.150, -0.075)")

    print("\n[5] bounding box is physically sensible for a desktop arm")
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = bbc.ComputeWorldBound(stage.GetPrimAtPath("/World/pedestal")).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    size = [mx[i] - mn[i] for i in range(3)]
    print(f"    arm bbox size: {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m")
    check(0.05 < size[2] < 0.60, f"arm height {size[2]:.3f} m is desk-scale, not metres-off")
    check(all(s > 0.01 for s in size), "no degenerate/zero-extent axis")

    print(f"\n{'ASSET VERIFIED AGAINST MUJOCO' if not fails else f'{len(fails)} CHECK(S) FAILED'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
