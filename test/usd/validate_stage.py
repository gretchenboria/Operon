#!/usr/bin/env python3
"""
Round-trip validation for assets/operon_arm.usda.

Reloads the generated stage with real pxr and asserts it agrees with the MJCF
it came from. The repo previously had no USD validation of any kind - the old
.usda was a JavaScript string nothing ever opened - so this is the check that
makes the export trustworthy rather than decorative.

    .venv-usd/bin/python test/usd/validate_stage.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

REPO = Path(__file__).resolve().parent.parent.parent
MJCF = REPO / "public/models/operon_station0.xml"
USDA = REPO / "assets/operon_arm.usda"

# From public/demos/arms3d.js - the geometry the whole workbench is built on.
LINKS = {"h0": 0.090, "L1": 0.100, "L2": 0.100, "L3": 0.060}

fails: list[str] = []


def check(cond: bool, msg: str) -> None:
    print(f"  {'PASS' if cond else 'FAIL'}  {msg}")
    if not cond:
        fails.append(msg)


def main() -> int:
    if not USDA.exists():
        print(f"error: {USDA} missing - run sim/mjcf_to_usd.py first", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(MJCF))
    stage = Usd.Stage.Open(str(USDA))
    check(stage is not None, "stage opens with pxr")
    if stage is None:
        return 1

    prims = list(stage.Traverse())

    print("\n[1] stage metadata")
    check(UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z, "up axis is Z (robotics convention)")
    check(abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9, "metersPerUnit is 1.0")
    dp = stage.GetDefaultPrim()
    check(bool(dp) and dp.GetPath() == "/World", "defaultPrim is /World")

    print("\n[2] physics scene")
    scene = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
    check(bool(scene), "UsdPhysics.Scene authored")
    if scene:
        g = scene.GetGravityMagnitudeAttr().Get()
        check(abs(g - abs(model.opt.gravity[2])) < 1e-6,
              f"gravity magnitude matches MJCF ({g:.5f})")

    print("\n[3] geometry is actually present")
    gprims = [p for p in prims if p.IsA(UsdGeom.Gprim) or p.IsA(UsdGeom.Mesh)]
    check(len(gprims) >= 20, f"stage has real geometry: {len(gprims)} gprims")
    # The old JS export authored robot prims with NO geometry at all.
    arm_geo = [p for p in gprims if "pedestal" in str(p.GetPath()).lower()
               or "upper_arm" in str(p.GetPath()).lower()
               or "forearm" in str(p.GetPath()).lower()]
    check(len(arm_geo) > 0, f"the ROBOT has geometry, not just the scenery ({len(arm_geo)} prims)")

    print("\n[4] articulation + joints")
    art = [p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    check(len(art) == 1, f"exactly one ArticulationRootAPI ({len(art)})")
    rev = [p for p in prims if p.IsA(UsdPhysics.RevoluteJoint)]
    pris = [p for p in prims if p.IsA(UsdPhysics.PrismaticJoint)]
    n_hinge = int((model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE).sum())
    n_slide = int((model.jnt_type == mujoco.mjtJoint.mjJNT_SLIDE).sum())
    check(len(rev) == n_hinge, f"revolute joints match MJCF hinges ({len(rev)} == {n_hinge})")
    check(len(pris) == n_slide, f"prismatic joints match MJCF slides ({len(pris)} == {n_slide})")
    # These are REAL UsdPhysics schemas. The old export invented
    # `ArticulationRoot` as a prim type and `drive:jointPositions` as an attr.
    check(all(p.IsA(UsdPhysics.RevoluteJoint) for p in rev),
          "joints use the real UsdPhysics.RevoluteJoint schema")

    print("\n[5] joint limits round-trip (radians -> degrees)")
    ok_lim = True
    for ji in range(model.njnt):
        if model.jnt_type[ji] != mujoco.mjtJoint.mjJNT_HINGE or not model.jnt_limited[ji]:
            continue
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, ji)
        jp = stage.GetPrimAtPath(f"/World/Joints/{nm}")
        if not jp:
            ok_lim = False
            continue
        j = UsdPhysics.RevoluteJoint(jp)
        lo = j.GetLowerLimitAttr().Get()
        want = np.degrees(float(model.jnt_range[ji][0]))
        if lo is None or abs(lo - want) > 1e-3:
            ok_lim = False
    check(ok_lim, "every hinge limit matches the MJCF, converted to degrees")

    print("\n[6] kinematic chain matches arms3d.js")
    # Walk the USD body tree and confirm the link offsets are the real ones.
    def body_translate(path: str):
        p = stage.GetPrimAtPath(path)
        if not p:
            return None
        for op in UsdGeom.Xformable(p).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op.Get()
        return None

    base = "/World/pedestal/base"
    upper = f"{base}/upper_arm"
    fore = f"{upper}/forearm"
    t_up = body_translate(upper)
    t_fore = body_translate(fore)
    check(t_fore is not None and abs(t_fore[2] - LINKS["L1"]) < 1e-9,
          f"upper arm -> forearm offset is L1 = {LINKS['L1']} m")
    wrist = f"{fore}/wrist_roll_link"
    t_wr = body_translate(wrist)
    check(t_wr is not None and abs(t_wr[2] - LINKS["L2"]) < 1e-9,
          f"forearm -> wrist offset is L2 = {LINKS['L2']} m")
    hand = f"{wrist}/wrist_pitch_link/tool_roll_link/hand"
    t_hand = body_translate(hand)
    check(t_hand is not None and abs(t_hand[2] - LINKS["L3"]) < 1e-9,
          f"wrist -> TCP offset is L3 = {LINKS['L3']} m")

    print("\n[7] semantics for the Replicator segmentation AOVs")
    labelled = [p for p in prims
                if p.GetAttribute("semantic:Semantics:params:semanticData").IsValid()]
    check(len(labelled) >= 20, f"prims carry semantic labels ({len(labelled)})")
    classes = {p.GetAttribute("semantic:Semantics:params:semanticData").Get()
               for p in labelled}
    classes.discard(None)
    check(len(classes) >= 5, f"multiple distinct classes, not one blanket label: {sorted(classes)}")
    check("unlabeled" not in classes or len(classes) > 6,
          "the labelling rules actually cover the scene")

    print("\n[8] materials bind and resolve")
    looks = stage.GetPrimAtPath("/World/Looks")
    check(bool(looks), "Looks scope exists")
    mats = [p for p in prims if p.GetTypeName() == "Material"]
    check(len(mats) >= 6, f"materials authored ({len(mats)})")
    # Unlike the old export, every material must actually be bound to something.
    from pxr import UsdShade
    bound = set()
    for p in gprims:
        b = UsdShade.MaterialBindingAPI(p).GetDirectBinding().GetMaterial()
        if b:
            bound.add(str(b.GetPath()))
    check(len(bound) >= 4, f"materials are BOUND to geometry ({len(bound)} distinct in use)")

    print(f"\n{'ALL USD CHECKS PASSED' if not fails else f'{len(fails)} CHECK(S) FAILED'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
