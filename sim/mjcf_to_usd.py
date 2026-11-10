#!/usr/bin/env python3
"""
MJCF -> OpenUSD converter for Operon.

Why this exists rather than just calling Isaac Sim's importer
-------------------------------------------------------------
Isaac Sim 6.x ships `isaacsim.asset.importer.mjcf`, and that remains the
cross-check on the GPU box. But its Python backend (`mujoco-usd-converter`)
depends on `usd-exchange`, which publishes **no macOS wheels** - only
manylinux and win_amd64. Conversion would therefore be gated behind the GPU
VM, which is exactly backwards: the asset is the thing that has to be correct
*before* any GPU is provisioned.

`mujoco` and `usd-core` (real pxr) both install natively everywhere, so this
converter runs on a laptop, in CI, and in the Isaac container identically.

What it guarantees
------------------
The stage is generated from `public/models/operon_station0.xml` - the same
MJCF that drives the shipped browser physics and whose chain reproduces
`fk6()` from `public/demos/arms3d.js` to under a nanometre. So the robot that
gets ray-traced is provably the robot that gets simulated, rather than a
hand-authored look-alike.

Usage
-----
    .venv-usd/bin/python sim/mjcf_to_usd.py \
        --mjcf public/models/operon_station0.xml \
        --out  assets/operon_arm.usda
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

# mjtGeom enum values we handle
G_PLANE, G_SPHERE, G_CAPSULE, G_ELLIPSOID, G_CYLINDER, G_BOX = 0, 2, 3, 4, 5, 6

# Semantic classes, kept in step with tagSemantic() in public/teleop-studio.js
# so the Replicator segmentation AOVs and the browser's instance-ID pass agree.
SEMANTIC_RULES = [
    ("cell", "battery_cell"),
    ("pad_", "gripper_pad"),
    ("finger", "gripper_finger"),
    ("grip_", "gripper_body"),
    ("ft_ring", "force_torque_sensor"),
    ("flange", "tool_flange"),
    ("charger", "charger_dock"),
    ("valve", "valve_wheel"),
    ("pick_pad", "locating_pad"),
    ("drop_pad", "locating_pad"),
    ("table", "optical_bench"),
    ("pedestal", "robot_base"),
    ("upper_arm", "robot_link"),
    ("forearm", "robot_link"),
    ("wrist", "robot_link"),
    ("base_g", "robot_base"),
    ("shoulder", "robot_link"),
    ("elbow", "robot_link"),
    ("tool_roll", "robot_link"),
]


def semantic_class(name: str) -> str:
    low = name.lower()
    for key, cls in SEMANTIC_RULES:
        if key in low:
            return cls
    return "unlabeled"


def mj_name(model, objtype, idx: int, fallback: str) -> str:
    nm = mujoco.mj_id2name(model, objtype, idx)
    return nm if nm else f"{fallback}{idx}"


def sanitize(name: str) -> str:
    """USD prim names must be valid identifiers."""
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    return ("_" + out) if (not out or out[0].isdigit()) else out


def quat_to_gf(q) -> Gf.Quatd:
    """
    MuJoCo stores quaternions w,x,y,z; Gf wants (real, imaginary).

    Double precision on purpose: a float32 orient op costs ~4 nm of placement
    error at this scale, which is enough to fail an exact comparison against
    MuJoCo's own float64 forward kinematics.
    """
    return Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3])))


def set_xform(prim, pos, quat) -> None:
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat_to_gf(quat))


def author_materials(stage, root: str) -> dict:
    """
    UsdPreviewSurface materials, which every USD renderer understands.

    The RTX-specific MDL (OmniPBR) upgrade is applied on top of these inside
    Kit by sim/shadow_rtx_bridge.py, where the MDL shader nodes actually
    resolve. Authoring MDL here would produce bindings that fail to resolve in
    usdview, Blender or Hydra Storm, so preview surface is the correct
    portable base layer.
    """
    looks = UsdGeom.Scope.Define(stage, f"{root}/Looks")
    palette = {
        "Steel":     ((0.62, 0.64, 0.68), 0.85, 0.35),
        "Carbon":    ((0.14, 0.15, 0.17), 0.30, 0.55),
        "Rubber":    ((0.12, 0.12, 0.14), 0.00, 0.90),
        "Copper":    ((0.72, 0.45, 0.20), 0.95, 0.28),
        "CellGold":  ((0.80, 0.72, 0.22), 0.80, 0.30),
        "PadBlue":   ((0.20, 0.42, 0.78), 0.10, 0.45),
        "PadGreen":  ((0.18, 0.62, 0.36), 0.10, 0.45),
        "Bench":     ((0.18, 0.19, 0.21), 0.25, 0.65),
    }
    mats = {}
    for name, (rgb, metal, rough) in palette.items():
        mat = UsdShade.Material.Define(stage, f"{looks.GetPath()}/{name}")
        sh = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        mats[name] = mat
    return mats


def pick_material(geom_name: str, rgba, mats: dict):
    n = geom_name.lower()
    if "pad_l" in n or "pad_r" in n:
        return mats["Rubber"]
    if "cell" in n:
        return mats["CellGold"]
    if "pick_pad" in n:
        return mats["PadBlue"]
    if "drop_pad" in n:
        return mats["PadGreen"]
    if "ft_ring" in n:
        return mats["Copper"]
    if "table" in n:
        return mats["Bench"]
    if "grip_body" in n or "pedestal" in n:
        return mats["Carbon"]
    return mats["Steel"]


def add_geom(stage, parent_path: str, model, gi: int, mats: dict):
    gtype = int(model.geom_type[gi])
    name = sanitize(mj_name(model, mujoco.mjtObj.mjOBJ_GEOM, gi, "geom"))
    path = f"{parent_path}/{name}"
    size = model.geom_size[gi]
    rgba = model.geom_rgba[gi]

    if gtype == G_SPHERE:
        prim = UsdGeom.Sphere.Define(stage, path)
        prim.CreateRadiusAttr(float(size[0]))
        r = float(size[0])
        prim.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-r, -r, -r), Gf.Vec3f(r, r, r)]))
    elif gtype == G_CAPSULE:
        prim = UsdGeom.Capsule.Define(stage, path)
        prim.CreateRadiusAttr(float(size[0]))
        prim.CreateHeightAttr(float(size[1]) * 2.0)  # MuJoCo stores half-length
        prim.CreateAxisAttr("Z")
        r, h = float(size[0]), float(size[1])
        prim.CreateExtentAttr(Vt.Vec3fArray(
            [Gf.Vec3f(-r, -r, -(h + r)), Gf.Vec3f(r, r, h + r)]))
    elif gtype == G_CYLINDER:
        prim = UsdGeom.Cylinder.Define(stage, path)
        prim.CreateRadiusAttr(float(size[0]))
        prim.CreateHeightAttr(float(size[1]) * 2.0)
        prim.CreateAxisAttr("Z")
        r, h = float(size[0]), float(size[1])
        prim.CreateExtentAttr(Vt.Vec3fArray(
            [Gf.Vec3f(-r, -r, -h), Gf.Vec3f(r, r, h)]))
    elif gtype == G_BOX:
        # UsdGeom.Cube is unit-sized; MuJoCo box size is a half-extent, so the
        # scale is 2 * half_extent.
        prim = UsdGeom.Cube.Define(stage, path)
        prim.CreateSizeAttr(1.0)
        prim.CreateExtentAttr(Vt.Vec3fArray(
            [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)]))
    elif gtype == G_ELLIPSOID:
        prim = UsdGeom.Sphere.Define(stage, path)
        prim.CreateRadiusAttr(1.0)
        prim.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, 1, 1)]))
    elif gtype == G_PLANE:
        # An infinite MuJoCo plane becomes a large finite Mesh so it is
        # actually visible to a renderer.
        half = float(size[0]) if size[0] > 0 else 1.0
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray([
            Gf.Vec3f(-half, -half, 0), Gf.Vec3f(half, -half, 0),
            Gf.Vec3f(half, half, 0), Gf.Vec3f(-half, half, 0)]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        mesh.CreateExtentAttr(Vt.Vec3fArray(
            [Gf.Vec3f(-half, -half, 0), Gf.Vec3f(half, half, 0)]))
        prim = mesh
    else:
        return None  # meshes/hfields are not used by this scene

    set_xform(prim.GetPrim(), model.geom_pos[gi], model.geom_quat[gi])

    if gtype == G_BOX:
        UsdGeom.Xformable(prim.GetPrim()).AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(float(size[0]) * 2, float(size[1]) * 2, float(size[2]) * 2))
    elif gtype == G_ELLIPSOID:
        UsdGeom.Xformable(prim.GetPrim()).AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(float(size[0]), float(size[1]), float(size[2])))

    prim.CreateDisplayColorAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(rgba[0]), float(rgba[1]), float(rgba[2]))]))

    mat = pick_material(name, rgba, mats)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)

    # Collision, so the stage is physically meaningful in Isaac, not just art.
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())

    # Semantic label for the Replicator segmentation AOVs. Kit's `Semantics`
    # schema is not in usd-core, so author the same data as custom attributes;
    # shadow_rtx_bridge.py promotes these to a real SemanticsAPI on import.
    p = prim.GetPrim()
    p.CreateAttribute("semantic:Semantics:params:semanticType",
                      Sdf.ValueTypeNames.String).Set("class")
    p.CreateAttribute("semantic:Semantics:params:semanticData",
                      Sdf.ValueTypeNames.String).Set(semantic_class(name))
    return prim


# Keys are plain ints on purpose: model.jnt_type yields numpy.int32, which does
# NOT hash equal to a mujoco.mjtJoint enum member. An enum-keyed dict silently
# misses every joint and the stage comes out with none at all.
JOINT_TYPE = {
    int(mujoco.mjtJoint.mjJNT_HINGE): "revolute",
    int(mujoco.mjtJoint.mjJNT_SLIDE): "prismatic",
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",
}


def add_joint(stage, model, ji: int, body_paths: dict, root: str):
    jtype = int(model.jnt_type[ji])
    kind = JOINT_TYPE.get(jtype)
    if kind in (None, "free"):
        return None  # a free body needs RigidBodyAPI, not a joint prim

    child_body = int(model.jnt_bodyid[ji])
    parent_body = int(model.body_parentid[child_body])
    child_path = body_paths.get(child_body)
    parent_path = body_paths.get(parent_body)
    if child_path is None or parent_path is None:
        return None

    name = sanitize(mj_name(model, mujoco.mjtObj.mjOBJ_JOINT, ji, "joint"))
    path = f"{root}/Joints/{name}"

    if kind == "revolute":
        j = UsdPhysics.RevoluteJoint.Define(stage, path)
    elif kind == "prismatic":
        j = UsdPhysics.PrismaticJoint.Define(stage, path)
    else:
        j = UsdPhysics.SphericalJoint.Define(stage, path)

    j.CreateBody0Rel().SetTargets([parent_path])
    j.CreateBody1Rel().SetTargets([child_path])

    # UsdPhysics takes a principal axis token, not a free vector. Pick the
    # dominant component; the MJCF only uses axis-aligned joints.
    axis = np.asarray(model.jnt_axis[ji], dtype=float)
    j.CreateAxisAttr(["X", "Y", "Z"][int(np.argmax(np.abs(axis)))])

    anchor = model.jnt_pos[ji]
    j.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in anchor]))
    bpos = model.body_pos[child_body]
    j.CreateLocalPos0Attr(Gf.Vec3f(*[float(bpos[i] + anchor[i]) for i in range(3)]))

    if bool(model.jnt_limited[ji]):
        lo, hi = [float(v) for v in model.jnt_range[ji]]
        if kind == "revolute":  # UsdPhysics angular limits are in DEGREES
            lo, hi = np.degrees(lo), np.degrees(hi)
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
    return j


def convert(mjcf_path: Path, out_path: Path, up_axis: str = "Z") -> Usd.Stage:
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))

    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z if up_axis == "Z" else UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = "/World"
    world = UsdGeom.Xform.Define(stage, root)
    stage.SetDefaultPrim(world.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, f"{root}/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(float(abs(model.opt.gravity[2])))

    mats = author_materials(stage, root)

    # Body tree. MuJoCo body 0 is the world body; its geoms are static scenery.
    body_paths = {0: root}
    for bi in range(1, model.nbody):
        parent = int(model.body_parentid[bi])
        name = sanitize(mj_name(model, mujoco.mjtObj.mjOBJ_BODY, bi, "body"))
        path = f"{body_paths.get(parent, root)}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        set_xform(xf.GetPrim(), model.body_pos[bi], model.body_quat[bi])
        body_paths[bi] = path

        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(xf.GetPrim())
        mass_api.CreateMassAttr(float(model.body_mass[bi]))

    # Geoms
    for gi in range(model.ngeom):
        bi = int(model.geom_bodyid[gi])
        add_geom(stage, body_paths.get(bi, root), model, gi, mats)

    # Sites. MuJoCo sites are massless frames, and the bridge needs them:
    # `forearm_imu` is where the IMUSensor is mounted and `tcp` is the tool
    # centre point. Exported as empty Xforms so Isaac can attach sensors to a
    # real prim path rather than a hardcoded offset.
    for si in range(model.nsite):
        bi = int(model.site_bodyid[si])
        nm = sanitize(mj_name(model, mujoco.mjtObj.mjOBJ_SITE, si, "site"))
        parent = body_paths.get(bi, root)
        sp = UsdGeom.Xform.Define(stage, f"{parent}/{nm}")
        set_xform(sp.GetPrim(), model.site_pos[si], model.site_quat[si])
        sp.GetPrim().CreateAttribute("operon:siteName", Sdf.ValueTypeNames.String).Set(
            mj_name(model, mujoco.mjtObj.mjOBJ_SITE, si, "site"))

    # Joints
    UsdGeom.Scope.Define(stage, f"{root}/Joints")
    njoint = 0
    for ji in range(model.njnt):
        if add_joint(stage, model, ji, body_paths, root) is not None:
            njoint += 1

    # Articulation root on the kinematic base, so Isaac treats the chain as one
    # articulation rather than a pile of rigid bodies.
    for bi in range(1, model.nbody):
        nm = mj_name(model, mujoco.mjtObj.mjOBJ_BODY, bi, "body")
        if nm == "pedestal":
            UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body_paths[bi]))
            break

    stage.GetRootLayer().Save()
    return stage


def main() -> int:
    ap = argparse.ArgumentParser(description="Convert Operon's MJCF to OpenUSD.")
    repo = Path(__file__).resolve().parent.parent
    ap.add_argument("--mjcf", default=str(repo / "public/models/operon_station0.xml"))
    ap.add_argument("--out", default=str(repo / "assets/operon_arm.usda"))
    args = ap.parse_args()

    mjcf, out = Path(args.mjcf), Path(args.out)
    if not mjcf.exists():
        print(f"error: MJCF not found: {mjcf}", file=sys.stderr)
        return 1
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()

    stage = convert(mjcf, out)
    nprims = sum(1 for _ in stage.Traverse())
    print(f"wrote {out}")
    print(f"  prims      : {nprims}")
    print(f"  up axis    : {UsdGeom.GetStageUpAxis(stage)}")
    print(f"  defaultPrim: {stage.GetDefaultPrim().GetPath()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
