#!/usr/bin/env python3
r"""
sim/shadow_rtx_bridge.py — Shadow PC Live Omniverse RTX + PhysX Bridge for Operon Studio
=========================================================================================
Runs inside NVIDIA Isaac Sim 4.5 on a Shadow PC (Windows 11 / RTX GPU) and connects
OUTBOUND over WebSockets to the Operon Cloud Run relay (zero port-forwarding required,
works seamlessly behind Shadow PC's CGNAT).

What it does in real time:
  1. Boots Omniverse Kit (`SimulationApp`) with the `RayTracedLighting` RTX renderer and
     configures `carb.settings` for DLSS, RTX reflections, translucency, ambient occlusion,
     indirect diffuse GI, sampled direct lighting, and ACES Filmic tonemapping.
  2. Loads your robot USD (`--usd`) OR builds a complete OpenUSD workbench scene on the fly:
     - `/World/Looks` Physically-Based `UsdShade.Material` + `UsdPreviewSurface` shader library
       bound via `UsdShade.MaterialBindingAPI` to every prim (`Mat_ESD_OpticalBench`,
       `Mat_IdealArm_AnodizedViolet`, `Mat_RealArm_AnodizedCobalt`, `Mat_BrushedSteel_Joints`,
       `Mat_CarbonGripper`, `Mat_BatteryCell_21700`, `Mat_BrushedNickel_Terminal`,
       `Mat_ChargerDock`, `Mat_ValveWheel`, `Mat_WatchIMU`).
     - 3-point studio lighting rig (`DomeLight`, `KeyLight` DistantLight, `FillRectLight`
       `UsdLux.RectLight`, `RimRectLight` `UsdLux.RectLight`).
     - Dual articulated 6-DOF arms (`IdealArm` and `RealArm`) with link capsules, brushed
       steel harmonic-drive joint housings, parallel-jaw carbon-fiber grippers, optical bench,
       21700 Li-Ion battery cell + terminals, charger dock, valve handwheel, and a PhysX
       `IMUSensor` mounted on `/World/AbelArm/forearm_link/watch_site/imu_sensor`.
     - `Semantics.SemanticsAPI` class labels on every prim for the Replicator
       `semantic_segmentation` AOV.
  3. Subscribes to live 6-DOF teleop joint targets (`q6`, `q4`, `closed`, `aov`,
     and `light`, `angle`, `rough` domain randomization sliders) from Operon Studio / Apple Watch.
  4. Articulates both `IdealArm` and `RealArm` in real time using full 6-DOF forward kinematics,
     updates `UsdShade.Shader` roughness and `UsdLux` lighting attributes on every frame,
     steps PhysX at 120 Hz, reads the real PhysX `IMUSensor` (`lin_acc`, `ang_vel`),
     captures the live Omniverse RTX Replicator AOV (`rgb`, `distance_to_camera`,
     `normals`, `semantic_segmentation`), and streams JPEG frames + PhysX IMU back
     into the Operon Studio browser viewport at ~20 FPS.

Usage on Shadow PC (PowerShell / CMD):
  set ISAAC_PYTHON=%LOCALAPPDATA%\ov\pkg\isaac-sim-4.5.0\python.bat
  %ISAAC_PYTHON% sim\shadow_rtx_bridge.py --room OPERON
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

G0 = 9.80665  # Standard gravity (m/s^2)
H0 = 0.090    # Base pedestal + shoulder height (m)
L1 = 0.100    # Upper arm length (m)
L2 = 0.100    # Forearm length (m)
L3 = 0.060    # Wrist-to-tool-tip length (m)


def ensure_websocket_client() -> None:
    try:
        import websockets  # noqa: F401
    except ImportError:
        print("[shadow-rtx] Installing 'websockets' and 'pillow' into Isaac Sim Python...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "websockets", "pillow"])


def rot_x(a: float, np):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rot_y(a: float, np):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(a: float, np):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def forearm_fk_rad(q_rad, np):
    """Compute world position p (3,) and rotation matrix R (3,3) of the forearm watch IMU site
    from URDF joint radians [t1, t2, t3, ...]. Matches public/teleop-studio.js:forearmSiteFK.
    """
    t1 = float(q_rad[0]) if len(q_rad) > 0 else 0.0
    t2 = float(q_rad[1]) if len(q_rad) > 1 else 0.0
    t3 = float(q_rad[2]) if len(q_rad) > 2 else 0.0
    p1 = t2
    p12 = t2 + t3
    r_local = L1 * math.sin(p1) + L2 * math.sin(p12)
    z_local = H0 + L1 * math.cos(p1) + L2 * math.cos(p12)
    cy, sy = math.cos(t1), math.sin(t1)
    p = np.array([cy * r_local, sy * r_local, z_local], dtype=float)
    R = rot_z(t1, np) @ rot_y(p12, np)
    return p, R


def fk6_chain(q_rad, np):
    """Full 6-DOF forward kinematics matching public/demos/arms3d.js:fk6.
    Accepts 4-DOF [t1, t2, t3, t4_pitch] or 6-DOF [t1, t2, t3, t4, t5, t6] in radians.
    Returns dict with:
      pts: list of 5 3D points [base, shoulder, elbow, wrist, flange/tip]
      joints: list of 6 (pos, axis_vec) tuples for actuator housings
      R_tip: 3x3 tool rotation matrix
      tip: 3D tool tip position
    """
    if len(q_rad) < 6:
        t1 = float(q_rad[0]) if len(q_rad) > 0 else 0.0
        t2 = float(q_rad[1]) if len(q_rad) > 1 else 0.0
        t3 = float(q_rad[2]) if len(q_rad) > 2 else 0.0
        t4 = 0.0
        t5 = float(q_rad[3]) if len(q_rad) > 3 else 0.0
        t6 = 0.0
    else:
        t1, t2, t3, t4, t5, t6 = [float(x) for x in q_rad[:6]]

    Y = rot_z(t1, np)
    base = np.array([0.0, 0.0, 0.0], dtype=float)
    sh = np.array([0.0, 0.0, H0], dtype=float)
    el = sh + Y @ np.array([L1 * math.sin(t2), 0.0, L1 * math.cos(t2)], dtype=float)
    R03 = Y @ rot_y(t2 + t3, np)
    fore_axis = R03[:, 2]
    wr = el + fore_axis * L2
    R04 = R03 @ rot_z(t4, np)
    R_tip = R04 @ rot_y(t5, np) @ rot_z(t6, np)
    tool_z = R_tip[:, 2]
    tip = wr + tool_z * L3
    flange = tip - tool_z * 0.018

    y_ax = Y[:, 1]
    joints = [
        (np.array([0.0, 0.0, 0.020], dtype=float), np.array([0.0, 0.0, 1.0], dtype=float)),
        (sh, y_ax),
        (el, y_ax),
        (el + fore_axis * (L2 * 0.55), fore_axis),
        (wr, R04[:, 1]),
        (wr + tool_z * (L3 * 0.35), tool_z),
    ]
    return {
        "pts": [base, sh, el, wr, flange],
        "joints": joints,
        "R_tip": R_tip,
        "tip": tip,
        "flange": flange,
    }


def configure_rtx_render_settings(mode: str = "RayTracedLighting", subframes: int = 32) -> None:
    """
    Configure Omniverse Kit carb.settings for the chosen RTX render mode.

    Two genuinely different paths, not a cosmetic switch:

      RayTracedLighting - real-time ray tracing. Interactive, ~20 fps, what
          the live studio overlay uses.
      PathTracing       - progressive path tracing with `subframes`
          accumulation steps per frame. Converged GI, soft shadows and
          reflections; far too slow to stream, used for the synthetic
          dataset that gets kept.

    The previous version of this file only ever requested RayTracedLighting
    and never exposed path tracing at all.
    """
    try:
        import carb.settings  # type: ignore

        settings = carb.settings.get_settings()
        settings.set("/rtx/rendermode", mode)
        if mode == "PathTracing":
            settings.set("/rtx/pathtracing/spp", 1)
            settings.set("/rtx/pathtracing/totalSpp", int(subframes))
            settings.set("/rtx/pathtracing/maxBounces", 6)
            settings.set("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 8)
            settings.set("/rtx/pathtracing/clampSpp", int(subframes))
        settings.set("/rtx/post/dlss/execMode", 1)  # 0=Performance, 1=Balanced, 2=Quality
        settings.set("/rtx/reflections/enabled", True)
        settings.set("/rtx/reflections/maxRoughness", 0.65)
        settings.set("/rtx/translucency/enabled", True)
        settings.set("/rtx/indirectDiffuse/enabled", True)
        settings.set("/rtx/ambientOcclusion/enabled", True)
        settings.set("/rtx/ambientOcclusion/intensity", 1.15)
        settings.set("/rtx/directLighting/sampledLighting/enabled", True)
        settings.set("/rtx/post/tonemap/op", 6)  # ACES Filmic tonemapping
        settings.set("/rtx/post/tonemap/cameraExposure", 0.15)
        print(f"[shadow-rtx] carb.settings configured: /rtx/rendermode={mode}"
              + (f" totalSpp={subframes}" if mode == "PathTracing" else ""))
    except Exception as exc:
        print(f"[shadow-rtx] Warning: carb.settings configuration skipped ({exc}).")



def find_site(stage, site_name: str):
    """
    Locate a MuJoCo site in the generated stage by its original name.

    sim/mjcf_to_usd.py tags each exported site with `operon:siteName`, so the
    bridge can resolve e.g. "forearm_imu" without hardcoding a prim path that
    silently breaks whenever the body hierarchy changes.
    """
    for prim in stage.Traverse():
        attr = prim.GetAttribute("operon:siteName")
        if attr.IsValid() and attr.Get() == site_name:
            return str(prim.GetPath())
    return None


def promote_semantic_attributes(stage) -> int:
    """
    Turn the portable semantic attributes into a real Kit Semantics schema.

    mjcf_to_usd.py runs on stock `usd-core`, which has no Kit `Semantics`
    type, so it writes the same information as plain custom attributes. Inside
    Kit we promote them, because the Replicator semantic/instance
    segmentation annotators only see the real schema - without this the
    segmentation AOVs come back empty and look like a renderer bug.
    """
    promoted = 0
    for prim in stage.Traverse():
        attr = prim.GetAttribute("semantic:Semantics:params:semanticData")
        if not attr.IsValid():
            continue
        cls = attr.Get()
        if not cls or cls == "unlabeled":
            continue
        apply_semantic_label(prim, cls)
        promoted += 1
    print(f"[shadow-rtx] Promoted {promoted} semantic labels to the Kit Semantics schema.")
    return promoted


def add_scene_lighting(stage) -> None:
    """
    Author the lighting rig.

    The generated stage is geometry + physics only: mjcf_to_usd.py
    deliberately does not invent lights, because MJCF has no lighting model
    worth translating. Ray tracing needs real lights, so they are authored
    here where the renderer actually is.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux  # type: ignore

    rig = "/World/Lighting"
    UsdGeom.Scope.Define(stage, rig)

    dome = UsdLux.DomeLight.Define(stage, f"{rig}/Dome")
    dome.CreateIntensityAttr(320.0)
    dome.CreateColorAttr(Gf.Vec3f(0.42, 0.47, 0.60))

    key = UsdLux.DistantLight.Define(stage, f"{rig}/Key")
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(1.2)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-38.0, 0.0, 35.0))

    fill = UsdLux.RectLight.Define(stage, f"{rig}/Fill")
    fill.CreateIntensityAttr(5200.0)
    fill.CreateWidthAttr(0.45)
    fill.CreateHeightAttr(0.30)
    fill.CreateColorAttr(Gf.Vec3f(0.65, 0.75, 1.0))
    fx = UsdGeom.Xformable(fill.GetPrim())
    fx.AddTranslateOp().Set(Gf.Vec3d(-0.25, 0.30, 0.42))
    fx.AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, -130.0))

    rim = UsdLux.RectLight.Define(stage, f"{rig}/Rim")
    rim.CreateIntensityAttr(3400.0)
    rim.CreateWidthAttr(0.30)
    rim.CreateHeightAttr(0.20)
    rim.CreateColorAttr(Gf.Vec3f(1.0, 0.86, 0.72))
    rx = UsdGeom.Xformable(rim.GetPrim())
    rx.AddTranslateOp().Set(Gf.Vec3d(0.42, -0.26, 0.30))
    rx.AddRotateXYZOp().Set(Gf.Vec3f(-62.0, 0.0, 52.0))

    print("[shadow-rtx] Authored 3-point lighting rig + dome.")


def upgrade_materials_to_mdl(stage) -> int:
    """
    Attach an Omniverse RTX OmniPBR MDL shader to every material in /World/Looks
    while preserving the portable UsdPreviewSurface shader on `outputs:surface`.

    Why both:
      - `outputs:surface` (UsdPreviewSurface) resolves in usdview, Storm, Blender,
        and CPU test harnesses.
      - `outputs:mdl:surface` (OmniPBR.mdl) resolves inside Omniverse RTX
        (RayTracedLighting and PathTracing) with full microfacet anisotropy,
        clearcoat, and physical IOR.
    """
    from pxr import Gf, Sdf, UsdShade  # type: ignore

    looks = stage.GetPrimAtPath("/World/Looks")
    if not looks or not looks.IsValid():
        return 0

    upgraded = 0
    for mat_prim in looks.GetChildren():
        if mat_prim.GetTypeName() != "Material":
            continue
        mat = UsdShade.Material(mat_prim)
        preview_sh_prim = stage.GetPrimAtPath(f"{mat_prim.GetPath()}/Shader")
        if not preview_sh_prim or not preview_sh_prim.IsValid():
            continue
        psh = UsdShade.Shader(preview_sh_prim)

        def _get_in(name: str, default):
            inp = psh.GetInput(name)
            return inp.Get() if (inp and inp.Get() is not None) else default

        rgb = _get_in("diffuseColor", Gf.Vec3f(0.6, 0.6, 0.6))
        metal = float(_get_in("metallic", 0.0))
        rough = float(_get_in("roughness", 0.35))
        cc = float(_get_in("clearcoat", 0.0))
        cc_rough = float(_get_in("clearcoatRoughness", 0.15))
        ior = float(_get_in("ior", 1.5))

        mdl_sh = UsdShade.Shader.Define(stage, f"{mat_prim.GetPath()}/MDLShader")
        mdl_sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
        mdl_sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        mdl_sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        mdl_sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(float(rgb[0]), float(rgb[1]), float(rgb[2]))
        )
        mdl_sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metal)
        mdl_sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(rough)
        mdl_sh.CreateInput("enable_coating", Sdf.ValueTypeNames.Bool).Set(bool(cc > 0.01))
        mdl_sh.CreateInput("coating_level", Sdf.ValueTypeNames.Float).Set(cc)
        mdl_sh.CreateInput("coating_roughness", Sdf.ValueTypeNames.Float).Set(cc_rough)
        mdl_sh.CreateInput("ior_constant", Sdf.ValueTypeNames.Float).Set(ior)

        mdl_out = mdl_sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput("mdl").ConnectToSource(mdl_out)
        upgraded += 1

    print(f"[shadow-rtx] Upgraded {upgraded} /World/Looks materials with OmniPBR MDL shaders.")
    return upgraded


# Exact body prim paths and rotation axes in assets/operon_arm.usda (generated from
# public/models/operon_station0.xml by sim/mjcf_to_usd.py).
OPERON_ARM_JOINT_BODIES = [
    ("yaw",         "/World/pedestal/base",                                                                            "Z"),
    ("shoulder",    "/World/pedestal/base/upper_arm",                                                                  "Y"),
    ("elbow",       "/World/pedestal/base/upper_arm/forearm",                                                          "Y"),
    ("wrist_roll",  "/World/pedestal/base/upper_arm/forearm/wrist_roll_link",                                          "Z"),
    ("wrist_pitch", "/World/pedestal/base/upper_arm/forearm/wrist_roll_link/wrist_pitch_link",                         "Y"),
    ("tool_roll",   "/World/pedestal/base/upper_arm/forearm/wrist_roll_link/wrist_pitch_link/tool_roll_link",          "Z"),
]
OPERON_HAND_PATH = "/World/pedestal/base/upper_arm/forearm/wrist_roll_link/wrist_pitch_link/tool_roll_link/hand"


def _axis_angle_quatd(axis: str, angle_rad: float):
    """Double-precision Gf.Quatd for a rotation of angle_rad about X, Y, or Z."""
    from pxr import Gf  # type: ignore

    half = 0.5 * float(angle_rad)
    c, s = math.cos(half), math.sin(half)
    if axis == "X":
        return Gf.Quatd(c, Gf.Vec3d(s, 0.0, 0.0))
    if axis == "Y":
        return Gf.Quatd(c, Gf.Vec3d(0.0, s, 0.0))
    return Gf.Quatd(c, Gf.Vec3d(0.0, 0.0, s))


def _set_double_orient(prim, quatd) -> None:
    from pxr import UsdGeom  # type: ignore

    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    if "xformOp:orient" in ops:
        ops["xformOp:orient"].Set(quatd)
    else:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quatd)


def _set_double_translate(prim, vec3) -> None:
    from pxr import Gf, UsdGeom  # type: ignore

    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    v = Gf.Vec3d(float(vec3[0]), float(vec3[1]), float(vec3[2]))
    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(v)
    else:
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(v)


def articulate_operon_stage(
    stage,
    q6_rad,
    closed: bool = False,
    jaw_travel: float | None = None,
    cell_pos=None,
    cell_quat=None,
    valve_rad: float | None = None,
) -> None:
    """
    Articulate the verified MJCF-derived stage (`assets/operon_arm.usda`).

    Updates both:
      1. The hierarchical body Xform ops (`xformOp:orient` / `xformOp:translate`
         in float64) so every gprim and sensor site (`forearm_imu`, `tcp`)
         composes to the exact MuJoCo forward-kinematics pose (0.000 nm error).
      2. The `UsdPhysics.DriveAPI` targetPosition attributes on `/World/Joints/*`
         so PhysX articulation drives track the same configuration.
    """
    from pxr import Gf  # type: ignore

    q = [float(x) for x in q6_rad[:6]] if len(q6_rad) >= 6 else [
        float(q6_rad[0]) if len(q6_rad) > 0 else 0.0,
        float(q6_rad[1]) if len(q6_rad) > 1 else 0.0,
        float(q6_rad[2]) if len(q6_rad) > 2 else 0.0,
        0.0,
        float(q6_rad[3]) if len(q6_rad) > 3 else 0.0,
        0.0,
    ]

    for idx, (jname, bpath, axis) in enumerate(OPERON_ARM_JOINT_BODIES):
        bprim = stage.GetPrimAtPath(bpath)
        if bprim and bprim.IsValid():
            _set_double_orient(bprim, _axis_angle_quatd(axis, q[idx]))
        jprim = stage.GetPrimAtPath(f"/World/Joints/{jname}")
        if jprim and jprim.IsValid():
            t_attr = jprim.GetAttribute("drive:angular:physics:targetPosition")
            if t_attr and t_attr.IsValid():
                t_attr.Set(float(math.degrees(q[idx])))

    travel = float(jaw_travel) if jaw_travel is not None else (0.0 if closed else 0.022)
    travel = max(0.0, min(0.022, travel))
    fl_prim = stage.GetPrimAtPath(f"{OPERON_HAND_PATH}/finger_l")
    if fl_prim and fl_prim.IsValid():
        _set_double_translate(fl_prim, (-0.009 - travel, 0.0, 0.0))
    fr_prim = stage.GetPrimAtPath(f"{OPERON_HAND_PATH}/finger_r")
    if fr_prim and fr_prim.IsValid():
        _set_double_translate(fr_prim, (0.009 + travel, 0.0, 0.0))

    for jname, target_m in [("finger_l", -travel), ("finger_r", travel)]:
        jprim = stage.GetPrimAtPath(f"/World/Joints/{jname}")
        if jprim and jprim.IsValid():
            t_attr = jprim.GetAttribute("drive:linear:physics:targetPosition")
            if t_attr and t_attr.IsValid():
                t_attr.Set(float(target_m))

    if cell_pos is not None and len(cell_pos) >= 3:
        cprim = stage.GetPrimAtPath("/World/cell")
        if cprim and cprim.IsValid():
            _set_double_translate(cprim, cell_pos)
            if cell_quat is not None and len(cell_quat) >= 4:
                _set_double_orient(
                    cprim,
                    Gf.Quatd(
                        float(cell_quat[0]),
                        Gf.Vec3d(float(cell_quat[1]), float(cell_quat[2]), float(cell_quat[3])),
                    ),
                )

    if valve_rad is not None:
        vprim = stage.GetPrimAtPath("/World/valve")
        if vprim and vprim.IsValid():
            _set_double_orient(vprim, _axis_angle_quatd("X", float(valve_rad)))


def apply_semantic_label(prim, class_name: str) -> None:
    """Attach a Semantic Class label to an OpenUSD prim so Replicator semantic_segmentation AOV works."""
    try:
        from pxr import Sdf, Semantics  # type: ignore

        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr().Set("class")
        sem.CreateSemanticDataAttr().Set(class_name)
    except Exception:
        try:
            from pxr import Sdf

            prim.CreateAttribute("semantic:Semantics:params:semanticType", Sdf.ValueTypeNames.String).Set("class")
            prim.CreateAttribute("semantic:Semantics:params:semanticData", Sdf.ValueTypeNames.String).Set(class_name)
        except Exception:
            pass


def create_pbr_material(
    stage,
    mat_path: str,
    diffuse: tuple[float, float, float],
    metallic: float = 0.0,
    roughness: float = 0.35,
    clearcoat: float = 0.0,
    clearcoat_roughness: float = 0.12,
    specular: float = 0.5,
    ior: float = 1.5,
    emissive: tuple[float, float, float] | None = None,
    opacity: float = 1.0,
):
    """Author a UsdShade.Material + UsdPreviewSurface shader in /World/Looks and return (Material, Shader)."""
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/PBRShader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(float(clearcoat))
    shader.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(float(clearcoat_roughness))
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(specular, specular, specular))
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(float(ior))
    if emissive is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    if opacity < 1.0:
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))

    surface_out = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(surface_out)
    return mat, shader


def bind_material(prim_or_schema, mat) -> None:
    """Bind a UsdShade.Material to a prim or UsdGeom schema using UsdShade.MaterialBindingAPI."""
    from pxr import UsdShade

    prim = prim_or_schema.GetPrim() if hasattr(prim_or_schema, "GetPrim") else prim_or_schema
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def set_xform_pose(prim_or_schema, pos_xyz, quat_wxyz=None, scale_xyz=None) -> None:
    """Set translate, orient (quatf w,x,y,z), and scale on an Xformable prim."""
    from pxr import Gf, UsdGeom

    prim = prim_or_schema.GetPrim() if hasattr(prim_or_schema, "GetPrim") else prim_or_schema
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}

    if "xformOp:translate" not in ops:
        t_op = xf.AddTranslateOp()
    else:
        t_op = ops["xformOp:translate"]
    t_op.Set(Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))

    if quat_wxyz is not None:
        if "xformOp:orient" not in ops:
            o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
        else:
            o_op = ops["xformOp:orient"]
        o_op.Set(
            Gf.Quatf(
                float(quat_wxyz[0]),
                Gf.Vec3f(float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])),
            )
        )

    if scale_xyz is not None:
        if "xformOp:scale" not in ops:
            s_op = xf.AddScaleOp()
        else:
            s_op = ops["xformOp:scale"]
        s_op.Set(Gf.Vec3f(float(scale_xyz[0]), float(scale_xyz[1]), float(scale_xyz[2])))


def quat_from_z_axis(z_dir, np) -> tuple[float, float, float, float]:
    """Return quaternion (w, x, y, z) rotating local +Z (0, 0, 1) onto unit vector z_dir."""
    v = np.asarray(z_dir, dtype=float)
    norm = float(np.linalg.norm(v))
    if norm < 1e-7:
        return (1.0, 0.0, 0.0, 0.0)
    v = v / norm
    if v[2] > 0.99999:
        return (1.0, 0.0, 0.0, 0.0)
    if v[2] < -0.99999:
        return (0.0, 1.0, 0.0, 0.0)
    ax, ay, az = -v[1], v[0], 0.0
    s = math.sqrt((1.0 + v[2]) * 2.0)
    inv_s = 1.0 / s
    return (0.5 * s, ax * inv_s, ay * inv_s, az * inv_s)


def quat_from_rot_matrix(R, np) -> tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix R into quaternion (w, x, y, z)."""
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = float(R[2, 1] - R[1, 2]) / s
        y = float(R[0, 2] - R[2, 0]) / s
        z = float(R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
        w = float(R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = float(R[0, 1] + R[1, 0]) / s
        z = float(R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
        w = float(R[0, 2] - R[2, 0]) / s
        x = float(R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = float(R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + float(R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
        w = float(R[1, 0] - R[0, 1]) / s
        x = float(R[0, 2] + R[2, 0]) / s
        y = float(R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / n, x / n, y / n, z / n)


def build_default_operon_stage(stage, usd_path: str | None = None):
    """Build or augment the Operon 6-DOF Sim-to-Real USD stage with /World/Looks PBR materials and 3-point RTX lighting."""
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/Looks"))

    # ── 1. /World/Looks PBR Material Library (UsdShade.Material + UsdPreviewSurface) ──
    mat_bench, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_ESD_OpticalBench",
        diffuse=(0.06, 0.075, 0.11),
        metallic=0.25,
        roughness=0.22,
        clearcoat=0.45,
        clearcoat_roughness=0.18,
    )
    mat_ideal, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_IdealArm_AnodizedViolet",
        diffuse=(0.55, 0.36, 0.96),
        metallic=0.78,
        roughness=0.24,
        clearcoat=0.35,
        clearcoat_roughness=0.12,
    )
    mat_real, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_RealArm_AnodizedCobalt",
        diffuse=(0.23, 0.51, 0.96),
        metallic=0.82,
        roughness=0.22,
        clearcoat=0.40,
        clearcoat_roughness=0.10,
    )
    mat_steel, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_BrushedSteel_Joints",
        diffuse=(0.72, 0.76, 0.82),
        metallic=0.92,
        roughness=0.18,
        clearcoat=0.50,
    )
    mat_gripper, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_CarbonGripper",
        diffuse=(0.10, 0.12, 0.16),
        metallic=0.35,
        roughness=0.38,
        clearcoat=0.25,
    )
    mat_cell, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_BatteryCell_21700",
        diffuse=(0.14, 0.48, 0.96),
        metallic=0.68,
        roughness=0.16,
        clearcoat=0.85,
        clearcoat_roughness=0.08,
    )
    mat_nickel, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_BrushedNickel_Terminal",
        diffuse=(0.82, 0.85, 0.90),
        metallic=0.95,
        roughness=0.14,
    )
    mat_dock, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_ChargerDock",
        diffuse=(0.16, 0.20, 0.28),
        metallic=0.62,
        roughness=0.26,
        clearcoat=0.55,
        emissive=(0.04, 0.14, 0.32),
    )
    mat_valve, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_ValveWheel",
        diffuse=(0.62, 0.42, 0.98),
        metallic=0.74,
        roughness=0.20,
        clearcoat=0.65,
        emissive=(0.10, 0.05, 0.22),
    )
    mat_watch, _ = create_pbr_material(
        stage,
        "/World/Looks/Mat_WatchIMU",
        diffuse=(0.12, 0.68, 0.98),
        metallic=0.45,
        roughness=0.15,
        clearcoat=0.80,
        emissive=(0.08, 0.36, 0.68),
    )

    # ── 2. 3-Point Studio Lighting Rig (DomeLight + Key DistantLight + Fill/Rim RectLights) ──
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
    dome.CreateIntensityAttr(950.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.84, 0.96))

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
    key.CreateIntensityAttr(2800.0)
    key.CreateAngleAttr(1.2)
    key.CreateColorAttr(Gf.Vec3f(0.98, 0.98, 1.0))
    UsdGeom.XformCommonAPI(key).SetRotate(Gf.Vec3f(-45.0, 35.0, 0.0))

    fill = UsdLux.RectLight.Define(stage, Sdf.Path("/World/FillRectLight"))
    fill.CreateIntensityAttr(650.0)
    fill.CreateWidthAttr(0.55)
    fill.CreateHeightAttr(0.45)
    fill.CreateColorAttr(Gf.Vec3f(0.52, 0.74, 1.0))
    fill_api = UsdGeom.XformCommonAPI(fill)
    fill_api.SetTranslate(Gf.Vec3d(0.42, 0.38, 0.46))
    fill_api.SetRotate(Gf.Vec3f(-35.0, -40.0, 0.0))

    rim = UsdLux.RectLight.Define(stage, Sdf.Path("/World/RimRectLight"))
    rim.CreateIntensityAttr(820.0)
    rim.CreateWidthAttr(0.50)
    rim.CreateHeightAttr(0.40)
    rim.CreateColorAttr(Gf.Vec3f(0.68, 0.48, 1.0))
    rim_api = UsdGeom.XformCommonAPI(rim)
    rim_api.SetTranslate(Gf.Vec3d(-0.35, -0.32, 0.48))
    rim_api.SetRotate(Gf.Vec3f(-40.0, 135.0, 0.0))

    # ── 3. ESD Optical Workbench Table & Beveled Calibration Reticles ──
    bench = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Bench"))
    bench.CreateSizeAttr(1.0)
    b_api = UsdGeom.XformCommonAPI(bench)
    b_api.SetTranslate(Gf.Vec3d(0.08, 0.0, -0.02))
    b_api.SetScale(Gf.Vec3f(0.62, 0.72, 0.04))
    bench.CreateDisplayColorAttr([Gf.Vec3f(0.06, 0.075, 0.11)])
    bind_material(bench, mat_bench)
    apply_semantic_label(bench.GetPrim(), "workbench_surface")
    UsdPhysics.CollisionAPI.Apply(bench.GetPrim())

    if usd_path and Path(usd_path).exists():
        from isaacsim.core.utils.stage import add_reference_to_stage  # Isaac Sim 6.x

        add_reference_to_stage(usd_path=str(Path(usd_path).resolve()), prim_path="/World/AbelArm")

    # ── 4. Forearm Watch IMU Sensor Hierarchy (/World/AbelArm/forearm_link/watch_site) ──
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/AbelArm"))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/AbelArm/forearm_link"))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/AbelArm/forearm_link/watch_site"))
    strap = UsdGeom.Capsule.Define(stage, Sdf.Path("/World/AbelArm/forearm_link/watch_site/WatchBody"))
    strap.CreateRadiusAttr(0.015)
    strap.CreateHeightAttr(0.022)
    strap.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.68, 0.98)])
    bind_material(strap, mat_watch)
    apply_semantic_label(strap.GetPrim(), "forearm_watch_imu")

    # ── 5. Workbench Manipulation Targets (21700 Li-Ion Cell + Terminal, Charger Dock, Valve Handwheel) ──
    cell_xform = UsdGeom.Xform.Define(stage, Sdf.Path("/World/BatteryCell"))
    UsdGeom.XformCommonAPI(cell_xform).SetTranslate(Gf.Vec3d(0.150, -0.075, 0.026))
    apply_semantic_label(cell_xform.GetPrim(), "battery_cell")

    cell_body = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/BatteryCell/CellSleeve"))
    cell_body.CreateRadiusAttr(0.012)
    cell_body.CreateHeightAttr(0.048)
    cell_body.CreateDisplayColorAttr([Gf.Vec3f(0.14, 0.48, 0.96)])
    bind_material(cell_body, mat_cell)
    apply_semantic_label(cell_body.GetPrim(), "battery_cell")

    cell_cap = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/BatteryCell/CathodeTerminal"))
    cell_cap.CreateRadiusAttr(0.0065)
    cell_cap.CreateHeightAttr(0.005)
    UsdGeom.XformCommonAPI(cell_cap).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0265))
    cell_cap.CreateDisplayColorAttr([Gf.Vec3f(0.82, 0.85, 0.90)])
    bind_material(cell_cap, mat_nickel)
    apply_semantic_label(cell_cap.GetPrim(), "battery_cell")

    # Charger Dock Station (Station 2)
    dock_xform = UsdGeom.Xform.Define(stage, Sdf.Path("/World/ChargerDock"))
    dock_api = UsdGeom.XformCommonAPI(dock_xform)
    dock_api.SetTranslate(Gf.Vec3d(0.10, 0.13, 0.085))
    dock_api.SetRotate(Gf.Vec3f(25.0, 0.0, 0.0))
    apply_semantic_label(dock_xform.GetPrim(), "charger_dock")

    dock_panel = UsdGeom.Cube.Define(stage, Sdf.Path("/World/ChargerDock/Housing"))
    dock_panel.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(dock_panel).SetScale(Gf.Vec3f(0.078, 0.022, 0.088))
    dock_panel.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.20, 0.28)])
    bind_material(dock_panel, mat_dock)
    apply_semantic_label(dock_panel.GetPrim(), "charger_dock")

    # Process Valve Wheel Station (Station 3)
    valve_xform = UsdGeom.Xform.Define(stage, Sdf.Path("/World/ValveWheel"))
    UsdGeom.XformCommonAPI(valve_xform).SetTranslate(Gf.Vec3d(0.185, 0.0, 0.14))
    apply_semantic_label(valve_xform.GetPrim(), "valve_wheel")

    valve_rim = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/ValveWheel/Handwheel"))
    valve_rim.CreateRadiusAttr(0.032)
    valve_rim.CreateHeightAttr(0.008)
    UsdGeom.XformCommonAPI(valve_rim).SetRotate(Gf.Vec3f(0.0, 90.0, 0.0))
    valve_rim.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.42, 0.98)])
    bind_material(valve_rim, mat_valve)
    apply_semantic_label(valve_rim.GetPrim(), "valve_wheel")

    valve_post = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/ValveWheel/SupportPost"))
    valve_post.CreateRadiusAttr(0.007)
    valve_post.CreateHeightAttr(0.14)
    UsdGeom.XformCommonAPI(valve_post).SetTranslate(Gf.Vec3d(0.018, 0.0, -0.07))
    bind_material(valve_post, mat_steel)
    apply_semantic_label(valve_post.GetPrim(), "valve_wheel")

    # ── 6. Dual Articulated 6-DOF Arms: IdealArm (Left, Anodized Violet) & RealArm (Right, Anodized Cobalt) ──
    arm_specs = [
        ("IdealArm", -0.115, mat_ideal, (0.55, 0.36, 0.96), "robot_arm_4dof"),
        ("RealArm", 0.115, mat_real, (0.23, 0.51, 0.96), "robot_arm_6dof"),
    ]
    for side, y_off, link_mat, color, sem_cls in arm_specs:
        root_p = f"/World/{side}"
        root_xf = UsdGeom.Xform.Define(stage, Sdf.Path(root_p))
        set_xform_pose(root_xf, (0.0, y_off, 0.0))
        apply_semantic_label(root_xf.GetPrim(), sem_cls)

        # CNC Pedestal Base Plinth
        plinth = UsdGeom.Cylinder.Define(stage, Sdf.Path(f"{root_p}/base_plinth"))
        plinth.CreateRadiusAttr(0.044)
        plinth.CreateHeightAttr(0.014)
        set_xform_pose(plinth, (0.0, 0.0, 0.007))
        bind_material(plinth, mat_gripper)
        apply_semantic_label(plinth.GetPrim(), sem_cls)

        # 4 Tubular Links (Pedestal->Shoulder, UpperArm, Forearm, Wrist->Flange)
        link_radii = [0.021, 0.016, 0.0135, 0.011]
        for j in range(4):
            seg = UsdGeom.Capsule.Define(stage, Sdf.Path(f"{root_p}/seg_{j}"))
            seg.CreateRadiusAttr(link_radii[j])
            seg.CreateHeightAttr(0.080)
            seg.CreateAxisAttr(UsdGeom.Tokens.z)
            seg.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            bind_material(seg, link_mat)
            apply_semantic_label(seg.GetPrim(), sem_cls)

        # 6 Brushed Steel Harmonic-Drive Joint Housings + Anodized End-Caps
        for j in range(6):
            big = j < 3
            j_cyl = UsdGeom.Cylinder.Define(stage, Sdf.Path(f"{root_p}/joint_{j}"))
            j_cyl.CreateRadiusAttr(0.021 if big else 0.0155)
            j_cyl.CreateHeightAttr(0.042 if big else 0.032)
            j_cyl.CreateAxisAttr(UsdGeom.Tokens.z)
            j_cyl.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.76, 0.82)])
            bind_material(j_cyl, mat_steel)
            apply_semantic_label(j_cyl.GetPrim(), sem_cls)

        # Robotiq 2F-85 Parallel-Jaw Carbon-Fiber Gripper Housing + Left/Right Fingers
        grip_xf = UsdGeom.Xform.Define(stage, Sdf.Path(f"{root_p}/gripper"))
        apply_semantic_label(grip_xf.GetPrim(), sem_cls)

        palm = UsdGeom.Cube.Define(stage, Sdf.Path(f"{root_p}/gripper/palm"))
        palm.CreateSizeAttr(1.0)
        set_xform_pose(palm, (0.0, 0.0, -0.018), scale_xyz=(0.064, 0.026, 0.024))
        palm.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.12, 0.16)])
        bind_material(palm, mat_gripper)
        apply_semantic_label(palm.GetPrim(), sem_cls)

        for f_name, s_dir in [("finger_left", -1.0), ("finger_right", 1.0)]:
            jaw = UsdGeom.Cube.Define(stage, Sdf.Path(f"{root_p}/gripper/{f_name}"))
            jaw.CreateSizeAttr(1.0)
            set_xform_pose(jaw, (s_dir * 0.022, 0.0, 0.006), scale_xyz=(0.008, 0.018, 0.032))
            jaw.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.76, 0.82)])
            bind_material(jaw, mat_steel)
            apply_semantic_label(jaw.GetPrim(), sem_cls)


def update_articulated_arm_usd(stage, side: str, y_off: float, q_rad, closed: bool, np) -> dict:
    """Articulate all 6-DOF links, joint housings, and parallel gripper fingers in USD for IdealArm or RealArm."""
    fk = fk6_chain(q_rad, np)
    pts = fk["pts"]
    root_p = f"/World/{side}"

    # 1. Articulate the 4 tubular link segments between consecutive kinematic points
    for j in range(4):
        seg_prim = stage.GetPrimAtPath(f"{root_p}/seg_{j}")
        if seg_prim.IsValid():
            p_a = pts[j]
            p_b = pts[j + 1]
            mid = 0.5 * (p_a + p_b)
            vec = p_b - p_a
            dist = float(np.linalg.norm(vec))
            quat = quat_from_z_axis(vec, np)
            z_scale = max(0.25, dist / 0.095)
            set_xform_pose(seg_prim, mid, quat_wxyz=quat, scale_xyz=(1.0, 1.0, z_scale))

    # 2. Articulate the 6 brushed-steel joint actuator housings
    for j, (j_pos, j_axis) in enumerate(fk["joints"]):
        j_prim = stage.GetPrimAtPath(f"{root_p}/joint_{j}")
        if j_prim.IsValid():
            quat = quat_from_z_axis(j_axis, np)
            set_xform_pose(j_prim, j_pos, quat_wxyz=quat)

    # 3. Articulate the parallel-jaw gripper and finger opening width
    grip_prim = stage.GetPrimAtPath(f"{root_p}/gripper")
    if grip_prim.IsValid():
        q_tip = quat_from_rot_matrix(fk["R_tip"], np)
        set_xform_pose(grip_prim, fk["tip"], quat_wxyz=q_tip)

    open_ratio = 0.0 if closed else 1.0
    jaw_offset = 0.008 + 0.018 * open_ratio
    for f_name, s_dir in [("finger_left", -1.0), ("finger_right", 1.0)]:
        f_prim = stage.GetPrimAtPath(f"{root_p}/gripper/{f_name}")
        if f_prim.IsValid():
            set_xform_pose(f_prim, (s_dir * jaw_offset, 0.0, 0.006), scale_xyz=(0.008, 0.018, 0.032))

    return fk


def update_domain_randomization_usd(stage, state: dict) -> None:
    """Live-bind Operon Studio's Domain Randomization sliders (light, angle, rough) to UsdLux and UsdShade.

    Targets the physical lighting rig at `/World/Lighting/{Key,Dome,Fill,Rim}`
    created by `add_scene_lighting()` and the dual-shader materials at
    `/World/Looks/{Steel,Carbon,Rubber,Copper,CellGold,PadBlue,PadGreen,Bench}`
    (`Shader` UsdPreviewSurface + `MDLShader` OmniPBR).
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

    light_mult = float(state.get("light", 2.8))
    angle_deg = float(state.get("angle", 35.0))
    rough = max(0.04, min(0.95, float(state.get("rough", 0.22))))

    for key_path in ("/World/Lighting/Key", "/World/KeyLight"):
        key_prim = stage.GetPrimAtPath(key_path)
        if key_prim.IsValid():
            if key_prim.IsA(UsdLux.RectLight):
                UsdLux.RectLight(key_prim).GetIntensityAttr().Set(float(light_mult * 285.0))
            elif key_prim.IsA(UsdLux.DistantLight):
                UsdLux.DistantLight(key_prim).GetIntensityAttr().Set(float(light_mult * 1000.0))
            UsdGeom.XformCommonAPI(key_prim).SetRotate(Gf.Vec3f(-42.0, angle_deg, 0.0))

    for dome_path in ("/World/Lighting/Dome", "/World/DomeLight"):
        dome_prim = stage.GetPrimAtPath(dome_path)
        if dome_prim.IsValid():
            UsdLux.DomeLight(dome_prim).GetIntensityAttr().Set(float(280.0 + light_mult * 150.0))

    for fill_path in ("/World/Lighting/Fill", "/World/FillRectLight"):
        fill_prim = stage.GetPrimAtPath(fill_path)
        if fill_prim.IsValid():
            UsdLux.RectLight(fill_prim).GetIntensityAttr().Set(float(180.0 + light_mult * 115.0))

    for rim_path in ("/World/Lighting/Rim", "/World/RimRectLight"):
        rim_prim = stage.GetPrimAtPath(rim_path)
        if rim_prim.IsValid():
            UsdLux.RectLight(rim_prim).GetIntensityAttr().Set(float(220.0 + light_mult * 135.0))

    # Material roughness scaling relative to studio base roughness (default 0.22)
    scale = rough / 0.22
    look_roughness = [
        ("Steel", max(0.04, min(0.95, 0.26 * scale))),
        ("Carbon", max(0.04, min(0.95, 0.34 * scale))),
        ("Rubber", max(0.20, min(0.98, 0.85 * (0.7 + 0.3 * scale)))),
        ("Copper", max(0.04, min(0.90, 0.22 * scale))),
        ("CellGold", max(0.04, min(0.90, 0.22 * scale))),
        ("PadBlue", max(0.08, min(0.95, 0.45 * scale))),
        ("PadGreen", max(0.08, min(0.95, 0.45 * scale))),
        ("Bench", max(0.15, min(0.98, 0.78 * (0.65 + 0.35 * scale)))),
    ]
    for mat_name, r_val in look_roughness:
        # 1. UsdPreviewSurface shader
        sh_prim = stage.GetPrimAtPath(f"/World/Looks/{mat_name}/Shader")
        if sh_prim.IsValid():
            sh = UsdShade.Shader(sh_prim)
            r_in = sh.GetInput("roughness") or sh.CreateInput("roughness", Sdf.ValueTypeNames.Float)
            r_in.Set(float(r_val))
        # 2. OmniPBR MDL shader
        mdl_prim = stage.GetPrimAtPath(f"/World/Looks/{mat_name}/MDLShader")
        if mdl_prim.IsValid():
            mdl = UsdShade.Shader(mdl_prim)
            mr_in = mdl.GetInput("reflection_roughness_constant") or mdl.CreateInput(
                "reflection_roughness_constant", Sdf.ValueTypeNames.Float
            )
            mr_in.Set(float(r_val))


def write_sdg_bundle(
    out_dir: Path,
    frame_id: str,
    state: dict,
    aov_buffers: dict,
    np,
) -> dict:
    """Write a Replicator Synthetic Data Generation (SDG) multi-AOV bundle to disk.

    Produces:
      - `<frame_id>_rgb.png` (or `.npy` fallback if PIL unavailable)
      - `<frame_id>_depth.npy` (metric float32 camera distance in meters)
      - `<frame_id>_normals.npy` (float32 unit surface normals)
      - `<frame_id>_semantic.png` (colorized semantic segmentation mask)
      - `<frame_id>_meta.json` (joint angles, domain randomization, semantics, camera pose)
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    files_written = []

    try:
        from PIL import Image
    except ImportError:
        Image = None

    # 1. RGB beauty pass
    rgb_raw = aov_buffers.get("rtx")
    rgb_u8 = colorize_aov_buffer("rtx", rgb_raw, np)
    if rgb_u8 is not None:
        if Image is not None:
            rgb_path = out_dir / f"{frame_id}_rgb.png"
            Image.fromarray(rgb_u8).save(rgb_path, format="PNG")
        else:
            rgb_path = out_dir / f"{frame_id}_rgb.npy"
            np.save(rgb_path, rgb_u8)
        files_written.append(rgb_path.name)

    # 2. Metric depth pass (float32 meters)
    depth_raw = aov_buffers.get("depth")
    if depth_raw is not None:
        d_arr = np.asarray(depth_raw, dtype=np.float32)
        depth_path = out_dir / f"{frame_id}_depth.npy"
        np.save(depth_path, d_arr)
        files_written.append(depth_path.name)

    # 3. Surface normals pass (float32 XYZ)
    norm_raw = aov_buffers.get("normal")
    if norm_raw is not None:
        n_arr = np.asarray(norm_raw, dtype=np.float32)
        norm_path = out_dir / f"{frame_id}_normals.npy"
        np.save(norm_path, n_arr)
        files_written.append(norm_path.name)

    # 4. Semantic segmentation pass
    sem_raw = aov_buffers.get("semantic")
    sem_u8 = colorize_aov_buffer("semantic", sem_raw, np)
    if sem_u8 is not None:
        if Image is not None:
            sem_path = out_dir / f"{frame_id}_semantic.png"
            Image.fromarray(sem_u8).save(sem_path, format="PNG")
        else:
            sem_path = out_dir / f"{frame_id}_semantic.npy"
            np.save(sem_path, sem_u8)
        files_written.append(sem_path.name)

    # 5. Structured metadata JSON
    meta = {
        "schema": "operon.replicator.sdg.v1",
        "frame_id": frame_id,
        "timestamp": time.time(),
        "render_mode": state.get("render_mode", "path"),
        "rt_subframes": int(state.get("rt_subframes", 32)),
        "q6_rad": [float(x) for x in state.get("q6", [0.0] * 6)],
        "q4_rad": [float(x) for x in state.get("q4", [0.0] * 4)],
        "gripper_closed": bool(state.get("closed", False)),
        "jaw_travel_m": float(state.get("jaw_travel", 0.0)),
        "domain_randomization": {
            "light_ev": float(state.get("light", 2.8)),
            "key_angle_deg": float(state.get("angle", 35.0)),
            "roughness": float(state.get("rough", 0.22)),
        },
        "semantic_classes": {
            "robot": ["/World/pedestal"],
            "workpiece": ["/World/cell"],
            "fixture": ["/World/pick_pad", "/World/place_pad", "/World/dock", "/World/valve_base"],
            "ground": ["/World/bench"],
        },
        "files": files_written,
    }
    meta_path = out_dir / f"{frame_id}_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    files_written.append(meta_path.name)

    return {
        "frame_id": frame_id,
        "local_dir": str(out_dir),
        "files": files_written,
        "meta": meta,
    }


def upload_sdg_to_gcs(local_dir: Path, frame_id: str, files: list[str], bucket_name: str, prefix: str = "operon-sdg") -> str:
    """Upload an SDG frame bundle to `gs://<bucket_name>/<prefix>/<frame_id>/`."""
    if not bucket_name:
        return ""
    bucket_clean = bucket_name.replace("gs://", "").strip("/")
    dest_uri = f"gs://{bucket_clean}/{prefix.strip('/')}/{frame_id}"

    # Try google-cloud-storage Python client first, then fall back to gcloud storage cp
    try:
        from google.cloud import storage  # type: ignore

        client = storage.Client()
        bucket = client.bucket(bucket_clean)
        for fname in files:
            fpath = Path(local_dir) / fname
            if fpath.exists():
                blob = bucket.blob(f"{prefix.strip('/')}/{frame_id}/{fname}")
                blob.upload_from_filename(str(fpath))
        return dest_uri
    except Exception:
        pass

    try:
        src_paths = [str(Path(local_dir) / f) for f in files if (Path(local_dir) / f).exists()]
        if src_paths:
            subprocess.check_call(
                ["gcloud", "storage", "cp", *src_paths, f"{dest_uri}/"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return dest_uri
    except Exception as exc:
        print(f"[shadow-rtx] GCS upload warning ({dest_uri}): {exc}", file=sys.stderr)
    return ""


def colorize_aov_buffer(aov: str, data, np):
    """Convert Replicator AOV numpy array (RGB, depth, normals, semantic) into an uint8 RGB image."""
    if data is None:
        return None
    arr = np.asarray(data)
    if arr.size == 0:
        return None
    if aov == "depth":
        d = np.nan_to_num(arr.squeeze(), nan=2.0, posinf=2.0, neginf=0.0)
        norm = np.clip((1.2 - d) / 1.0, 0.0, 1.0)
        u8 = (norm * 255).astype(np.uint8)
        return np.stack([u8, (u8 * 0.85).astype(np.uint8), np.clip(u8 * 1.15, 0, 255).astype(np.uint8)], axis=-1)
    if aov == "normal":
        n = arr[..., :3]
        u8 = np.clip((n * 0.5 + 0.5) * 255.0, 0, 255).astype(np.uint8)
        return u8
    if aov in ("semantic", "instance"):
        if arr.ndim == 3 and arr.shape[-1] >= 3:
            return arr[..., :3].astype(np.uint8)
        ids = arr.squeeze().astype(np.int32)
        r = ((ids * 97) % 255).astype(np.uint8)
        g = ((ids * 57 + 80) % 255).astype(np.uint8)
        b = ((ids * 151 + 140) % 255).astype(np.uint8)
        return np.stack([r, g, b], axis=-1)
    if arr.ndim == 3 and arr.shape[-1] >= 3:
        return arr[..., :3].astype(np.uint8)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Shadow PC Live Omniverse RTX + PhysX Bridge for Operon Studio")
    parser.add_argument("--server", type=str, default="wss://operon-421717890494.us-central1.run.app",
                        help="Operon Cloud Run WebSocket base URL (wss://... or ws://localhost:8080)")
    parser.add_argument("--room", type=str, default="OPERON", help="Operon Studio room code (default: OPERON)")
    parser.add_argument("--usd", type=str, default=None,
                        help="Path to the USD stage (default: assets/operon_arm.usda, "
                             "generated and verified by sim/mjcf_to_usd.py)")
    parser.add_argument("--width", type=int, default=960, help="RTX render width (default: 960)")
    parser.add_argument("--height", type=int, default=600, help="RTX render height (default: 600)")
    parser.add_argument("--fps", type=float, default=20.0, help="Target JPEG stream frame rate (default: 20)")
    parser.add_argument("--jpeg-quality", type=int, default=78, help="JPEG quality 40..95 (default: 78)")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim headless without desktop GUI window")
    parser.add_argument("--render-mode", choices=["rt", "path"], default="rt",
                        help="rt = RayTracedLighting for the live 20 fps overlay; "
                             "path = PathTracing for offline synthetic-data capture")
    parser.add_argument("--rt-subframes", type=int, default=32,
                        help="Accumulation subframes per frame in --render-mode path")
    parser.add_argument("--sdg-dir", type=str, default="/tmp/operon-sdg",
                        help="Local output directory for Replicator SDG frame bundles")
    parser.add_argument("--gcs-bucket", type=str,
                        default=os.environ.get("OPERON_GCS_BUCKET", "run-sources-gen-lang-client-0221638641-us-central1"),
                        help="GCS bucket for uploading SDG frame bundles (empty disables upload)")
    parser.add_argument("--idle-exit-minutes", type=float, default=0.0,
                        help="Exit when no studio deck has been in the room for this long. "
                             "0 disables. REQUIRED on a cloud GPU: SkyPilot measures idleness "
                             "as 'no running job', so a bridge that never exits means a "
                             "cluster that is never torn down.")
    args = parser.parse_args()

    ensure_websocket_client()

    # No fallback renderer. The previous --local-sim path drew a Pillow
    # stick figure and labelled it "LIVE PYTHON BRIDGE", which is exactly the
    # failure you do not want to discover mid-demo: it looks like it worked.
    # If Isaac is not here, say so and stop.
    try:
        from isaacsim import SimulationApp  # type: ignore
    except ImportError as exc:
        print("[shadow-rtx] FATAL: 'isaacsim' is not importable.", file=sys.stderr)
        print("  This bridge requires Isaac Sim 6.x and an RTX GPU.", file=sys.stderr)
        print("  Run it inside the Isaac container:", file=sys.stderr)
        print("    ./python.sh /operon/sim/shadow_rtx_bridge.py ...", file=sys.stderr)
        print(f"  ({exc})", file=sys.stderr)
        raise SystemExit(2)

    renderer = "PathTracing" if args.render_mode == "path" else "RayTracedLighting"
    sim_app = SimulationApp({
        "headless": args.headless,
        "renderer": renderer,
        "width": args.width,
        "height": args.height,
        "anti_aliasing": 3,
    })
    print(f"[shadow-rtx] Isaac Sim up, renderer={renderer}")

    import numpy as np
    from PIL import Image
    import websockets

    import omni.replicator.core as rep  # type: ignore
    import omni.usd  # type: ignore

    # Isaac Sim 6.x namespaces. The legacy omni.isaac.* aliases are gone;
    # importing them here would mask a genuinely wrong container image.
    from isaacsim.core.api import World  # type: ignore
    from isaacsim.sensors.physics import IMUSensor  # type: ignore

    configure_rtx_render_settings(
        mode="PathTracing" if args.render_mode == "path" else "RayTracedLighting",
        subframes=args.rt_subframes,
    )

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)

    # Load the verified stage rather than building geometry procedurally.
    # assets/operon_arm.usda is generated from the same MJCF that drives the
    # browser physics and is checked against MuJoCo's forward kinematics to
    # 0.000 nm (test/usd/verify_against_mujoco.py), so what gets ray-traced
    # is provably what gets simulated.
    usd_path = args.usd or str(Path(__file__).resolve().parent.parent / "assets" / "operon_arm.usda")
    if not Path(usd_path).exists():
        print(f"[shadow-rtx] FATAL: stage not found: {usd_path}", file=sys.stderr)
        print("  Generate it with:  python sim/mjcf_to_usd.py", file=sys.stderr)
        raise SystemExit(2)
    print(f"[shadow-rtx] Opening stage {usd_path}")
    omni.usd.get_context().open_stage(usd_path)
    stage = omni.usd.get_context().get_stage()

    promote_semantic_attributes(stage)
    add_scene_lighting(stage)
    upgrade_materials_to_mdl(stage)

    # The IMU mounts on the forearm site exported by mjcf_to_usd.py, which is
    # the same site the analytic forearmSiteFK() uses in the studio.
    imu_prim_path = find_site(stage, "forearm_imu")
    if imu_prim_path is None:
        print("[shadow-rtx] FATAL: no forearm_imu site in the stage", file=sys.stderr)
        raise SystemExit(2)
    print(f"[shadow-rtx] IMU site: {imu_prim_path}")
    imu_sensor = IMUSensor(
        prim_path=f"{imu_prim_path}/imu_sensor",
        name="watch_imu",
        frequency=120,
        translation=np.array([0.0, 0.0, 0.0]),
    )

    # Create RTX Camera & Replicator Render Product + AOV Annotators
    cam = rep.create.camera(position=(0.48, -0.34, 0.34), look_at=(0.08, 0.0, 0.08), focal_length=24.0)
    rp = rep.create.render_product(cam, (args.width, args.height))
    annotators = {
        "rtx": rep.AnnotatorRegistry.get_annotator("rgb"),
        "depth": rep.AnnotatorRegistry.get_annotator("distance_to_camera"),
        "normal": rep.AnnotatorRegistry.get_annotator("normals"),
        "semantic": rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": True}),
    }
    for ann in annotators.values():
        ann.attach([rp])

    world.reset()

    state = {
        "q4": [0.0, 0.35, 0.65, 0.0],
        "q6": [0.0, 0.35, 0.65, 0.0, 0.4, 0.0],
        "closed": False,
        "jaw_travel": None,
        "cell_pos": None,
        "cell_quat": None,
        "valve_rad": None,
        "aov": "rtx",
        "render_mode": args.render_mode,
        "rt_subframes": args.rt_subframes,
        "sdg_capture_pending": False,
        "sdg_count": 0,
        "last_sdg_uri": "",
        "light": 2.8,
        "angle": 35.0,
        "rough": 0.22,
    }
    prev_p = None
    prev_v = np.zeros(3)
    prev_R = None

    ws_base = args.server.rstrip("/").replace("https://", "wss://").replace("http://", "ws://")
    ws_url = f"{ws_base}/api/room/{args.room.upper()}/ws?role=isaac&id=shadow-rtx"

    # Idle tracking. This is a COST CONTROL, not a nicety: SkyPilot measures
    # cluster idleness as "no running job", so a bridge that streams forever
    # keeps an L4 billing indefinitely even with autostop configured. The
    # bridge must end its own job when nobody is watching.
    idle_limit = max(0.0, float(args.idle_exit_minutes)) * 60.0
    last_cmd_at = time.time()
    idle_exit = False

    async def bridge_loop() -> None:
        nonlocal prev_p, prev_v, prev_R, idle_exit
        min_interval = 1.0 / max(5.0, min(30.0, args.fps))

        while sim_app.is_running() and not idle_exit:
            try:
                print(f"[shadow-rtx] Connecting to Operon Relay: {ws_url}")
                async with websockets.connect(ws_url, max_size=512 * 1024, ping_interval=20) as ws:
                    print("[shadow-rtx] Connected! Streaming Omniverse RTX + PhysX IMU to Operon Studio...")

                    async def recv_loop() -> None:
                        async for raw in ws:
                            try:
                                msg = json.loads(raw)
                                if msg.get("t") == "isaac_cmd":
                                    nonlocal last_cmd_at
                                    last_cmd_at = time.time()
                                    if isinstance(msg.get("q4"), list) and len(msg["q4"]) >= 4:
                                        state["q4"] = [float(x) for x in msg["q4"][:4]]
                                    if isinstance(msg.get("q6"), list) and len(msg["q6"]) >= 6:
                                        state["q6"] = [float(x) for x in msg["q6"][:6]]
                                    if "closed" in msg:
                                        state["closed"] = bool(msg["closed"])
                                    if "jaw_travel" in msg and msg["jaw_travel"] is not None:
                                        state["jaw_travel"] = float(msg["jaw_travel"])
                                    if isinstance(msg.get("cell_pos"), list) and len(msg["cell_pos"]) == 3:
                                        state["cell_pos"] = [float(x) for x in msg["cell_pos"]]
                                    if isinstance(msg.get("cell_quat"), list) and len(msg["cell_quat"]) == 4:
                                        state["cell_quat"] = [float(x) for x in msg["cell_quat"]]
                                    if "valve_rad" in msg and msg["valve_rad"] is not None:
                                        state["valve_rad"] = float(msg["valve_rad"])
                                    if msg.get("aov") in ("rtx", "depth", "normal", "semantic", "instance"):
                                        state["aov"] = msg["aov"]
                                    if msg.get("render_mode") in ("rt", "path"):
                                        new_rm = msg["render_mode"]
                                        new_sf = int(msg.get("rt_subframes", state["rt_subframes"]))
                                        if new_rm != state["render_mode"] or new_sf != state["rt_subframes"]:
                                            state["render_mode"] = new_rm
                                            state["rt_subframes"] = new_sf
                                            configure_rtx_render_settings(
                                                mode="PathTracing" if new_rm == "path" else "RayTracedLighting",
                                                subframes=new_sf,
                                            )
                                    if msg.get("sdg_capture"):
                                        state["sdg_capture_pending"] = True
                                    if "light" in msg and msg["light"] is not None:
                                        state["light"] = float(msg["light"])
                                    if "angle" in msg and msg["angle"] is not None:
                                        state["angle"] = float(msg["angle"])
                                    if "rough" in msg and msg["rough"] is not None:
                                        state["rough"] = float(msg["rough"])
                            except Exception:
                                pass

                    recv_task = asyncio.create_task(recv_loop())
                    last_send = 0.0
                    dt = 1.0 / 30.0

                    try:
                        while sim_app.is_running():
                            # 1. Articulate the verified `/World/pedestal` 6-DOF robot stage
                            q6_arr = np.array(state["q6"], dtype=float)
                            articulate_operon_stage(
                                stage,
                                q6_arr,
                                closed=state["closed"],
                                jaw_travel=state["jaw_travel"],
                                cell_pos=state["cell_pos"],
                                cell_quat=state["cell_quat"],
                                valve_rad=state["valve_rad"],
                            )

                            # 2. Compute analytical forearm site pose for fallback / cross-check
                            p_site, R_site = forearm_fk_rad(q6_arr, np)

                            # 3. Live-bind Domain Randomization sliders (light, angle, rough) to UsdLux & UsdShade
                            update_domain_randomization_usd(stage, state)

                            # 4. Step PhysX + RTX Renderer (with multi-subframe accumulation when PathTracing SDG capture runs)
                            if state["sdg_capture_pending"] and state["render_mode"] == "path":
                                for _ in range(max(1, min(64, int(state["rt_subframes"])))):
                                    world.step(render=True)
                            else:
                                world.step(render=True)

                            # 5. Read PhysX IMUSensor (with analytical kinematic fallback while sensor buffer warms up)
                            frame = imu_sensor.get_current_frame()
                            lin_acc = frame.get("lin_acc")
                            ang_vel = frame.get("ang_vel")

                            if prev_p is None:
                                prev_p = p_site.copy()
                                prev_R = R_site.copy()
                            vel = (p_site - prev_p) / max(1e-3, dt)
                            acc_world = (vel - prev_v) / max(1e-3, dt)
                            a_proper = R_site.T @ (acc_world + np.array([0.0, 0.0, G0]))
                            Rdot = (R_site - prev_R) / max(1e-3, dt)
                            S = 0.5 * (R_site.T @ Rdot - (R_site.T @ Rdot).T)
                            gyro_body = np.array([S[2, 1], S[0, 2], S[1, 0]])
                            prev_p, prev_v, prev_R = p_site.copy(), vel, R_site.copy()

                            if lin_acc is not None and np.linalg.norm(lin_acc) > 0.1:
                                ax, ay, az = float(lin_acc[0]), float(lin_acc[1]), float(lin_acc[2])
                            else:
                                ax, ay, az = float(a_proper[0]), float(a_proper[1]), float(a_proper[2])

                            if ang_vel is not None and np.linalg.norm(ang_vel) > 1e-4:
                                gx, gy, gz = float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2])
                            else:
                                gx, gy, gz = float(gyro_body[0]), float(gyro_body[1]), float(gyro_body[2])

                            # 6. Optional Replicator SDG multi-AOV capture + GCS upload
                            sdg_info = None
                            if state["sdg_capture_pending"]:
                                state["sdg_capture_pending"] = False
                                state["sdg_count"] += 1
                                fid = f"sdg_{int(time.time())}_{state['sdg_count']:04d}"
                                all_aovs = {}
                                for k, ann in annotators.items():
                                    rb = ann.get_data()
                                    if isinstance(rb, dict):
                                        rb = rb.get("data")
                                    all_aovs[k] = rb
                                bundle = write_sdg_bundle(Path(args.sdg_dir), fid, state, all_aovs, np)
                                gcs_uri = upload_sdg_to_gcs(
                                    Path(args.sdg_dir),
                                    fid,
                                    bundle["files"],
                                    args.gcs_bucket,
                                    prefix=f"operon-sdg/{args.room.upper()}",
                                )
                                state["last_sdg_uri"] = gcs_uri or bundle["local_dir"]
                                sdg_info = {
                                    "frame_id": fid,
                                    "files": bundle["files"],
                                    "uri": state["last_sdg_uri"],
                                }

                            now = time.perf_counter()
                            if now - last_send >= min_interval or sdg_info is not None:
                                dt = now - last_send if last_send > 0 else min_interval
                                last_send = now
                                active_aov = state["aov"]
                                ann_key = "semantic" if active_aov == "instance" else active_aov
                                raw_buf = annotators.get(ann_key, annotators["rtx"]).get_data()
                                if isinstance(raw_buf, dict):
                                    raw_buf = raw_buf.get("data")
                                rgb = colorize_aov_buffer(active_aov, raw_buf, np)
                                jpg_b64 = ""
                                if rgb is not None:
                                    im = Image.fromarray(rgb)
                                    bio = io.BytesIO()
                                    im.save(bio, format="JPEG", quality=args.jpeg_quality)
                                    jpg_b64 = base64.b64encode(bio.getvalue()).decode("ascii")

                                payload = {
                                    "t": "isaac_frame",
                                    "aov": active_aov,
                                    "render_mode": state["render_mode"],
                                    "subframes": state["rt_subframes"],
                                    "jpg": jpg_b64,
                                    "fps": round(1.0 / max(1e-3, dt), 1),
                                    "imu": {
                                        "ax": round(ax, 4),
                                        "ay": round(ay, 4),
                                        "az": round(az, 4),
                                        "gx": round(gx, 4),
                                        "gy": round(gy, 4),
                                        "gz": round(gz, 4),
                                    },
                                }
                                if sdg_info is not None:
                                    payload["sdg"] = sdg_info
                                await ws.send(json.dumps(payload))

                            if idle_limit > 0 and (time.time() - last_cmd_at) > idle_limit:
                                mins = idle_limit / 60.0
                                print(f"[shadow-rtx] No studio deck for {mins:.0f} min "
                                      f"-> exiting so the GPU cluster can auto-tear-down.")
                                idle_exit = True
                                break

                            await asyncio.sleep(0.005)
                    finally:
                        recv_task.cancel()
            except Exception as exc:
                print(f"[shadow-rtx] Reconnecting in 2s ({exc})...")
                await asyncio.sleep(2.0)

    try:
        asyncio.run(bridge_loop())
    finally:
        sim_app.close()
    if idle_exit:
        print("[shadow-rtx] Clean idle exit. Job over; cluster autostop --down may now fire.")


if __name__ == "__main__":
    main()
