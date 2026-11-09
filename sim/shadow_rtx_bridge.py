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


def configure_rtx_render_settings() -> None:
    """Configure Omniverse Kit carb.settings for high-contrast RTX RayTracedLighting on Shadow PC."""
    try:
        import carb.settings  # type: ignore

        settings = carb.settings.get_settings()
        settings.set("/rtx/rendermode", "RayTracedLighting")
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
        print("[shadow-rtx] Configured carb.settings for RTX RayTracedLighting + DLSS + Reflections + GI + AO.")
    except Exception as exc:
        print(f"[shadow-rtx] Warning: carb.settings configuration skipped ({exc}).")


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
        from omni.isaac.core.utils.stage import add_reference_to_stage

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
    """Live-bind Operon Studio's Domain Randomization sliders (light, angle, rough) to UsdLux and UsdShade."""
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

    light_mult = float(state.get("light", 2.8))
    angle_deg = float(state.get("angle", 35.0))
    rough = max(0.04, min(0.95, float(state.get("rough", 0.22))))

    key_prim = stage.GetPrimAtPath("/World/KeyLight")
    if key_prim.IsValid():
        UsdLux.DistantLight(key_prim).GetIntensityAttr().Set(light_mult * 1000.0)
        UsdGeom.XformCommonAPI(key_prim).SetRotate(Gf.Vec3f(-45.0, angle_deg, 0.0))

    dome_prim = stage.GetPrimAtPath("/World/DomeLight")
    if dome_prim.IsValid():
        UsdLux.DomeLight(dome_prim).GetIntensityAttr().Set(350.0 + light_mult * 220.0)

    fill_prim = stage.GetPrimAtPath("/World/FillRectLight")
    if fill_prim.IsValid():
        UsdLux.RectLight(fill_prim).GetIntensityAttr().Set(240.0 + light_mult * 160.0)

    rim_prim = stage.GetPrimAtPath("/World/RimRectLight")
    if rim_prim.IsValid():
        UsdLux.RectLight(rim_prim).GetIntensityAttr().Set(300.0 + light_mult * 190.0)

    shader_targets = [
        ("/World/Looks/Mat_ESD_OpticalBench/PBRShader", min(0.95, rough * 0.85 + 0.06)),
        ("/World/Looks/Mat_IdealArm_AnodizedViolet/PBRShader", rough),
        ("/World/Looks/Mat_RealArm_AnodizedCobalt/PBRShader", rough),
        ("/World/Looks/Mat_BrushedSteel_Joints/PBRShader", max(0.06, rough * 0.75)),
        ("/World/Looks/Mat_CarbonGripper/PBRShader", min(0.95, rough * 1.25)),
    ]
    for sh_path, r_val in shader_targets:
        sh_prim = stage.GetPrimAtPath(sh_path)
        if sh_prim.IsValid():
            sh = UsdShade.Shader(sh_prim)
            r_in = sh.GetInput("roughness")
            if not r_in:
                r_in = sh.CreateInput("roughness", Sdf.ValueTypeNames.Float)
            r_in.Set(float(r_val))


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


def render_local_sim_aov(state: dict, width: int, height: int, np, Image, ImageDraw):
    """Software 3D perspective renderer on macOS/CPU when Isaac Sim (`isaacsim`) is not installed locally."""
    aov = state.get("aov", "rtx")
    light_mult = float(state.get("light", 2.8))
    rough = float(state.get("rough", 0.22))
    q4 = np.array(state.get("q4", [0.0, 0.35, 0.65, 0.0]), dtype=float)
    q6 = np.array(state.get("q6", [0.0, 0.35, 0.65, 0.0, 0.4, 0.0]), dtype=float)
    fk_left = fk6_chain(q4, np)
    fk_right = fk6_chain(q6, np)

    # Background gradient
    y_grad = np.linspace(0.0, 1.0, height, dtype=np.float32)[:, None]
    if aov == "depth":
        bg = np.zeros((height, width, 3), dtype=np.uint8) + np.uint8(28)
    elif aov == "normal":
        bg = np.full((height, width, 3), (128, 128, 255), dtype=np.uint8)
    elif aov in ("semantic", "instance"):
        bg = np.full((height, width, 3), (16, 20, 30), dtype=np.uint8)
    else:
        r_ch = np.clip((14 + (1.0 - y_grad) * 10 * (light_mult / 2.8)), 8, 48).astype(np.uint8)
        g_ch = np.clip((18 + (1.0 - y_grad) * 14 * (light_mult / 2.8)), 10, 56).astype(np.uint8)
        b_ch = np.clip((30 + (1.0 - y_grad) * 22 * (light_mult / 2.8)), 16, 78).astype(np.uint8)
        bg = np.broadcast_to(np.stack([r_ch, g_ch, b_ch], axis=-1), (height, width, 3)).copy()

    im = Image.fromarray(bg)
    draw = ImageDraw.Draw(im)

    # Simple 3D camera projection: cam at (0.46, 0.0, 0.28) looking at (0.08, 0.0, 0.08)
    cam_pos = np.array([0.46, 0.0, 0.27], dtype=float)
    target = np.array([0.08, 0.0, 0.08], dtype=float)
    fwd = target - cam_pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    focal = width * 1.05

    def proj(pt3):
        v = np.asarray(pt3, dtype=float) - cam_pos
        z = float(np.dot(v, fwd))
        if z <= 0.05:
            z = 0.05
        x = float(np.dot(v, right)) / z * focal + width * 0.5
        y = -float(np.dot(v, up)) / z * focal + height * 0.54
        return (x, y, z)

    # Draw workbench optical grid
    grid_col = (44, 56, 84) if aov == "rtx" else (70, 80, 100)
    for gx in np.linspace(-0.10, 0.26, 9):
        p0 = proj((gx, -0.26, 0.0))
        p1 = proj((gx, 0.26, 0.0))
        draw.line([(p0[0], p0[1]), (p1[0], p1[1])], fill=grid_col, width=1)
    for gy in np.linspace(-0.26, 0.26, 11):
        p0 = proj((-0.10, gy, 0.0))
        p1 = proj((0.26, gy, 0.0))
        draw.line([(p0[0], p0[1]), (p1[0], p1[1])], fill=grid_col, width=1)

    # Draw both arms
    arms = [
        (fk_left, -0.14, (157, 114, 255), (185, 154, 255)),
        (fk_right, 0.14, (59, 130, 246), (96, 165, 250)),
    ]
    for fk, y_off, c_main, c_hi in arms:
        pts = [p + np.array([0.0, y_off, 0.0]) for p in fk["pts"]]
        for j in range(4):
            pa = proj(pts[j])
            pb = proj(pts[j + 1])
            if aov == "depth":
                d_u8 = int(np.clip((1.0 - pa[2]) * 240, 60, 245))
                col = (d_u8, int(d_u8 * 0.9), min(255, int(d_u8 * 1.1)))
            elif aov == "normal":
                col = (140, 210, 245) if y_off > 0 else (220, 130, 240)
            else:
                spec_boost = int((1.0 - rough) * 35)
                col = tuple(min(255, c + spec_boost) for c in c_main)
            w_px = max(6, int((22 - j * 3) / max(0.25, pa[2] * 1.8)))
            draw.line([(pa[0], pa[1]), (pb[0], pb[1])], fill=col, width=w_px)
            r_j = w_px * 0.65
            draw.ellipse([pa[0] - r_j, pa[1] - r_j, pa[0] + r_j, pa[1] + r_j], fill=c_hi)

        tip_p = proj(fk["tip"] + np.array([0.0, y_off, 0.0]))
        jaw_r = 9 if state.get("closed") else 14
        draw.ellipse(
            [tip_p[0] - jaw_r, tip_p[1] - jaw_r, tip_p[0] + jaw_r, tip_p[1] + jaw_r],
            outline=c_hi,
            width=3,
        )

    # HUD watermark badge so user sees live Python Bridge status
    draw.rectangle([14, 14, 415, 42], fill=(12, 16, 26))
    draw.text(
        (24, 21),
        f"LIVE PYTHON BRIDGE ({aov.upper()}) | Light {light_mult:.1f}x | Rough {rough:.2f}",
        fill=(96, 165, 250),
    )
    return np.asarray(im, dtype=np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser(description="Shadow PC Live Omniverse RTX + PhysX Bridge for Operon Studio")
    parser.add_argument("--server", type=str, default="wss://operon-421717890494.us-central1.run.app",
                        help="Operon Cloud Run WebSocket base URL (wss://... or ws://localhost:8080)")
    parser.add_argument("--room", type=str, default="OPERON", help="Operon Studio room code (default: OPERON)")
    parser.add_argument("--usd", type=str, default=None, help="Optional path to custom robot USD/URDF stage")
    parser.add_argument("--width", type=int, default=960, help="RTX render width (default: 960)")
    parser.add_argument("--height", type=int, default=600, help="RTX render height (default: 600)")
    parser.add_argument("--fps", type=float, default=20.0, help="Target JPEG stream frame rate (default: 20)")
    parser.add_argument("--jpeg-quality", type=int, default=78, help="JPEG quality 40..95 (default: 78)")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim headless without desktop GUI window")
    parser.add_argument("--local-sim", action="store_true",
                        help="Run lightweight local Mac/CPU bridge without requiring NVIDIA Isaac Sim")
    args = parser.parse_args()

    ensure_websocket_client()

    has_isaac = False
    sim_app = None
    if not args.local_sim:
        try:
            from isaacsim import SimulationApp  # type: ignore

            sim_app = SimulationApp({
                "headless": args.headless,
                "renderer": "RayTracedLighting",
                "width": args.width,
                "height": args.height,
                "anti_aliasing": 3,
            })
            has_isaac = True
        except ImportError:
            print("[shadow-rtx] 'isaacsim' not found on this machine -> auto-activating --local-sim mode ($0 Mac preview)!")
            args.local_sim = True

    import numpy as np
    from PIL import Image, ImageDraw
    import websockets

    if args.local_sim or not has_isaac:
        state = {
            "q4": [0.0, 0.35, 0.65, 0.0],
            "q6": [0.0, 0.35, 0.65, 0.0, 0.4, 0.0],
            "closed": False,
            "aov": "rtx",
            "light": 2.8,
            "angle": 35.0,
            "rough": 0.22,
        }
        prev_p = None
        prev_v = np.zeros(3)
        prev_R = None
        ws_base = args.server.rstrip("/").replace("https://", "wss://").replace("http://", "ws://")
        ws_url = f"{ws_base}/api/room/{args.room.upper()}/ws?role=isaac&id=local-mac-bridge"

        async def local_bridge_loop() -> None:
            nonlocal prev_p, prev_v, prev_R
            min_interval = 1.0 / max(5.0, min(30.0, args.fps))
            while True:
                try:
                    print(f"[local-sim] Connecting to Operon Relay: {ws_url}")
                    async with websockets.connect(ws_url, max_size=512 * 1024, ping_interval=20) as ws:
                        print("[local-sim] Connected! Streaming live frames + kinematic IMU to Operon Studio...")

                        async def recv_loop() -> None:
                            async for raw in ws:
                                try:
                                    msg = json.loads(raw)
                                    if msg.get("t") == "isaac_cmd":
                                        if isinstance(msg.get("q4"), list) and len(msg["q4"]) >= 4:
                                            state["q4"] = [float(x) for x in msg["q4"][:4]]
                                        if isinstance(msg.get("q6"), list) and len(msg["q6"]) >= 6:
                                            state["q6"] = [float(x) for x in msg["q6"][:6]]
                                        if "closed" in msg:
                                            state["closed"] = bool(msg["closed"])
                                        if msg.get("aov") in ("rtx", "depth", "normal", "semantic", "instance"):
                                            state["aov"] = msg["aov"]
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
                        dt = min_interval
                        try:
                            while True:
                                now = time.perf_counter()
                                if now - last_send >= min_interval:
                                    dt = now - last_send if last_send > 0 else min_interval
                                    last_send = now
                                    q6_arr = np.array(state["q6"], dtype=float)
                                    p_site, R_site = forearm_fk_rad(q6_arr, np)
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

                                    rgb = render_local_sim_aov(state, args.width, args.height, np, Image, ImageDraw)
                                    im = Image.fromarray(rgb)
                                    bio = io.BytesIO()
                                    im.save(bio, format="JPEG", quality=args.jpeg_quality)
                                    jpg_b64 = base64.b64encode(bio.getvalue()).decode("ascii")
                                    payload = {
                                        "t": "isaac_frame",
                                        "aov": state["aov"],
                                        "jpg": jpg_b64,
                                        "fps": round(1.0 / max(1e-3, dt), 1),
                                        "imu": {
                                            "ax": round(float(a_proper[0]), 4),
                                            "ay": round(float(a_proper[1]), 4),
                                            "az": round(float(a_proper[2]), 4),
                                            "gx": round(float(gyro_body[0]), 4),
                                            "gy": round(float(gyro_body[1]), 4),
                                            "gz": round(float(gyro_body[2]), 4),
                                        },
                                    }
                                    await ws.send(json.dumps(payload))
                                await asyncio.sleep(0.01)
                        finally:
                            recv_task.cancel()
                except Exception as exc:
                    print(f"[local-sim] Reconnecting in 2s ({exc})...")
                    await asyncio.sleep(2.0)

        asyncio.run(local_bridge_loop())
        return

    import omni.replicator.core as rep  # type: ignore
    import omni.usd  # type: ignore
    from pxr import Gf, UsdGeom

    try:
        from isaacsim.core.api import World  # type: ignore
        from isaacsim.sensors.physics import IMUSensor  # type: ignore
    except ImportError:
        from omni.isaac.core import World  # type: ignore
        from omni.isaac.sensor import IMUSensor  # type: ignore

    configure_rtx_render_settings()

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
    stage = omni.usd.get_context().get_stage()
    build_default_operon_stage(stage, args.usd)

    imu_sensor = IMUSensor(
        prim_path="/World/AbelArm/forearm_link/watch_site/imu_sensor",
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
        "aov": "rtx",
        "light": 2.8,
        "angle": 35.0,
        "rough": 0.22,
    }
    prev_p = None
    prev_v = np.zeros(3)
    prev_R = None

    ws_base = args.server.rstrip("/").replace("https://", "wss://").replace("http://", "ws://")
    ws_url = f"{ws_base}/api/room/{args.room.upper()}/ws?role=isaac&id=shadow-rtx"

    async def bridge_loop() -> None:
        nonlocal prev_p, prev_v, prev_R
        min_interval = 1.0 / max(5.0, min(30.0, args.fps))

        while sim_app.is_running():
            try:
                print(f"[shadow-rtx] Connecting to Operon Relay: {ws_url}")
                async with websockets.connect(ws_url, max_size=512 * 1024, ping_interval=20) as ws:
                    print("[shadow-rtx] Connected! Streaming Omniverse RTX + PhysX IMU to Operon Studio...")

                    async def recv_loop() -> None:
                        async for raw in ws:
                            try:
                                msg = json.loads(raw)
                                if msg.get("t") == "isaac_cmd":
                                    if isinstance(msg.get("q4"), list) and len(msg["q4"]) >= 4:
                                        state["q4"] = [float(x) for x in msg["q4"][:4]]
                                    if isinstance(msg.get("q6"), list) and len(msg["q6"]) >= 6:
                                        state["q6"] = [float(x) for x in msg["q6"][:6]]
                                    if "closed" in msg:
                                        state["closed"] = bool(msg["closed"])
                                    if msg.get("aov") in ("rtx", "depth", "normal", "semantic", "instance"):
                                        state["aov"] = msg["aov"]
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
                            # 1. Articulate both IdealArm (Left) and RealArm (Right) 6-DOF arms in USD
                            q4_arr = np.array(state["q4"], dtype=float)
                            q6_arr = np.array(state["q6"], dtype=float)
                            update_articulated_arm_usd(stage, "IdealArm", -0.115, q4_arr, state["closed"], np)
                            update_articulated_arm_usd(stage, "RealArm", 0.115, q6_arr, state["closed"], np)

                            # 2. Update forearm watch_site pose from live teleop joints so PhysX IMU experiences real kinematics
                            p_site, R_site = forearm_fk_rad(q6_arr, np)
                            ws_prim = stage.GetPrimAtPath("/World/AbelArm/forearm_link/watch_site")
                            if ws_prim.IsValid():
                                q_site = quat_from_rot_matrix(R_site, np)
                                set_xform_pose(
                                    ws_prim,
                                    (float(p_site[0]), float(p_site[1]) + 0.115, float(p_site[2])),
                                    quat_wxyz=q_site,
                                )

                            # 3. Live-bind Domain Randomization sliders (light, angle, rough) to UsdLux & UsdShade
                            update_domain_randomization_usd(stage, state)

                            # 4. Step PhysX + RTX Renderer
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

                            now = time.perf_counter()
                            if now - last_send >= min_interval:
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
                                await ws.send(json.dumps(payload))

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


if __name__ == "__main__":
    main()
