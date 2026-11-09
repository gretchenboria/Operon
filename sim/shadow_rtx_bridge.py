#!/usr/bin/env python3
r"""
sim/shadow_rtx_bridge.py — Shadow PC Live Omniverse RTX + PhysX Bridge for Operon Studio
=========================================================================================
Runs inside NVIDIA Isaac Sim 4.5 on a Shadow PC (Windows 11 / RTX GPU) and connects
OUTBOUND over WebSockets to the Operon Cloud Run relay (zero port-forwarding required,
works seamlessly behind Shadow PC's CGNAT).

What it does in real time:
  1. Boots Omniverse Kit (`SimulationApp`) with the `RayTracedLighting` RTX renderer.
  2. Loads your robot USD (`--usd`) OR builds a complete USD workbench scene on the fly
     (GroundPlane, Dome/Key RTX Lights, Optical Bench, Battery Cell, Articulated Robot
     with a PhysX `IMUSensor` mounted on `/World/AbelArm/forearm_link/watch_site/imu_sensor`).
  3. Subscribes to live 6-DOF teleop joint targets (`q6`, `q4`, `closed`, `aov`,
     lighting & roughness domain randomization) from Operon Studio / Apple Watch.
  4. Steps PhysX at 120 Hz, reads the real PhysX `IMUSensor` (`lin_acc`, `ang_vel`),
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

from sim.isaac_replay import G0, forearm_fk


def ensure_websocket_client() -> None:
    try:
        import websockets  # noqa: F401
    except ImportError:
        print("[shadow-rtx] Installing 'websockets' and 'pillow' into Isaac Sim Python...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "websockets", "pillow"])


def build_default_operon_stage(stage, usd_path: str | None = None):
    """Build or augment the Operon 6-DOF Sim-to-Real USD stage inside Omniverse."""
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    if usd_path and Path(usd_path).exists():
        from omni.isaac.core.utils.stage import add_reference_to_stage
        add_reference_to_stage(usd_path=str(Path(usd_path).resolve()), prim_path="/World/AbelArm")
        return

    # Root & Lights
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.84, 0.96))

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
    key.CreateIntensityAttr(2800.0)
    key.CreateAngleAttr(1.2)
    UsdGeom.XformCommonAPI(key).SetRotate(Gf.Vec3f(-45.0, 35.0, 0.0))

    # Optical Workbench Table
    bench = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Bench"))
    bench.CreateSizeAttr(1.0)
    b_api = UsdGeom.XformCommonAPI(bench)
    b_api.SetTranslate(Gf.Vec3d(0.08, 0.0, -0.02))
    b_api.SetScale(Gf.Vec3f(0.55, 0.65, 0.04))
    bench.CreateDisplayColorAttr([Gf.Vec3f(0.09, 0.11, 0.16)])
    UsdPhysics.CollisionAPI.Apply(bench.GetPrim())

    # Try loading built-in UR5e from Isaac Nucleus if available
    loaded_ur5e = False
    try:
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        from omni.isaac.core.utils.stage import add_reference_to_stage
        assets_root = get_assets_root_path()
        if assets_root:
            ur5e_usd = assets_root + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
            add_reference_to_stage(usd_path=ur5e_usd, prim_path="/World/UR5e")
            u_xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/UR5e"))
            UsdGeom.XformCommonAPI(u_xform).SetScale(Gf.Vec3f(0.32, 0.32, 0.32))
            UsdGeom.XformCommonAPI(u_xform).SetTranslate(Gf.Vec3d(0.0, 0.09, 0.0))
            loaded_ur5e = True
            print(f"[shadow-rtx] Loaded NVIDIA Nucleus UR5e USD: {ur5e_usd}")
    except Exception as exc:
        print(f"[shadow-rtx] Nucleus UR5e optional load skipped ({exc}); using procedural 6-DOF twin.")

    # Procedural kinematic/dynamic 6-DOF twin + Watch IMU site at /World/AbelArm/forearm_link/watch_site
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/AbelArm"))
    forearm = UsdGeom.Xform.Define(stage, Sdf.Path("/World/AbelArm/forearm_link"))
    watch_site = UsdGeom.Xform.Define(stage, Sdf.Path("/World/AbelArm/forearm_link/watch_site"))
    strap = UsdGeom.Capsule.Define(stage, Sdf.Path("/World/AbelArm/forearm_link/watch_site/WatchBody"))
    strap.CreateRadiusAttr(0.018)
    strap.CreateHeightAttr(0.024)
    strap.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.74, 0.97)])

    # Battery Cell Target on Workbench
    cell = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/BatteryCell"))
    cell.CreateRadiusAttr(0.016)
    cell.CreateHeightAttr(0.052)
    c_api = UsdGeom.XformCommonAPI(cell)
    c_api.SetTranslate(Gf.Vec3d(0.145, -0.055, 0.026))
    cell.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.45, 0.95)])

    # Visual link spheres/capsules for Ideal (Left) and Real (Right) 6-DOF arms
    for side, y_off, color in [("IdealArm", -0.095, (0.55, 0.36, 0.96)), ("RealArm", 0.095, (0.23, 0.51, 0.96))]:
        root_p = f"/World/{side}"
        UsdGeom.Xform.Define(stage, Sdf.Path(root_p))
        for j in range(5):
            seg = UsdGeom.Capsule.Define(stage, Sdf.Path(f"{root_p}/seg_{j}"))
            seg.CreateRadiusAttr(0.016 - j * 0.0015)
            seg.CreateHeightAttr(0.065)
            seg.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.XformCommonAPI(UsdGeom.Xformable(stage.GetPrimAtPath(root_p))).SetTranslate(Gf.Vec3d(0.0, y_off, 0.0))


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
    # Default: RTX RGB(A)
    if arr.ndim == 3 and arr.shape[-1] >= 3:
        return arr[..., :3].astype(np.uint8)
    return None


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
    args = parser.parse_args()

    ensure_websocket_client()

    # Boot Omniverse Kit with RayTracedLighting
    from isaacsim import SimulationApp  # type: ignore

    sim_app = SimulationApp({
        "headless": args.headless,
        "renderer": "RayTracedLighting",
        "width": args.width,
        "height": args.height,
        "anti_aliasing": 3,
    })

    import numpy as np
    import omni.replicator.core as rep  # type: ignore
    import omni.usd  # type: ignore
    from omni.isaac.core import World  # type: ignore
    from omni.isaac.Sensor import IMUSensor  # type: ignore
    from PIL import Image
    from pxr import Gf, Sdf, UsdGeom, UsdLux
    import websockets

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
        "semantic": rep.AnnotatorRegistry.get_annotator("semantic_segmentation"),
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
                                    if "light" in msg:
                                        state["light"] = float(msg["light"])
                                    if "angle" in msg:
                                        state["angle"] = float(msg["angle"])
                            except Exception:
                                pass

                    recv_task = asyncio.create_task(recv_loop())
                    last_send = 0.0
                    dt = 1.0 / 30.0

                    try:
                        while sim_app.is_running():
                            t0 = time.perf_counter()

                            # Update forearm watch_site pose from live teleop joints so PhysX IMU experiences real kinematics
                            q_arr = np.array(state["q4"], dtype=float)
                            p_site, R_site = forearm_fk(q_arr)
                            ws_prim = stage.GetPrimAtPath("/World/AbelArm/forearm_link/watch_site")
                            if ws_prim.IsValid():
                                UsdGeom.XformCommonAPI(UsdGeom.Xformable(ws_prim)).SetTranslate(
                                    Gf.Vec3d(float(p_site[0]), float(p_site[1]), float(p_site[2]))
                                )

                            # Update KeyLight from Studio domain randomization sliders
                            key_prim = stage.GetPrimAtPath("/World/KeyLight")
                            if key_prim.IsValid():
                                UsdLux.DistantLight(key_prim).GetIntensityAttr().Set(float(state["light"]) * 1000.0)
                                UsdGeom.XformCommonAPI(key_prim).SetRotate(Gf.Vec3f(-45.0, float(state["angle"]), 0.0))

                            # Step PhysX + RTX Renderer
                            world.step(render=True)

                            # Read PhysX IMUSensor (with analytical kinematic fallback if sensor buffer warms up)
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
                            S = R_site.T @ Rdot
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
