"""Per-test viewport camera + light presets for GUI runs.

A preset is a JSON file in ``views/`` named after the pytest id of the case
(``test_gripper_closes-Robotiq_2F_85.json``), falling back to the function
name (``test_gripper_closes.json``). It stores the viewport camera pose, the
viewport lighting mode (Camera Light / Stage Lights / ...) and every UsdLux
light in the stage. ``save_view`` captures the current state at
the end of a ``--gui --hold --save-view`` run; ``apply_view`` restores it on
later ``--gui`` runs. Headless runs never touch it.

The light part is plain USD (testable without Kit); the camera part needs the
Kit viewport and is only imported when used.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

VIEWS_DIR = Path(__file__).resolve().parent / "views"

_LIGHT_ATTRS = ("inputs:intensity", "inputs:exposure", "inputs:color", "inputs:texture:file", "inputs:radius", "inputs:angle")


def preset_paths(nodeid: str) -> list[Path]:
    """Candidate preset files for a pytest node id, most specific first."""
    name = nodeid.split("::")[-1]  # test_gripper_closes[Robotiq_2F_85]
    func = name.split("[")[0]
    out = []
    if "[" in name:
        case = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.replace("[", "-").rstrip("]"))
        out.append(VIEWS_DIR / f"{case}.json")
    out.append(VIEWS_DIR / f"{func}.json")
    return out


def find_preset(nodeid: str) -> Path | None:
    return next((p for p in preset_paths(nodeid) if p.is_file()), None)


# --- lights (pure USD) -----------------------------------------------------


def _value(v):
    if isinstance(v, (Gf.Vec3f, Gf.Vec3d)):
        return [float(x) for x in v]
    if isinstance(v, Sdf.AssetPath):
        return v.path
    return v


def collect_lights(stage: Usd.Stage) -> list[dict]:
    lights = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdLux.LightAPI):
            continue
        attrs = {}
        for name in _LIGHT_ATTRS:
            a = prim.GetAttribute(name)
            if a and a.HasAuthoredValue():
                attrs[name] = _value(a.Get())
        t, r, s, _pivot, _order = UsdGeom.XformCommonAPI(prim).GetXformVectors(Usd.TimeCode.Default())
        lights.append({
            "path": str(prim.GetPath()),
            "type": prim.GetTypeName(),
            "translate": [float(x) for x in t],
            "rotateXYZ": [float(x) for x in r],
            "scale": [float(x) for x in s],
            "attrs": attrs,
        })
    return lights


def define_lights(stage: Usd.Stage, lights: list[dict]) -> None:
    for spec in lights:
        prim = stage.DefinePrim(spec["path"], spec["type"])
        xf = UsdGeom.XformCommonAPI(prim)
        xf.SetTranslate(Gf.Vec3d(*spec["translate"]))
        xf.SetRotate(Gf.Vec3f(*spec["rotateXYZ"]))
        xf.SetScale(Gf.Vec3f(*spec["scale"]))
        for name, value in spec["attrs"].items():
            attr = prim.GetAttribute(name)
            if not attr:
                continue
            if name == "inputs:color":
                value = Gf.Vec3f(*value)
            elif name == "inputs:texture:file":
                value = Sdf.AssetPath(value)
            attr.Set(value)


# --- camera (needs Kit) -----------------------------------------------------


def _camera_state():
    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    vp = get_active_viewport()
    return ViewportCameraState(str(vp.camera_path), vp)


def collect_camera() -> dict:
    cs = _camera_state()
    return {"position": _value(cs.position_world), "target": _value(cs.target_world)}


def set_camera(cam: dict) -> None:
    cs = _camera_state()
    cs.set_position_world(Gf.Vec3d(*cam["position"]), True)
    cs.set_target_world(Gf.Vec3d(*cam["target"]), True)


# --- viewport lighting mode (needs Kit) -------------------------------------
# The viewport's Lighting menu ("Camera Light" / "Stage Lights" / "Lights Off"
# / a light rig) is a per-stage carb setting, not USD, so it is stored apart
# from the light prims. It is applied through the menu's registered actions.

_LIGHTING_EXT = "omni.kit.viewport.menubar.lighting"


def _lighting_mode_key(stage: Usd.Stage) -> str:
    from pxr import UsdUtils

    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    return f"/exts/{_LIGHTING_EXT}/lightingMode/{stage_id}"


def collect_lighting_mode(stage: Usd.Stage) -> str:
    import carb.settings

    return carb.settings.get_settings().get(_lighting_mode_key(stage)) or "stage"


def set_lighting_mode(mode: str) -> None:
    import omni.kit.actions.core

    registry = omni.kit.actions.core.get_action_registry()
    if mode in ("stage", "camera", "off"):
        registry.get_action(_LIGHTING_EXT, f"set_lighting_mode_{mode}").execute()
    else:  # a light-rig name
        registry.get_action(_LIGHTING_EXT, "set_lighting_mode_rig").execute(mode)


# --- save / apply -----------------------------------------------------------


def save_view(nodeid: str, stage: Usd.Stage) -> Path:
    path = preset_paths(nodeid)[0]
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "camera": collect_camera(),
        "lighting_mode": collect_lighting_mode(stage),
        "lights": collect_lights(stage),
    }
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path


def apply_view(nodeid: str, stage: Usd.Stage) -> Path | None:
    path = find_preset(nodeid)
    if path is None:
        return None
    data = json.loads(path.read_text())
    define_lights(stage, data.get("lights", []))
    if "lighting_mode" in data:
        set_lighting_mode(data["lighting_mode"])
    if "camera" in data:
        set_camera(data["camera"])
    return path
