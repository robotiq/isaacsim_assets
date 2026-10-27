"""Readers of the TSF-85 tactile sensor of the 2F-85 and 2F-140, one per ``SensorSim`` variant (readers.py).

Names are imported on first use, so ``tactile.effective_response`` (numpy / scipy only) imports without Isaac.
"""

_EXPORTS = {
    "ContactPoint": "contact_points", "ContactPointReader": "contact_points", "link_contacts": "contact_points",
    "effective_response": "effective_response", "pad_maps": "effective_response", "patch_force": "effective_response",
    "TactilePlot": "plot",
    "COLS": "readers", "ROWS": "readers", "SIDES": "readers", "PAD_LINKS": "readers", "EffectiveTactile": "readers",
    "RaycastTactile": "readers", "TactileSensor": "readers", "make_tactile_sensor": "readers",
    "markers_path": "readers",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        import importlib

        return getattr(importlib.import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
