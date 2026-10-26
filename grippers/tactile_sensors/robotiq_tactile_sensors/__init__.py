"""Simulated Robotiq TSF tactile sensors.

- ``tsf``: the taxel array of a TSF (numpy only) and where it sits on a gripper's fingertips (``PadMount``).
- ``mounts``: the mounts of the Robotiq grippers that carry a TSF (``ROBOTIQ_2F_85``).
- ``spread``: spreading a contact force over the taxels (torch).
- ``isaaclab``: the Isaac Lab sensor ``TactileSensorCfg`` / ``TactileSensor`` built on these.

Names are imported on first use, so the numpy-only parts import without torch or Isaac Lab.
"""

_EXPORTS = {
    "TsfSpec": "tsf", "TSF85": "tsf", "PadMount": "tsf", "FingertipMounts": "tsf", "taxel_positions": "tsf",
    "ROBOTIQ_2F_85": "mounts",
    "spread_forces": "spread",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        import importlib

        return getattr(importlib.import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
