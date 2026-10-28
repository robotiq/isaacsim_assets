"""Ready-made Isaac Lab robot configs of the Robotiq grippers with TSF fingertips (the gripper alone).

The USD assets are not part of this package: they are the ``Robotiq_2F_85`` / ``Robotiq_2F_140`` folders of the repo's ``grippers/``
folder. From a clone installed with ``pip install -e`` they are found next to this package; otherwise point to them with the
``ROBOTIQ_GRIPPERS_DIR`` environment variable or the ``grippers_dir`` argument. A missing asset is an error when the config is built,
not when the scene is spawned. The constants ``ROBOTIQ_2F85_TSF85_CFG`` and ``ROBOTIQ_2F140_TSF140_CFG`` are built on first use.
"""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


def grippers_dir(path: str | os.PathLike | None = None) -> Path:
    """The folder that holds the gripper assets: ``path``, else ``$ROBOTIQ_GRIPPERS_DIR``, else the repo's ``grippers/``."""
    if path is not None:
        return Path(path)
    if os.environ.get("ROBOTIQ_GRIPPERS_DIR"):
        return Path(os.environ["ROBOTIQ_GRIPPERS_DIR"])
    return Path(__file__).resolve().parents[3]  # .../grippers, from an editable install of the repo


def _usd(gripper: str, path: str | os.PathLike | None) -> str:
    usd = grippers_dir(path) / gripper / f"{gripper}.usda"
    if not usd.is_file():
        raise FileNotFoundError(
            f"gripper asset not found: {usd}. Pass grippers_dir=<the repo's grippers/ folder> or set ROBOTIQ_GRIPPERS_DIR"
            " (this package only holds the sensor, the USD assets are in the isaacsim_assets repo)."
        )
    return str(usd)


def _gripper_cfg(gripper: str, tip: str, path: str | os.PathLike | None) -> ArticulationCfg:
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=_usd(gripper, path),
            # the tactile fingertips, the TSF pad collider and the pad contact reports of the asset
            variants={"Fingertip": tip, "SensorSim": "EffectiveResponse"},
            activate_contact_sensors=True,
        ),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={"finger_joint": 0.0}),
        actuators={
            # the other gripper joints follow finger_joint through PhysX mimic joints
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["finger_joint"], effort_limit_sim=26.0, stiffness=3.0, damping=0.05
            ),
        },
    )


def robotiq_2f85_tsf85_cfg(grippers_dir: str | os.PathLike | None = None) -> ArticulationCfg:
    """The 2F-85 on TSF-85 fingertips, alone (its own articulation root, fixed base). Position it with ``init_state.pos/rot``; to
    mount it on an arm, weld its ``base_link`` to the flange as in grippers/demo/teleop/ur5robot_with_2F-85.usda. The pad bodies are
    then at ``<prim>/Robotiq_2F_85/{left,right}_fingertip``: give that prim as ``TactileSensorCfg.prim_path``. The gripper joint
    goes 0 (open) to about 0.8 rad (closed)."""
    return _gripper_cfg("Robotiq_2F_85", "Tactile_TSF85", grippers_dir)


def robotiq_2f140_tsf140_cfg(grippers_dir: str | os.PathLike | None = None) -> ArticulationCfg:
    """The 2F-140 on TSF-140 fingertips (the same sensor as the TSF-85, further out on the finger), alone. Like the 2F-85 config,
    with the pad bodies at ``<prim>/Robotiq_2F_140/{left,right}_fingertip`` and the mount ``mounts.ROBOTIQ_2F_140``. The gripper
    joint goes 0 (open) to 0.785 rad (closed): its limit is 45 degrees."""
    return _gripper_cfg("Robotiq_2F_140", "Tactile_TSF140", grippers_dir)


_CONSTANTS = {"ROBOTIQ_2F85_TSF85_CFG": robotiq_2f85_tsf85_cfg, "ROBOTIQ_2F140_TSF140_CFG": robotiq_2f140_tsf140_cfg}


def __getattr__(name):
    if name in _CONSTANTS:
        return _CONSTANTS[name]()
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
