"""Configuration of the Isaac Lab tactile sensor. Light imports only (usable before Isaac Sim starts)."""

from __future__ import annotations

from isaaclab.sensors.contact_sensor.contact_sensor_cfg import ContactSensorCfg
from isaaclab.utils import configclass

from ..mounts import ROBOTIQ_2F_85
from ..tsf import FingertipMounts


@configclass
class TactileSensorCfg(ContactSensorCfg):
    """TSF tactile sensor on the two fingertips of a gripper.

    ``prim_path`` is the prim that directly holds the two pad bodies (e.g. ``{ENV_REGEX_NS}/Robot/.../Robotiq_2F_85``); the
    sensor appends the pad names. It is a ``ContactSensor`` on the two pad bodies, filtered on the objects the pads should feel
    (``filter_prim_paths_expr``, rigid bodies: a static collider cannot be a filter target, make it a kinematic rigid
    body), plus the taxel maps built from it. Read ``sensor.taxel_force`` (N per taxel, ``(num_envs, 2, rows, cols)``,
    pads in (left, right) order) and ``sensor.pad_force``. PhysX backend only.

    The robot must be spawned with ``activate_contact_sensors=True`` and with the gripper's tactile fingertips
    (``Fingertip=Tactile_TSF85``), see ``ROBOTIQ_2F85_TSF85_CFG``.
    """

    class_type: type | str = "robotiq_tactile_sensors.isaaclab.sensor:TactileSensor"

    mount: FingertipMounts = ROBOTIQ_2F_85
    """Where the taxel arrays sit on the pad bodies (the gripper's mounts, see ``mounts.py``)."""

    sigma: float = 0.006
    """Width [m] of the Gaussian that spreads an object's force over the taxels around its contact point."""

    force_floor: float = 0.05
    """Contact forces below this [N] are ignored (solver noise)."""

    def __post_init__(self):
        # the pad poses place the taxels; the contact points locate the force on them
        self.track_pose = True
        self.track_contact_points = True
        if self.max_contact_data_count_per_prim is None:
            self.max_contact_data_count_per_prim = 16
