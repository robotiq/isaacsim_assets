"""The Isaac Lab tactile sensor: a PhysX ``ContactSensor`` on the two pads that also builds the taxel maps."""

from __future__ import annotations

import torch
from isaaclab_physx.sensors.contact_sensor import ContactSensor

from ..sensing import pad_object_forces, taxel_force_map, taxel_positions_world
from ..tsf import taxel_positions
from .cfg import TactileSensorCfg


class TactileSensor(ContactSensor):
    """Contact sensor on the two pads of a gripper with TSF fingertips, with the force per taxel.

    ``taxel_force``: ``(num_envs, 2, rows, cols)`` [N], pad 0 = left, pad 1 = right, row 0 at the fingertip end.
    ``pad_force``: ``(num_envs, 2)`` [N], the force each pad carries. ``object_force``: ``(num_envs, 2, num_objects)`` [N],
    what each pad carries from each filtered object. ``taxel_positions_w``: the taxel centres in the world
    frame, ``(num_envs, 2, rows * cols, 3)``. The raw contact data stays available as ``data``.
    """

    cfg: TactileSensorCfg

    def __init__(self, cfg: TactileSensorCfg):
        # cfg.prim_path is the prim that holds the pads (the scene has expanded its {ENV_REGEX_NS}): the sensing bodies
        # are the two pads under it. Done here, once, because the scene copies configs and a path derived in the
        # config's __post_init__ would be rebuilt from the unexpanded one.
        pads = f"({cfg.mount.left.body}|{cfg.mount.right.body})"
        if not cfg.prim_path.endswith(pads):
            cfg.prim_path = f"{cfg.prim_path}/{pads}"
        super().__init__(cfg)

    def _initialize_impl(self):
        super()._initialize_impl()
        mount, spec = self.cfg.mount, self.cfg.mount.spec
        names = self.body_names  # the sensor lists its bodies in its own order: put them in (left, right)
        missing = [p.body for p in mount.pads if p.body not in names]
        if missing:
            raise RuntimeError(f"tactile sensor at '{self.cfg.prim_path}': no pad body {missing}, found {names}")
        self._pad_order = torch.tensor([names.index(p.body) for p in mount.pads], device=self._device)
        self._taxels_local = torch.stack(
            [torch.tensor(taxel_positions(p, spec), dtype=torch.float32, device=self._device) for p in mount.pads]
        )  # (2, T, 3)

    @property
    def taxel_positions_w(self) -> torch.Tensor:
        data = self.data
        return taxel_positions_world(data.pos_w.torch, data.quat_w.torch, self._taxels_local, self._pad_order)

    @property
    def object_force(self) -> torch.Tensor:
        """Normal force [N] of each pad on each filtered object, ``(num_envs, 2, num_objects)``: pads in (left, right)
        order, objects in the order of ``filter_prim_paths_expr``. Forces below ``force_floor`` read 0."""
        return pad_object_forces(self.data.normal_force_matrix_w.torch, self._pad_order, self.cfg.force_floor)

    @property
    def taxel_force(self) -> torch.Tensor:
        data = self.data
        taxel = taxel_force_map(
            data.pos_w.torch,
            data.quat_w.torch,
            data.normal_force_matrix_w.torch,
            data.contact_pos_w.torch,
            self._taxels_local,
            self._pad_order,
            self.cfg.force_floor,
            self.cfg.sigma,
            self.cfg.max_contact_distance,
        )
        spec = self.cfg.mount.spec
        return taxel.view(-1, 2, spec.rows, spec.cols)

    @property
    def pad_force(self) -> torch.Tensor:
        return self.taxel_force.sum(dim=(-1, -2))
