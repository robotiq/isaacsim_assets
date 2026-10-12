"""Contact forces between the gripper links and an object, on both engines.

The engine-level reading lives in grippers/demo/mcp_bridge/contact_forces.py,
shared with the teleop demo's contact plot; this module only groups it per
gripper link touching one object.

``read()`` returns ``{link name: Contact}`` for every gripper link touching the
object this frame.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pxr import Usd, UsdPhysics

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "grippers" / "demo" / "mcp_bridge"))
import contact_forces as cf  # noqa: E402

# MuJoCo lists every geom pair within its collision margin as a contact, even
# centimetres apart and carrying no force; count only pairs this close.
NEWTON_TOUCH_DIST = 1.0e-3  # m


@dataclass
class Contact:
    force: float  # N, magnitude of the summed contact force on the object
    min_separation: float  # m, negative = penetration
    points: int


class ContactReader:
    def __init__(self, gripper: str, root: str, object_path: str, engine: str):
        self.gripper, self.root, self.object_path, self.engine = gripper, root, object_path, engine

    # -- before play -----------------------------------------------------------
    def prepare(self, stage: Usd.Stage) -> None:
        if self.engine != "physx":
            return
        root = stage.GetPrimAtPath(self.root)
        cf.physx_report_contacts(cf.rigid_bodies_under(root))
        # A gripper stalled on a static object comes to rest and PhysX puts the
        # articulation to sleep (test stage only, the asset is untouched).
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                cf.physx_keep_awake(prim)

    # -- while playing ---------------------------------------------------------
    def read(self) -> dict[str, Contact]:
        return self._read_physx() if self.engine == "physx" else self._read_newton()

    def _read_physx(self) -> dict[str, Contact]:
        from isaacsim.core.simulation_manager import SimulationManager

        out: dict[str, list] = {}
        for pc in cf.physx_pairs(SimulationManager.get_physics_dt()):
            if self.object_path not in (pc.a, pc.b):
                continue
            link = pc.other(self.object_path)
            if not link.startswith(self.root + "/"):
                continue
            name = link[len(self.root) + 1:].split("/")[0]
            acc = out.setdefault(name, [np.zeros(3), math.inf, 0])
            acc[0] += pc.force_vec
            acc[1] = min(acc[1], pc.min_separation)
            acc[2] += pc.points
        return {k: Contact(float(np.linalg.norm(v[0])), v[1], v[2]) for k, v in out.items()}

    def _read_newton(self) -> dict[str, Contact]:
        solver = cf.newton_solver()
        names = cf.newton_body_names(solver)
        # The object: its own body if it is dynamic (MuJoCo names it after its path,
        # "/World/Cube" -> "..._World_Cube"), else world (body 0) for a static collider.
        obj_tag = self.object_path.replace("/", "_")
        obj_bodies = {b for b, n in names.items() if n.endswith(obj_tag)} or {0}
        tag = f"{self.gripper}_"
        out: dict[str, list] = {}
        for pc in cf.newton_pairs(solver, touch_dist=NEWTON_TOUCH_DIST, names=names):
            if pc.a_id in obj_bodies and pc.b_id not in obj_bodies:
                name = pc.b
            elif pc.b_id in obj_bodies and pc.a_id not in obj_bodies:
                name = pc.a
            else:
                continue  # self-contact, or not involving the object
            if tag not in name:
                continue
            link = name.split(tag, 1)[1]
            acc = out.setdefault(link, [0.0, math.inf, 0])
            acc[0] += pc.force
            acc[1] = min(acc[1], pc.min_separation)
            acc[2] += pc.points
        return {k: Contact(v[0], v[1], v[2]) for k, v in out.items()}
