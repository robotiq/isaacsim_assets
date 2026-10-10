"""Contact forces between the gripper links and a static object, on both engines.

Same approach as grippers/demo/mcp_bridge/make_contact_plot.py:
- PhysX: ``PhysxContactReportAPI`` on every gripper rigid body (applied before
  play), then the simulation interface's contact report each frame:
  force = sum of contact impulses / physics dt.
- Newton (MuJoCo-Warp): the live solver's contacts; the normal force of each
  contact is ``efc.force`` at its constraint address, the separation is
  ``contact.dist``. The object is its own body if dynamic, else a world-body geom.

``read()`` returns ``{link name: Contact}`` for every gripper link touching the
object this frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from pxr import PhysxSchema, Usd, UsdPhysics

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
        for prim in Usd.PrimRange(stage.GetPrimAtPath(self.root)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            # A gripper stalled on a static object comes to rest and PhysX puts the
            # articulation to sleep; sleeping bodies produce no contact reports.
            # Keep it awake (test stage only, the asset is untouched).
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)

    # -- while playing ---------------------------------------------------------
    def read(self) -> dict[str, Contact]:
        return self._read_physx() if self.engine == "physx" else self._read_newton()

    def _read_physx(self) -> dict[str, Contact]:
        import omni.physx
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import PhysicsSchemaTools

        dt = SimulationManager.get_physics_dt()
        headers, data = omni.physx.get_physx_simulation_interface().get_contact_report()
        out: dict[str, list] = {}
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if self.object_path == a0:
                link = a1
            elif self.object_path == a1:
                link = a0
            else:
                continue
            if not link.startswith(self.root + "/"):
                continue
            imp = np.zeros(3)
            sep = math.inf
            for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                imp += np.asarray(data[i].impulse, dtype=float)
                sep = min(sep, float(data[i].separation))
            name = link[len(self.root) + 1:].split("/")[0]
            acc = out.setdefault(name, [np.zeros(3), math.inf, 0])
            acc[0] += imp / dt
            acc[1] = min(acc[1], sep)
            acc[2] += h.num_contact_data
        return {k: Contact(float(np.linalg.norm(v[0])), v[1], v[2]) for k, v in out.items()}

    def _read_newton(self) -> dict[str, Contact]:
        import mujoco
        import isaacsim.physics.newton as newton_ext

        solver = newton_ext.acquire_stage().solver  # the current solver
        mj, mjm, mjd = solver.mj_model, solver.mjw_model, solver.mjw_data
        flat = lambda x: np.asarray(x.numpy()).reshape(-1)
        nacon = int(flat(mjd.nacon)[0])
        if nacon == 0:
            return {}
        geom_body = flat(mjm.geom_bodyid).astype(int)
        geom = np.asarray(mjd.contact.geom.numpy())
        efc_addr = np.asarray(mjd.contact.efc_address.numpy())
        dist = flat(mjd.contact.dist)
        efc_force = flat(mjd.efc.force)
        tag = f"{self.gripper}_"
        name_of = lambda b: mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        # The object: its own body if it is dynamic (MuJoCo names it after its path,
        # "/World/Cube" -> "..._World_Cube"), else world (body 0) for a static collider.
        obj_tag = self.object_path.replace("/", "_")
        obj_bodies = {b for b in range(mj.nbody) if name_of(b).endswith(obj_tag)} or {0}
        out: dict[str, list] = {}
        for k in range(nacon):
            g0, g1 = int(geom[k, 0]), int(geom[k, 1])
            b0, b1 = int(geom_body[g0]), int(geom_body[g1])
            if b0 in obj_bodies and b1 not in obj_bodies:
                body = b1
            elif b1 in obj_bodies and b0 not in obj_bodies:
                body = b0
            else:
                continue  # self-contact, or not involving the object
            if float(dist[k]) > NEWTON_TOUCH_DIST:
                continue  # within the collision margin, not touching
            name = name_of(body)
            if tag not in name:
                continue
            link = name.split(tag, 1)[1]
            addr = int(efc_addr[k, 0])
            f = abs(float(efc_force[addr])) if 0 <= addr < efc_force.size else 0.0
            acc = out.setdefault(link, [0.0, math.inf, 0])
            acc[0] += f
            acc[1] = min(acc[1], float(dist[k]))
            acc[2] += 1
        return {k: Contact(v[0], v[1], v[2]) for k, v in out.items()}
