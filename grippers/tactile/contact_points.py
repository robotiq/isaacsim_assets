"""Contact points (position, normal, normal force) between a gripper and what it touches, on both engines.

- PhysX: ``PhysxContactReportAPI`` on every gripper rigid body (``prepare``, before play), then the
  simulation interface's contact report each frame; a point's force is its impulse / physics dt.
- Newton (MuJoCo-Warp): the live solver's contacts: ``contact.pos`` and ``contact.frame`` give position
  and normal, ``efc.force`` the force, and only contacts within ``NEWTON_TOUCH_DIST`` count (MuJoCo lists
  every pair inside its collision margin, even centimetres apart and carrying no force).

``object_path`` is the touched prim (a dynamic body, or a static collider that MuJoCo puts on the world
body); ``None`` takes whatever is not the gripper.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

NEWTON_TOUCH_DIST = 1.0e-3  # m
_MJ_PYRAMIDAL = 0  # mjtCone


@dataclass
class ContactPoint:
    link: str  # gripper link touching the object, e.g. "left_fingertip"
    position: np.ndarray  # world, m
    normal: np.ndarray  # world, unit
    force: float  # N, normal force of this point
    separation: float  # m, negative = penetration
    impulse_force: np.ndarray | None = None  # N, the full contact force vector where the engine gives one (PhysX)


@dataclass
class Contact:
    """Everything one gripper link pushes on the object, summed."""

    force: float  # N, magnitude of the summed contact force
    min_separation: float  # m
    points: int


def link_contacts(points: list[ContactPoint]) -> dict[str, Contact]:
    """``{link: Contact}``: the force of a link is the magnitude of its summed force vectors (PhysX) or the sum of
    its normal forces (Newton)."""
    acc: dict[str, list] = {}
    for p in points:
        a = acc.setdefault(p.link, [np.zeros(3), 0.0, math.inf, 0])
        if p.impulse_force is not None:
            a[0] += p.impulse_force
        else:
            a[1] += p.force
        a[2] = min(a[2], p.separation)
        a[3] += 1
    return {k: Contact(float(np.linalg.norm(a[0])) + a[1], a[2], a[3]) for k, a in acc.items()}


class ContactPointReader:
    def __init__(self, gripper: str, root: str, engine: str, object_path: str | None = None):
        self.gripper, self.root, self.engine, self.object_path = gripper, root, engine, object_path
        self._scenes = None

    def _physx_dt(self) -> float:
        """The PhysX step the contact impulses are per, from the stage's physics scene. (SimulationManager's
        ``get_physics_dt`` is the first scene it registered, which need not be the one the stage steps with.)"""
        import omni.usd
        from pxr import UsdPhysics

        if self._scenes is None:
            stage = omni.usd.get_context().get_stage()
            self._scenes = [p.GetAttribute("physxScene:timeStepsPerSecond") for p in stage.Traverse()
                            if p.IsA(UsdPhysics.Scene)]
        hz = [a.Get() for a in self._scenes if a and a.HasAuthoredValue()]
        return 1.0 / float(max(hz)) if hz else 1.0 / 60.0

    def prepare(self, stage) -> None:
        """Before play. PhysX only. A gripper stalled on a static object comes to rest and PhysX puts the
        articulation to sleep, and sleeping bodies produce no contact reports: keep it awake. On the stage's
        current edit target (the test stage, or the session layer for a live scene)."""
        if self.engine != "physx":
            return
        from pxr import PhysxSchema, Usd, UsdPhysics

        for prim in Usd.PrimRange(stage.GetPrimAtPath(self.root)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)

    def read(self) -> list[ContactPoint]:
        return self._read_physx() if self.engine == "physx" else self._read_newton()

    def _read_physx(self) -> list[ContactPoint]:
        import omni.physx
        from pxr import PhysicsSchemaTools

        dt = self._physx_dt()
        headers, data = omni.physx.get_physx_simulation_interface().get_contact_report()
        inside = self.root + "/"
        out = []
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if a0.startswith(inside) == a1.startswith(inside):
                continue  # gripper-gripper or neither
            link, other = (a0, a1) if a0.startswith(inside) else (a1, a0)
            if self.object_path is not None and other != self.object_path:
                continue
            name = link[len(inside):].split("/")[0]
            for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                c = data[i]
                n = np.asarray(c.normal, dtype=float)
                f = np.asarray(c.impulse, dtype=float) / dt
                out.append(ContactPoint(name, np.asarray(c.position, dtype=float), n, abs(float(np.dot(f, n))),
                                        float(c.separation), f))
        return out

    def _read_newton(self) -> list[ContactPoint]:
        import mujoco
        import isaacsim.physics.newton as newton_ext

        solver = newton_ext.acquire_stage().solver
        mj, mjm, mjd = solver.mj_model, solver.mjw_model, solver.mjw_data
        flat = lambda x: np.asarray(x.numpy()).reshape(-1)
        nacon = int(flat(mjd.nacon)[0])
        if nacon == 0:
            return []
        geom_body = flat(mjm.geom_bodyid).astype(int)
        geom = np.asarray(mjd.contact.geom.numpy())
        efc_addr = np.asarray(mjd.contact.efc_address.numpy())
        pos = np.asarray(mjd.contact.pos.numpy()).reshape(-1, 3)
        frame = np.asarray(mjd.contact.frame.numpy()).reshape(-1, 3, 3)
        dist = flat(mjd.contact.dist)
        efc_force = flat(mjd.efc.force)
        pyramidal = int(mj.opt.cone) == _MJ_PYRAMIDAL
        tag = f"{self.gripper}_"
        name_of = lambda b: mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        is_gripper = {b: tag in name_of(b) for b in range(mj.nbody)}
        # A dynamic object is its own body, named after its path ("/World/Cube" -> "..._World_Cube"); a
        # static collider belongs to the world body (0).
        obj_bodies = None
        if self.object_path is not None:
            obj_tag = self.object_path.replace("/", "_")
            obj_bodies = {b for b in range(mj.nbody) if name_of(b).endswith(obj_tag)} or {0}
        out = []
        for k in range(nacon):
            b0, b1 = int(geom_body[int(geom[k, 0])]), int(geom_body[int(geom[k, 1])])
            if is_gripper[b0] == is_gripper[b1]:
                continue  # self-contact, or not involving the gripper
            body, other = (b0, b1) if is_gripper[b0] else (b1, b0)
            if obj_bodies is not None and other not in obj_bodies:
                continue
            if float(dist[k]) > NEWTON_TOUCH_DIST:
                continue
            rows = efc_addr[k][efc_addr[k] >= 0]
            rows = rows[rows < efc_force.size]
            if rows.size == 0:
                f = 0.0
            elif pyramidal:  # the normal force is the sum of the facet forces
                f = float(np.abs(efc_force[rows]).sum())
            else:  # elliptic: the first row is the normal
                f = abs(float(efc_force[rows[0]]))
            out.append(ContactPoint(name_of(body).rsplit(tag, 1)[-1], pos[k].astype(float), frame[k, 0].astype(float),
                                    f, float(dist[k])))
        return out
