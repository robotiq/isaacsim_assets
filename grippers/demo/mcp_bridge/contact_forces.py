"""Read contact forces from a running Isaac Sim simulation, on PhysX and on Newton.

Single source for the engine-level contact reading shared by:
- make_contact_plot.py, which prepends this file's source to the code it injects
  into the live Isaac process through the MCP bridge (so it has no import path);
- tests/runtime/_contacts.py, which imports it.

Keep it self-contained: only the standard library, numpy and pxr at module level;
Kit / Isaac modules are imported inside the functions that need them. No
``from __future__ import annotations``: the plot exec()s this source outside any
module, where dataclasses cannot resolve string annotations.

PhysX: put ``PhysxContactReportAPI`` on the bodies to watch (before play, or do a
Stop -> Play after), then read the simulation interface's contact report each
frame. Each pair's force is its contact impulses over the physics step. Sleeping
bodies report nothing: a gripper stalled on an object comes to rest and sleeps,
so ``physx_keep_awake`` an articulation whose contacts must keep reporting.

Newton (MuJoCo-Warp): the live solver's contacts. Each contact's normal force is
``efc.force`` at its constraint address, its separation ``contact.dist``. MuJoCo
lists every geom pair within its collision margin, even centimetres apart with no
force, so callers that want touching pairs only pass ``touch_dist``.

Callers group the per-pair records themselves (per link, per pad side, ...).
"""

import math
from dataclasses import dataclass, field

import numpy as np
from pxr import PhysxSchema, Usd, UsdPhysics


@dataclass
class PairContact:
    """Contacts between two actors during the last physics step.

    PhysX: ``a`` / ``b`` are actor prim paths, ``force_vec`` the summed contact
    force vector (impulses / dt) and ``force`` the sum of the per-point force
    magnitudes. Newton: ``a`` / ``b`` are MuJoCo body names, ``a_id`` / ``b_id``
    body ids, ``force`` the summed normal force (no vector)."""

    a: str
    b: str
    force: float  # N
    min_separation: float  # m, negative = penetration
    points: int
    force_vec: np.ndarray = field(default_factory=lambda: np.zeros(3))
    a_id: int = -1
    b_id: int = -1

    def other(self, name: str) -> str:
        return self.b if self.a == name else self.a


# --------------------------------------------------------------------------- PhysX
def physx_report_contacts(prims, threshold: float = 0.0) -> bool:
    """Apply ``PhysxContactReportAPI`` to ``prims`` (on the current edit target).
    Returns True if any prim did not have it yet: PhysX only reports those after
    the next Play (or a Stop -> Play if already playing)."""
    newly = False
    for prim in prims:
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            newly = True
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(threshold)
    return newly


def physx_keep_awake(articulation_root) -> None:
    """Never let this articulation sleep, so its contacts keep being reported."""
    PhysxSchema.PhysxArticulationAPI.Apply(articulation_root).CreateSleepThresholdAttr().Set(0.0)


def rigid_bodies_under(root) -> list:
    return [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def physx_scene_dt(stage) -> float:
    """Physics step (s) authored on the stage's PhysicsScene, else PhysX's 1/60."""
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            a = PhysxSchema.PhysxSceneAPI(p).GetTimeStepsPerSecondAttr()
            if a and a.Get():
                return 1.0 / float(a.Get())
            break
    return 1.0 / 60.0


def physx_pairs(dt: float) -> list:
    """The current step's PhysX contact report, one ``PairContact`` per actor pair;
    ``dt`` is the physics step (s) the impulses were integrated over."""
    import omni.physx
    from pxr import PhysicsSchemaTools

    headers, data = omni.physx.get_physx_simulation_interface().get_contact_report()
    out = []
    for h in headers:
        vec = np.zeros(3)
        mag = 0.0
        sep = math.inf
        for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
            if i >= len(data):
                break
            imp = np.asarray(data[i].impulse, dtype=float)
            vec += imp
            mag += float(np.linalg.norm(imp))
            sep = min(sep, float(data[i].separation))
        out.append(PairContact(
            a=str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
            b=str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
            force=mag / dt, min_separation=sep, points=h.num_contact_data, force_vec=vec / dt,
        ))
    return out


# -------------------------------------------------------------------------- Newton
def newton_solver():
    """The SolverMuJoCo Isaac's Newton stage is stepping now, or None (not on
    Newton, not built yet). Cheap, and always the live one, unlike a heap scan,
    which also finds the stale solvers a Stop -> Play leaves behind."""
    try:
        import isaacsim.physics.newton as newton_ext

        stage = newton_ext.acquire_stage()
        solver = getattr(stage, "solver", None) if stage is not None else None
        if solver is not None and getattr(solver, "mjw_data", None) is not None:
            return solver
    except Exception:
        pass
    return None


def _flat(x) -> np.ndarray:
    try:
        return np.asarray(x.numpy()).reshape(-1)
    except Exception:
        return np.asarray(x).reshape(-1)


def newton_body_names(solver) -> dict:
    """{MuJoCo body id: name}; names follow the prim path ("..._World_Cube")."""
    import mujoco

    mj = solver.mj_model
    return {i: mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_BODY, i) or "" for i in range(mj.nbody)}


def newton_pairs(solver, touch_dist: float | None = None, names: dict | None = None) -> list:
    """The solver's current contacts, one ``PairContact`` per body pair. With
    ``touch_dist`` (m), contacts farther apart than that are skipped."""
    mjm, mjd = solver.mjw_model, solver.mjw_data
    nacon = int(_flat(mjd.nacon)[0])
    if nacon == 0:
        return []
    names = newton_body_names(solver) if names is None else names
    geom_body = _flat(mjm.geom_bodyid).astype(int)
    geom = np.asarray(mjd.contact.geom.numpy())
    efc_addr = np.asarray(mjd.contact.efc_address.numpy())
    dist = _flat(mjd.contact.dist)
    efc_force = _flat(mjd.efc.force)
    pairs: dict = {}
    for k in range(nacon):
        d = float(dist[k])
        if touch_dist is not None and d > touch_dist:
            continue
        b0, b1 = int(geom_body[int(geom[k, 0])]), int(geom_body[int(geom[k, 1])])
        addr = int(efc_addr[k, 0])
        f = abs(float(efc_force[addr])) if 0 <= addr < efc_force.size else 0.0
        key = (b0, b1)
        pc = pairs.get(key)
        if pc is None:
            pc = pairs[key] = PairContact(a=names.get(b0, ""), b=names.get(b1, ""), force=0.0,
                                          min_separation=math.inf, points=0, a_id=b0, b_id=b1)
        pc.force += f
        pc.min_separation = min(pc.min_separation, d)
        pc.points += 1
    return list(pairs.values())
