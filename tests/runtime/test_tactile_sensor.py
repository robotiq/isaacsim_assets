"""The TSF-85 tactile sensor of the 2F-85 and the 2F-140: both ``SensorSim`` models under the same tests,
on both grippers.

Every common test runs once per model on the force map (``read_force()``, N per taxel) against the same
baseline, the pad force of the contact report (``ContactReader``) averaged over the same frames; only the
passing thresholds differ (``MODELS``). Model-specific tests cover the raycast depth, the effective response
r against Eq. 1, and the two models agreeing on one simulation. What they check is in tests/runtime/README.md.

Most tests close the gripper on a free box (fixed_cubes/<gripper>_tactile.json gives its size and starting
place) in zero gravity, and average the last frames. Free, the box is squeezed as in a grasp: the pads centre it
and press it flat, so both push equally (checked). A box fixed in space over-constrains the two fingers instead:
the load split follows sub-0.1 mm differences (0 / 42 N, 162 / 20 N on the 2F-140) and a rigid pad can meet it
along one edge. The box is judged where it is at the stall, not where it started.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import omni.timeline
import omni.usd
import pytest
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

from _contacts import ContactReader
from _cube import CUBE_PATH, add_fixed_cube, load_preset
from _scene import REPO, engine_for, load_gripper, move_to, step
from _tactile import COLS, FINGERTIPS, GRIPPERS, ROWS, SIDES, TactilePlot, sensor_variants
from tactile import ContactPointReader, EffectiveTactile, RaycastTactile, make_tactile_sensor, markers_path
from tactile.effective_response import GAMMA, effective_response, pad_plane

DRIVEN = "finger_joint"
PADS = {"left": "left_fingertip", "right": "right_fingertip"}
TARGET = 0.8  # rad, full close: beyond the box
SETTLE_FRAMES = 30
HOLD_FRAMES = 60
SAMPLE_FRAMES = 20
PHYSX_HZ = 1000  # as the fixed-cube test
BOX_MASS = 0.1  # kg, the free box (zero gravity: it only sets the box's inertia)
EFFORTS = (5.0, 20.0)  # finger_joint max effort for the weak / strong grip (stock: 26 parallel grip, 16.5 compliant, 15-25 Newton)


def tactile_preset(gripper: str) -> str:
    """A box over part of the sensing area, clear of the finger links: fixed_cubes/<gripper>_tactile.json (its size
    and starting place). Only part: on the 2F-140 the TSF tip's collider extends 34 mm below the sensing area, so a
    box there would push where no taxel reads."""
    return f"{gripper}_tactile"


@dataclass(frozen=True)
class Model:
    """One sensor method and what it is held to. The tests are the same for every model; only
    these thresholds differ."""

    sensor_sim: str  # the SensorSim variant (under the gripper's TSF fingertip)
    sensor_class: type
    physics: tuple  # the Physics variants it runs on
    sum_band: tuple  # force-map total / contact-report pad force, on a held grip
    fast_band: tuple  # the same after a full close in one step (test_tactile_force_stable_on_fast_close)
    scale_tol: float  # (map total ratio) / (pad force ratio) within 1 +- this, weak -> strong grip
    pad_compliant: bool  # the variant softens the pad (the raycast needs it) or leaves it rigid


MODELS = {
    # The raycast turns depth into force with the soft layer's stiffness (E / thickness), while PhysX's compliant
    # pad is a spring per contact point, so the depth per newton follows how many contact points PhysX makes.
    # Measured (Isaac Sim 6.2): x0.7 at light grips, x1.10-1.51 held on the free box (sets the 1.6 bound), up to
    # x1.69 after a full close in one step at 240 Hz (sets the 1.9 fast-close bound).
    "raycast": Model("Raycast", RaycastTactile, ("Physx_parallel_grip", "Physx_compliant"),
                     (0.65, 1.6), (0.65, 1.9), 0.3, True),
    "effective": Model("EffectiveResponse", EffectiveTactile,
                       ("Physx_parallel_grip", "Physx_compliant", "Newton_parallel_grip", "Newton_compliant"),
                       (0.9, 1.1), (0.9, 1.1), 0.15, False),
}
SENSORS = list(MODELS)


def _cases(*axes) -> list:
    """pytest params over the grippers and ``axes`` (lists of values), id "<gripper>-<value>-...": the
    gripper leads the id, where --save-cube and the view presets look for it."""
    out = [(g,) for g in GRIPPERS]
    for axis in axes:
        out = [c + (v,) for c in out for v in axis]
    return [pytest.param(*c, id="-".join(map(str, c))) for c in out]


SENSOR_CASES = _cases(SENSORS)
CASES = [pytest.param(g, s, p, id=f"{g}-{s}-{p}") for g in GRIPPERS for s in SENSORS for p in MODELS[s].physics]
SCALE_PHYSICS = ("Physx_parallel_grip", "Newton_parallel_grip")
SCALE_CASES = [c for c in CASES if c.values[2] in SCALE_PHYSICS]  # _known() below, once KNOWN_FAILURES exists
EFFECTIVE_SCALE_CASES = [c for c in SCALE_CASES if c.values[1] == "effective"]
RAYCAST_CASES = _cases(MODELS["raycast"].physics)
# On Newton_parallel_grip the 2F-140's pads do not stay parallel under the grip: they pitch 1.3 / 1.7 deg along
# the rows, in a slight V (the 2F-85's 0.3-0.7 deg, PhysX's mimic coupling 0.00 deg), so the free box wedges and
# one pad's force bunches at one end. The pads balance; it is the asset's Newton coupling, not the sensor.
_NEWTON_PATCH = ("on Newton_parallel_grip the 2F-140's pads pitch 1.3-1.7 deg under the grip (not parallel), so the "
                 "free box wedges between them and one pad's force bunches at one end")
KNOWN_FAILURES = {"Robotiq_2F_140-effective-Newton_parallel_grip": _NEWTON_PATCH,
                  "Robotiq_2F_140-Newton_parallel_grip": _NEWTON_PATCH}


def _known(params: list) -> list:
    """``params`` with a non-strict xfail on the cases of KNOWN_FAILURES."""
    return [pytest.param(*p.values, id=p.id, marks=[*p.marks, pytest.mark.xfail(reason=KNOWN_FAILURES[p.id])])
            if p.id in KNOWN_FAILURES else p for p in params]


EFFECTIVE_CASES = _known(_cases(MODELS["effective"].physics))
SCALE_CASES, EFFECTIVE_SCALE_CASES = _known(SCALE_CASES), _known(EFFECTIVE_SCALE_CASES)
# One static box (a collider, no body), as a fixture or a table edge: PhysX contact against a static collider and
# Newton's world body as the object. On the 2F-85, where its load split stays well conditioned.
STATIC_CASES = [pytest.param("Robotiq_2F_85", "effective", p, "static", id=f"Robotiq_2F_85-effective-{p}-static")
                for p in SCALE_PHYSICS]
FOOTPRINT_CASES = [pytest.param(*c.values, "free", id=c.id, marks=c.marks) for c in _known(CASES)] + STATIC_CASES
RATE_PHYSICS = "Physx_parallel_grip"  # the rate tests set PhysX time steps


def _at_60hz(gripper: str, sensor: str, *params, hz: int = 60):
    """A rate case; the raycast method fails it at 60 Hz (strict xfail): PhysX does not resolve the soft pad's
    compliant contact at that step, so the depth no longer follows the force. A held (kinematic) box barely sinks
    (force map x0.04-0.43 of the pad force), a free box sinks far too deep (x2.6-4.7); at 240 Hz it is within
    the raycast band. The effective response reads the contact report, which is fine at 60 Hz."""
    marks = []
    if sensor == "raycast" and hz == 60:
        marks.append(pytest.mark.xfail(strict=True, reason="the compliant pad is not resolved at a 60 Hz PhysX step: "
                                                           "the depth does not follow the force"))
    return pytest.param(gripper, sensor, *params, marks=marks, id="-".join([gripper, sensor, *map(str, params)]))


DEFAULT_RATE_CASES = [_at_60hz(g, s, obj) for g in GRIPPERS for s in SENSORS for obj in ("free", "kinematic")]
FAST_CLOSE_CASES = [_at_60hz(g, s, hz, hz=hz) for g in GRIPPERS for s in SENSORS for hz in (60, 240)]

FORCE_ACTIVE_N = 0.1  # a taxel "carries force" above this
PAD_MIN_N = 1.0  # each pad pushes at least this hard on the box
FOOTPRINT_MARGIN = 0.0025  # m: taxels this close to the box edge are not judged as covered
CLEAR_SHARE = 0.05  # at most this share of the force is on taxels clear of the box
PEAK_BOUND = 1.15  # no taxel carries more than this times the pad force
BALANCE_TOL = 0.15  # a free box: the two pad forces within this fraction of the larger
# The footprint (_taxel_masks, _model) is the box's face kept upright at its stall centre. A free box turning
# 1 deg moves its 12 x 16.5 mm face's corners by < 0.2 mm, well inside FOOTPRINT_MARGIN; it turns <= 0.23 deg.
MAX_BOX_ROT_DEG = 1.0

ACTIVE_MM = 0.05  # a taxel "reads" above this depth
MM_PER_N = (0.003, 0.05)  # depth per newton of pad force, around the pad spring's 0.012
# The TSF-85 soft layer is 3 mm thick: deeper and the object would be through it. The depth test grips below the
# sensor's rated range (0-225 N, robotiq.com/tactile-sensor-fingertips); the stock drive at 1000 Hz pushes the
# raycast's compliant pad ~320 N per pad on the 2F-85 (139 N on the 2F-140), past it on the 2F-85.
MAX_SINK_MM = 3.0
DEPTH_EFFORT = EFFORTS[1]  # finger_joint max effort of the depth test: the strong grip, 60-95 N per pad on the raycast's compliant pad
ACTIVE_R = 0.03  # a taxel "reads" above this response
MODEL_TOL = 0.35  # mean |r - Eq. 1 on the ideal footprint| within this fraction of the peak
POWER_TOL = 0.25  # r ratio within 25 % of (force ratio) ** GAMMA
UNIFORM_SLACK = 1.15  # r never exceeds (F / box area) ** GAMMA, within this slack
RIGID_DEPTH_MM = 0.3  # the raycast reader on the rigid pad reads below this
FORCE_CORR = 0.95  # correlation of the two force maps
RAY_RESPONSE_TOL = 0.10  # mean |r raycast - r effective| within this fraction of the peak


def _box(gripper: str) -> tuple[np.ndarray, np.ndarray]:
    """(size, starting centre) of the box."""
    preset = load_preset(tactile_preset(gripper), fixed=True)
    size = preset["size"]
    size = np.array(size if isinstance(size, list) else [size] * 3, dtype=float)
    return size, np.array(preset["position"], dtype=float)


_plot: TactilePlot | None = None


@pytest.fixture(autouse=True)
def _tactile_plot(request: pytest.FixtureRequest):
    """With --gui, show the taxel forces live while each test runs."""
    global _plot
    if request.config.getoption("--gui") and _plot is None:
        _plot = TactilePlot()
    yield


@dataclass
class Rig:
    scene: object
    sensor: object  # the method under test, ``read()`` = force
    contacts: ContactReader  # the baseline: pad force of the contact report
    extra: object = None  # a second reader on the same sim, when a test asks for one
    box: object = None  # RigidPrim of a free / kinematic box, to read where it is; None for a static one
    box_start: np.ndarray = None  # its starting centre (from the preset)

    def box_pose(self) -> tuple[np.ndarray, float]:
        """(centre m, rotation from its starting orientation in deg) of the box now."""
        if self.box is None:  # a static collider does not move
            return self.box_start, 0.0
        pos, quat = (a.numpy()[0] for a in self.box.get_world_poses())  # quaternion wxyz
        return pos.astype(float), math.degrees(2.0 * math.acos(min(1.0, abs(float(quat[0])))))


def _load(simulation_app, view, gripper: str, sensor: str, physics: str, hz: int | None = PHYSX_HZ,
          obj: str = "free", extra: str | None = None) -> Rig:
    """Fresh stage with the box and ``sensor``'s variant, zero gravity. ``obj``: "free" (a dynamic body, the pads
    centre it), "kinematic" (held in place) or "static" (a collider only, no body: the world on Newton).
    ``extra``: "depth" adds a raycast depth reader
    (on the effective variant's rigid pad), "effective" an effective-response reader (on the raycast
    variant's compliant pad)."""
    model = MODELS[sensor]
    engine = engine_for(physics)
    built = {}

    def setup(stage):
        prim = add_fixed_cube(stage, tactile_preset(gripper)).GetPrim()
        if obj == "kinematic":
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(True)
        elif obj == "free":
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(BOX_MASS)
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)  # keep reporting contacts
        scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
        UsdPhysics.Scene(scene_prim).CreateGravityMagnitudeAttr().Set(0.0)  # the box floats until squeezed
        if hz and engine == "physx":
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(hz)
        root = f"/World/Gripper/{gripper}"
        contacts = ContactReader(gripper, root, CUBE_PATH, engine)
        contacts.prepare(stage)
        built["contacts"] = contacts
        built["sensor"] = make_tactile_sensor(stage, root, engine, gripper, CUBE_PATH)  # quantity: force
        assert isinstance(built["sensor"], model.sensor_class), type(built["sensor"])
        if extra == "depth":
            built["extra"] = RaycastTactile(root, "depth")
            built["extra"].prepare(stage)
        elif extra == "effective":
            built["extra"] = EffectiveTactile(root, ContactPointReader(gripper, root, engine, CUBE_PATH), "response")
            built["extra"].prepare(stage)

    scene = load_gripper(simulation_app, gripper, physics, view, setup=setup,
                         variants=sensor_variants(gripper, model.sensor_sim))
    if _plot:
        _plot.show(built["sensor"])
    from isaacsim.core.experimental.prims import RigidPrim

    return Rig(scene, built["sensor"], built["contacts"], built.get("extra"),
               None if obj == "static" else RigidPrim(CUBE_PATH), _box(gripper)[1])


@dataclass
class Squeeze:
    force: dict  # {side: (ROWS, COLS)} mean taxel force, N
    pad: dict  # {side: mean pad force of the contact report, N}
    others: dict  # {link: peak force N} of the links other than the pads that touch the box
    angles: np.ndarray  # finger_joint, deg, per frame
    totals: np.ndarray  # force-map total of both pads, per frame
    points: dict  # {side: (ROWS, COLS, 3)} taxel positions at the stall
    maps: dict  # {quantity: {side: grid}}, the ``also`` quantities
    box: np.ndarray  # the box centre at the stall, m
    box_rot: float  # how far the box turned from its starting orientation, deg


def _squeeze(simulation_app, rig: Rig, ramp: bool = True, also: tuple = ()) -> Squeeze:
    """Close toward full close past the box, hold, and average the last frames. ``also``: more
    quantities of the sensor (``read_<q>``, e.g. "depth", "response") averaged over the same frames."""
    scene, sensor, contacts = rig.scene, rig.sensor, rig.contacts
    if ramp:
        move_to(simulation_app, scene, scene.dof(DRIVEN), TARGET, settle_frames=HOLD_FRAMES - SAMPLE_FRAMES)
    else:
        scene.robot.set_dof_position_targets([TARGET], dof_indices=[scene.dof(DRIVEN)])
        step(simulation_app, HOLD_FRAMES)
    force = {s: [] for s in SIDES}
    pad = {s: [] for s in SIDES}
    maps = {q: {s: [] for s in SIDES} for q in also}
    others, angles, totals, box = {}, [], [], []
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        grids = sensor.read()
        for s, g in grids.items():
            force[s].append(g)
        totals.append(sum(float(g.sum()) for g in grids.values()))
        for q in also:
            for s, g in getattr(rig.extra or sensor, f"read_{q}")().items():
                maps[q][s].append(g)
        c = contacts.read()
        for s in SIDES:
            pad[s].append(c[PADS[s]].force if PADS[s] in c else 0.0)
        for link in set(c) - set(PADS.values()):
            others[link] = max(others.get(link, 0.0), c[link].force)
        angles.append(math.degrees(float(scene.robot.get_dof_positions().numpy()[0][scene.dof(DRIVEN)])))
        box.append(rig.box_pose()[0])
    mean = lambda d: {s: np.mean(v, axis=0) for s, v in d.items()}
    return Squeeze(mean(force), {s: float(np.mean(v)) for s, v in pad.items()}, others, np.array(angles),
                   np.array(totals), sensor.taxel_points(), {q: mean(m) for q, m in maps.items()},
                   np.mean(box, axis=0), rig.box_pose()[1])


def _tilt_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle of the pad surface out of the gripper's XZ plane between taxels ``a`` and ``b`` (``(n, 3)``)."""
    d = (b - a).mean(axis=0)
    return math.degrees(math.atan2(d[1], math.hypot(d[0], d[2])))


def _print(label: str, sq: Squeeze) -> None:
    np.set_printoptions(precision=2, suppress=True)
    for s in SIDES:
        g = sq.force[s]
        print(f"{label} {s}: pad force {sq.pad[s]:.1f} N, map {g.sum():.1f} N (x{g.sum() / max(sq.pad[s], 1e-9):.2f}), "
              f"{int((g > FORCE_ACTIVE_N).sum())}/{ROWS * COLS} taxels, peak {g.max():.1f} N\n{g}")


def _finger_limit_deg(stage) -> float:
    joint = next(p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Gripper")) if p.GetName() == DRIVEN)
    return float(joint.GetAttribute("physics:upperLimit").Get())


def _assert_pad_forces(model: Model, sq: Squeeze, label: str, band: tuple | None = None) -> None:
    """The common baseline check: the force map carries the pad force of the contact report."""
    for s in SIDES:
        total = float(sq.force[s].sum())
        assert np.isfinite(sq.force[s]).all() and (sq.force[s] >= 0).all(), f"{label} {s}: invalid forces"
        assert sq.pad[s] >= PAD_MIN_N, f"{label} {s}: the pad pushes with only {sq.pad[s]:.2f} N"
        ratio = total / sq.pad[s]
        band = band or model.sum_band
        assert band[0] <= ratio <= band[1], (
            f"{label} {s}: the force map carries {total:.1f} N of the {sq.pad[s]:.1f} N pad force (x{ratio:.2f}, "
            f"allowed {band})")
        assert sq.force[s].max() <= PEAK_BOUND * sq.pad[s], f"{label} {s}: one taxel carries {sq.force[s].max():.1f} N"


def _assert_balanced(sq: Squeeze, label: str) -> None:
    """A free box in equilibrium: the two pads push it equally."""
    lo, hi = sorted(sq.pad.values())
    assert hi - lo <= BALANCE_TOL * hi, (
        f"{label}: the pads push the free box unequally, left {sq.pad['left']:.1f} / right {sq.pad['right']:.1f} N")


def _taxel_masks(pitch_mm, points: np.ndarray, size, centre):
    """(covered, clear): taxels well inside the box, and taxels whose rectangle does not touch it."""
    dx = np.abs(points[..., 0] - centre[0]) - size[0] / 2.0
    dz = np.abs(points[..., 2] - centre[2]) - size[2] / 2.0
    covered = (dx <= -FOOTPRINT_MARGIN) & (dz <= -FOOTPRINT_MARGIN)
    clear = (dx >= pitch_mm[0] / 2000.0) | (dz >= pitch_mm[1] / 2000.0)
    return covered, clear


@pytest.mark.parametrize(("gripper", "sensor"), SENSOR_CASES)
def test_tactile_layout(simulation_app, view, gripper: str, sensor: str) -> None:
    model = MODELS[sensor]
    rig = _load(simulation_app, view, gripper, sensor, model.physics[0])
    stage = omni.usd.get_context().get_stage()
    for side in SIDES:
        # 7 x 4 taxels, each (row, col) once
        cells = sorted((r, c) for r, c, _rel, _half in rig.sensor.taxels[side])
        assert cells == [(r, c) for r in range(ROWS) for c in range(COLS)], f"{side}: taxels {cells}"
        assert stage.GetPrimAtPath(markers_path(rig.scene.root, side))
        # the pad: compliant where the method reads the depth, rigid where it does not need it
        case = stage.GetPrimAtPath(f"{rig.scene.root}/{side}_fingertip/visuals/{FINGERTIPS[gripper][1]}")
        assert case, f"no TSF pad collider on the {side} finger"
        mat, _ = UsdShade.MaterialBindingAPI(case).ComputeBoundMaterial("physics")
        k = mat.GetPrim().GetAttribute("physxMaterial:compliantContactStiffness").Get() if mat else None
        assert bool(k and k > 0) == model.pad_compliant, f"{side} pad: compliance {k}, expected {model.pad_compliant}"
    markers = [p for side in SIDES for p in Usd.PrimRange(stage.GetPrimAtPath(markers_path(rig.scene.root, side)))]
    colliding = [str(p.GetPath()) for p in markers if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert not colliding, f"the pad must be the only collider, markers collide: {colliding[:3]}"
    omni.timeline.get_timeline_interface().stop()


@pytest.mark.parametrize(("gripper", "sensor", "physics", "obj"), FOOTPRINT_CASES)
def test_tactile_force_footprint(simulation_app, view, gripper: str, sensor: str, physics: str, obj: str) -> None:
    model = MODELS[sensor]
    rig = _load(simulation_app, view, gripper, sensor, physics, obj=obj)
    step(simulation_app, SETTLE_FRAMES)
    for s, g in rig.sensor.read().items():
        assert g.max() <= FORCE_ACTIVE_N, f"{s} carries {g.max():.2f} N with the gripper open"

    sq = _squeeze(simulation_app, rig)
    limit = _finger_limit_deg(omni.usd.get_context().get_stage())
    move_to(simulation_app, rig.scene, rig.scene.dof(DRIVEN), 0.0, settle_frames=HOLD_FRAMES)
    reopened = rig.sensor.read()
    omni.timeline.get_timeline_interface().stop()
    _print(f"{gripper} {sensor} {physics}", sq)
    for s in SIDES:  # where the taxels sit at the stall, to place fixed_cubes/<gripper>_tactile.json
        p = sq.points[s]
        print(f"{s} taxel rows z (mm): {np.round(p[:, :, 2].mean(axis=1) * 1000, 1)}; cols x (mm): "
              f"{np.round(p[:, :, 0].mean(axis=0) * 1000, 1)}; pad y (mm): {p[..., 1].mean() * 1000:.1f}; tilt (deg) "
              f"across the columns {_tilt_deg(p[:, 0], p[:, -1]):+.2f}, along the rows {_tilt_deg(p[-1], p[0]):+.2f}")

    assert sq.angles.max() <= limit + 0.5, f"finger_joint {sq.angles.max():.1f} deg past its {limit:.0f} deg limit"
    assert not sq.others, "only the pads should touch the box, also: " + ", ".join(
        f"{k} {v:.1f} N" for k, v in sorted(sq.others.items()))
    _assert_pad_forces(model, sq, f"{sensor} {physics}")
    if obj == "free":  # a static box takes any split (it is the world's, not the pads', to balance)
        _assert_balanced(sq, f"{sensor} {physics}")
    size, start = _box(gripper)
    centre = sq.box
    print(f"{gripper} {sensor} {physics}: box moved {np.round((centre - start) * 1000, 2)} mm, turned {sq.box_rot:.2f} deg")
    assert sq.box_rot <= MAX_BOX_ROT_DEG, f"the box turned {sq.box_rot:.2f} deg: the upright footprint no longer fits it"
    for s in SIDES:
        covered, clear = _taxel_masks(pad_plane(sq.points[s])[4], sq.points[s], size, centre)
        g = sq.force[s]
        assert covered.any(), f"{s}: no taxel lies over the box, check fixed_cubes/ placement"
        missing = np.argwhere(covered & (g <= FORCE_ACTIVE_N))
        assert not missing.size, f"{s}: taxels over the box carry no force (row, col): {missing.tolist()}"
        share = float(g[clear].sum() / g.sum())
        print(f"{sensor} {physics} {s}: {share:.1%} of the force on the {int(clear.sum())} taxels clear of the box")
        assert share <= CLEAR_SHARE, f"{s}: taxels clear of the box carry {share:.0%} of the force"
        assert reopened[s].max() <= FORCE_ACTIVE_N, f"{s} still carries {reopened[s].max():.2f} N after reopening"


def _two_efforts(simulation_app, view, gripper: str, sensor: str, physics: str, also: tuple = ()):
    """Squeeze with the weak and the strong grip: [Squeeze, Squeeze]."""
    runs = []
    for effort in EFFORTS:
        rig = _load(simulation_app, view, gripper, sensor, physics)
        rig.scene.robot.set_dof_max_efforts([effort], dof_indices=[rig.scene.dof(DRIVEN)])
        step(simulation_app, SETTLE_FRAMES)
        sq = _squeeze(simulation_app, rig, also=also)
        omni.timeline.get_timeline_interface().stop()
        _print(f"{gripper} {sensor} {physics} max effort {effort}", sq)
        runs.append(sq)
    return runs


@pytest.mark.parametrize(("gripper", "sensor", "physics"), SCALE_CASES)
def test_tactile_force_scales_with_grip_force(simulation_app, view, gripper: str, sensor: str, physics: str) -> None:
    model = MODELS[sensor]
    weak, strong = _two_efforts(simulation_app, view, gripper, sensor, physics)
    for label, sq in (("weak", weak), ("strong", strong)):
        _assert_pad_forces(model, sq, f"{sensor} {physics} {label} grip")
    pad = lambda sq: float(np.mean(list(sq.pad.values())))
    total = lambda sq: float(np.mean([g.sum() for g in sq.force.values()]))
    print(f"{sensor} {physics}: pad force {pad(weak):.1f} -> {pad(strong):.1f} N (x{pad(strong) / pad(weak):.2f}), "
          f"force map {total(weak):.1f} -> {total(strong):.1f} N (x{total(strong) / total(weak):.2f})")
    assert pad(strong) > 1.5 * pad(weak), f"the stronger grip did not push harder: {pad(weak):.1f} -> {pad(strong):.1f} N"
    ratio = (total(strong) / total(weak)) / (pad(strong) / pad(weak))
    assert abs(ratio - 1.0) <= model.scale_tol, (
        f"the force map ratio is x{ratio:.2f} of the pad force ratio (allowed 1 +- {model.scale_tol})")


def _assert_stable(model: Model, sq: Squeeze, label: str, band: tuple | None = None) -> None:
    limit = _finger_limit_deg(omni.usd.get_context().get_stage())
    print(f"{label}: finger_joint peak {sq.angles.max():.1f} deg; force-map total over frames "
          f"{sq.totals.min():.1f}..{sq.totals.max():.1f} N")
    assert sq.angles.max() <= limit + 0.5, f"finger_joint {sq.angles.max():.1f} deg past its {limit:.0f} deg limit"
    assert sq.totals.min() > 0.0, "the force map drops out on some frames"
    _assert_pad_forces(model, sq, label, band)


@pytest.mark.parametrize(("gripper", "sensor", "obj"), DEFAULT_RATE_CASES)
def test_tactile_force_stable_at_default_rate(simulation_app, view, gripper: str, sensor: str, obj: str) -> None:
    rig = _load(simulation_app, view, gripper, sensor, RATE_PHYSICS, hz=None, obj=obj)
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig)
    omni.timeline.get_timeline_interface().stop()
    _print(f"{gripper} {sensor} @ 60 Hz, {obj} box", sq)
    _assert_stable(MODELS[sensor], sq, f"{sensor} 60 Hz {obj}")


@pytest.mark.parametrize(("gripper", "sensor", "hz"), FAST_CLOSE_CASES)
def test_tactile_force_stable_on_fast_close(simulation_app, view, gripper: str, sensor: str, hz: int) -> None:
    """The full-close target in one step, no ramp: the finger hits the pad at speed."""
    rig = _load(simulation_app, view, gripper, sensor, RATE_PHYSICS, hz=hz)
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, ramp=False)
    omni.timeline.get_timeline_interface().stop()
    _print(f"{gripper} {sensor} fast close @ {hz} Hz", sq)
    _assert_stable(MODELS[sensor], sq, f"{sensor} fast close {hz} Hz", MODELS[sensor].fast_band)


@pytest.mark.parametrize(("gripper", "physics"), RAYCAST_CASES)
def test_raycast_depth(simulation_app, view, gripper: str, physics: str) -> None:
    """The depth the force is made from: millimetres per newton of pad force in the spring's range, and
    never through the soft layer at a grip within the sensor's rating."""
    rig = _load(simulation_app, view, gripper, "raycast", physics)
    rig.scene.robot.set_dof_max_efforts([DEPTH_EFFORT], dof_indices=[rig.scene.dof(DRIVEN)])
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, also=("depth",))
    omni.timeline.get_timeline_interface().stop()
    _print(f"{gripper} raycast {physics}", sq)
    for s in SIDES:
        d = sq.maps["depth"][s]
        active = d[d > ACTIVE_MM]
        assert active.size, f"{s}: no taxel reads a depth"
        assert d.max() <= MAX_SINK_MM, f"{s}: the box sinks {d.max():.1f} mm into the pad (max {MAX_SINK_MM} mm)"
        mm_per_n = float(active.mean()) / sq.pad[s]
        print(f"raycast {physics} {s}: {active.mean():.2f} mm at {sq.pad[s]:.1f} N = {mm_per_n:.3f} mm/N")
        assert MM_PER_N[0] <= mm_per_n <= MM_PER_N[1], f"{s}: {mm_per_n:.3f} mm/N outside {MM_PER_N}"


def _model(points: np.ndarray, size, centre, force: float) -> np.ndarray:
    """Eq. 1 on the ideal footprint: the box's face on the pad, at the measured force."""
    o, u, v, uv, pitch = pad_plane(points)
    corners = np.array([[centre[0] + sx * size[0] / 2, o[1], centre[2] + sz * size[2] / 2]
                        for sx in (-1, 1) for sz in (-1, 1)])
    patch = np.array([[np.dot(p - o, u), np.dot(p - o, v)] for p in corners]) * 1000.0
    return effective_response(patch, force, uv, pitch)


@pytest.mark.parametrize(("gripper", "physics"), EFFECTIVE_CASES)
def test_effective_response_matches_eq1(simulation_app, view, gripper: str, physics: str) -> None:
    """r (``read_response()``) on the box is Eq. 1 of the box footprint at the measured pad force, and
    never exceeds the uniform-pressure value."""
    rig = _load(simulation_app, view, gripper, "effective", physics)
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, also=("response",))
    omni.timeline.get_timeline_interface().stop()
    size, _start = _box(gripper)
    assert sq.box_rot <= MAX_BOX_ROT_DEG, f"the box turned {sq.box_rot:.2f} deg: the upright footprint no longer fits it"
    for s in SIDES:
        r = sq.maps["response"][s]
        model = _model(sq.points[s], size, sq.box, sq.pad[s])
        err = float(np.abs(r - model).mean() / max(model.max(), 1e-9))
        bound = UNIFORM_SLACK * (sq.pad[s] / (size[0] * size[2] * 1e6)) ** GAMMA
        print(f"{gripper} {physics} {s}: peak r {r.max():.3f} (Eq. 1 on the box {model.max():.3f}, uniform bound {bound:.3f}), "
              f"mean |r - model| {err:.1%} of the peak")
        assert err <= MODEL_TOL, f"{s}: r differs from Eq. 1 on the box footprint by {err:.0%} of the peak"
        assert r.max() <= bound, f"{s}: peak r {r.max():.3f} exceeds the uniform-pressure value {bound:.3f}"


@pytest.mark.parametrize(("gripper", "sensor", "physics"), EFFECTIVE_SCALE_CASES)
def test_effective_response_power_law(simulation_app, view, gripper: str, sensor: str, physics: str) -> None:
    """r goes as the pad force to the power gamma (0.5), not proportionally like the force map."""
    weak, strong = _two_efforts(simulation_app, view, gripper, sensor, physics, also=("response",))
    mean_r = lambda sq: float(np.mean([g[g > ACTIVE_R].mean() for g in sq.maps["response"].values()]))
    pad = lambda sq: float(np.mean(list(sq.pad.values())))
    expected = (pad(strong) / pad(weak)) ** GAMMA
    print(f"r {mean_r(weak):.3f} -> {mean_r(strong):.3f} (x{mean_r(strong) / mean_r(weak):.2f}), pad force "
          f"x{pad(strong) / pad(weak):.2f} (** {GAMMA} = x{expected:.2f})")
    assert mean_r(strong) > mean_r(weak), "a stronger grip should respond more"
    assert abs(mean_r(strong) / mean_r(weak) / expected - 1.0) <= POWER_TOL, (
        f"r ratio {mean_r(strong) / mean_r(weak):.2f} vs (force ratio) ** {GAMMA} = {expected:.2f}")


@pytest.mark.parametrize("gripper", _cases())
def test_effective_rigid_pad(simulation_app, view, gripper: str) -> None:
    """The EffectiveResponse pad is rigid: the raycast depth reads (almost) nothing there, the force
    map (the common tests) and r still show the footprint."""
    physics = "Physx_parallel_grip"
    rig = _load(simulation_app, view, gripper, "effective", physics, extra="depth")
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, also=("depth",))
    omni.timeline.get_timeline_interface().stop()
    peak = max(g.max() for g in sq.maps["depth"].values())
    print(f"raycast depth peak {peak:.2f} mm on the rigid pad, force map {sum(g.sum() for g in sq.force.values()):.0f} N")
    assert peak <= RIGID_DEPTH_MM, f"the rigid pad should read ~no depth, got {peak:.2f} mm"
    _assert_pad_forces(MODELS["effective"], sq, "effective rigid pad")


@pytest.mark.parametrize("gripper", _cases())
def test_sensorsim_variants(simulation_app, gripper: str) -> None:
    """The two methods are variants: Raycast (compliant pad, ray parameters) and EffectiveResponse
    (rigid pad), both tagging the shared taxel grid with their method and the Eq. 1 parameters. The 2F-85 has a third,
    Precise (the deformable pad of the ETS CoRo lab), which tags the grid and adds the pads and their backings
    (tests/runtime/test_tactile_precise.py runs it)."""
    asset = Usd.Stage.Open(str(REPO / "grippers" / gripper / f"{gripper}.usda"))
    prim = asset.GetDefaultPrim()
    tsf = FINGERTIPS[gripper][0]
    for other in set(prim.GetVariantSet("Fingertip").GetVariantNames()) - {tsf}:
        prim.GetVariantSet("Fingertip").SetVariantSelection(other)
        assert "SensorSim" not in prim.GetVariantSets().GetNames(), f"SensorSim should only exist under {tsf}"
    prim.GetVariantSet("Fingertip").SetVariantSelection(tsf)
    precise = gripper == "Robotiq_2F_85"
    assert set(prim.GetVariantSet("SensorSim").GetVariantNames()) == \
        {"None", "Raycast", "EffectiveResponse"} | ({"Precise"} if precise else set())
    if precise:  # the grid is tagged and the deformable pads and their backings are there
        prim.GetVariantSet("SensorSim").SetVariantSelection("Precise")
        markers = asset.GetPrimAtPath(f"{prim.GetPath()}/{gripper}/left_inner_finger/tactile_zone_visuals")
        assert markers.GetAttribute("robotiq:tactile:method").Get() == "precise"
        for side in ("left", "right"):
            assert asset.GetPrimAtPath(f"{prim.GetPath()}/{side}_tsf_pad/simulation_mesh"), side
            assert asset.GetPrimAtPath(f"{prim.GetPath()}/{gripper}/{side}_pad_backing"), side
    expected = {"None": None, "Raycast": "raycast", "EffectiveResponse": "effectiveResponse"}
    for variant, method in expected.items():
        prim.GetVariantSet("SensorSim").SetVariantSelection(variant)
        markers = asset.GetPrimAtPath(f"{prim.GetPath()}/{gripper}/left_inner_finger/tactile_zone_visuals")
        pad = asset.GetPrimAtPath(f"{prim.GetPath()}/{gripper}/Physics/GripMaterial")
        k = pad.GetAttribute("physxMaterial:compliantContactStiffness").Get() if pad else None
        if method is None:
            assert not markers, "SensorSim=None should have no taxel grid"
            continue
        attrs = {a.GetName().split(":")[-1]: a.Get() for a in markers.GetAttributes()
                 if a.GetName().startswith("robotiq:tactile:")}
        assert attrs["method"] == method, f"{variant}: {attrs}"
        assert len([c for c in markers.GetChildren() if c.GetName().startswith("zone_")]) == ROWS * COLS
        assert tuple(attrs["sigmaMm"]) == pytest.approx((1.5, 1.25)) and attrs["eta"] == pytest.approx(0.7) \
            and attrs["gamma"] == pytest.approx(0.5), attrs
        if method == "raycast":
            assert k and k > 0, "the raycast variant needs the compliant pad"
            assert {"rayStandoff", "rayMaxDistance", "padStiffness"} <= set(attrs)
        else:
            assert not k, f"the effective-response variant should keep the pad rigid (stiffness {k})"


@pytest.mark.parametrize(("gripper", "physics"), RAYCAST_CASES)
def test_raycast_and_effective_agree(simulation_app, view, gripper: str, physics: str) -> None:
    """Both methods return the same kind of values: on one Raycast-variant simulation (compliant pad)
    the raycast force map and response r match the contact-based ones (Eq. 1 on the contact patch)."""
    rig = _load(simulation_app, view, gripper, "raycast", physics, extra="effective")
    step(simulation_app, SETTLE_FRAMES)
    ray, eff = rig.sensor, rig.extra
    _squeeze(simulation_app, rig)  # close and hold
    acc = {k: {s: [] for s in SIDES} for k in ("ray_force", "ray_r", "eff_force", "eff_r")}
    pad = {s: [] for s in SIDES}
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        for k, fn in (("ray_force", ray.read_force), ("ray_r", ray.read_response),
                      ("eff_force", eff.read_force), ("eff_r", eff.read_response)):
            for s, g in fn().items():
                acc[k][s].append(g)
        for s in SIDES:
            pad[s].append(eff.last_force[s])
    omni.timeline.get_timeline_interface().stop()
    lo, hi = MODELS["raycast"].sum_band
    for s in SIDES:
        rf, ef = (np.mean(acc[k][s], axis=0) for k in ("ray_force", "eff_force"))
        rr, er = (np.mean(acc[k][s], axis=0) for k in ("ray_r", "eff_r"))
        total = float(rf.sum()) / float(np.mean(pad[s]))
        r_err = float(np.abs(rr - er).mean() / er.max())
        f_corr = float(np.corrcoef(rf.ravel(), ef.ravel())[0, 1])
        print(f"{gripper} {physics} {s}: raycast force x{total:.2f} of the pad force, force-map correlation {f_corr:.2f}; "
              f"response peak raycast {rr.max():.3f} vs effective {er.max():.3f}, mean |difference| {r_err:.1%}")
        assert lo <= total <= hi, f"{s}: the raycast force is x{total:.2f} of the pad force"
        assert f_corr >= FORCE_CORR, f"{s}: the force maps disagree (correlation {f_corr:.2f})"
        assert r_err <= RAY_RESPONSE_TOL, f"{s}: the responses differ by {r_err:.0%} of the peak"
