"""The 2F-85's TSF-85 tactile sensor: both ``SensorSim`` models under the same tests.

Every common test runs once per model on the force map (``read_force()``, N per taxel) against the same
baseline, the pad force of the contact report (``ContactReader``) averaged over the same frames; only the
passing thresholds differ (``MODELS``). Model-specific tests cover the raycast depth, the effective response
r against Eq. 1, and the two models agreeing on one simulation. What they check is in tests/runtime/README.md.

Most tests close the gripper on a static box (fixed_cubes/Robotiq_2F_85_tactile.json) and average the last
frames. On a fixed rigid box the load split between the two fingers is ill-conditioned (it follows sub-0.1 mm
penetration differences, e.g. 284 / 111 N, or 0 / 399 N with the box shifted 2 mm), so every assertion is per
pad, never left against right. A free cube on its holder, as in test_gripper_grasp.py, would split it as a grasp does.
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
from _tactile import COLS, EFFECTIVE_VARIANTS, RAYCAST_VARIANTS, ROWS, SIDES, TactilePlot
from tactile import ContactPointReader, EffectiveTactile, RaycastTactile, make_tactile_sensor, markers_path
from tactile.effective_response import GAMMA, effective_response, pad_plane

GRIPPER = "Robotiq_2F_85"
DRIVEN = "finger_joint"
PADS = {"left": "left_fingertip", "right": "right_fingertip"}
TARGET = 0.8  # rad, full close: beyond the box
SETTLE_FRAMES = 30
HOLD_FRAMES = 60
SAMPLE_FRAMES = 20
PHYSX_HZ = 1000  # as the fixed-cube test
EFFORTS = (5.0, 20.0)  # finger_joint max effort for the weak / strong grip (stock 26)

TACTILE_PRESET = f"{GRIPPER}_tactile"  # a box over part of the sensing area, clear of the finger links


@dataclass(frozen=True)
class Model:
    """One sensor method and what it is held to. The tests are the same for every model; only
    these thresholds differ."""

    variants: list  # Fingertip / SensorSim selections
    sensor_class: type
    physics: tuple  # the Physics variants it runs on
    sum_band: tuple  # force-map total / contact-report pad force
    scale_tol: float  # (map total ratio) / (pad force ratio) within 1 +- this, weak -> strong grip
    pad_compliant: bool  # the variant softens the pad (the raycast needs it) or leaves it rigid


MODELS = {
    # the depth under-reads at weak grips (x0.7 at 20 N) and on a 240 Hz step, and the pad force varies run to run
    "raycast": Model(RAYCAST_VARIANTS, RaycastTactile, ("Physx_parallel_grip", "Physx_compliant"),
                     (0.65, 1.15), 0.3, True),
    "effective": Model(EFFECTIVE_VARIANTS, EffectiveTactile,
                       ("Physx_parallel_grip", "Physx_compliant", "Newton_parallel_grip", "Newton_compliant"),
                       (0.9, 1.1), 0.15, False),
}
SENSORS = list(MODELS)
SENSOR_IDS = [f"{GRIPPER}-{s}" for s in SENSORS]
CASES = [(s, p) for s in SENSORS for p in MODELS[s].physics]
CASE_IDS = [f"{GRIPPER}-{s}-{p}" for s, p in CASES]
SCALE_CASES = [(s, p) for s, p in CASES if p in ("Physx_parallel_grip", "Newton_parallel_grip")]
SCALE_IDS = [f"{GRIPPER}-{s}-{p}" for s, p in SCALE_CASES]
EFFECTIVE_SCALE = [(c, i) for c, i in zip(SCALE_CASES, SCALE_IDS) if c[0] == "effective"]
RATE_PHYSICS = "Physx_parallel_grip"  # the rate tests set PhysX time steps


def _at_60hz(sensor: str, *params, hz: int = 60):
    """A rate case; the raycast method fails it at 60 Hz (strict xfail): PhysX does not resolve the soft pad's
    compliant contact at that step, so the box does not sink and the depth is ~0 (force map x0.00-0.29 of the
    pad force, x0.82 at 240 Hz). The effective response reads the contact report, which is fine at 60 Hz."""
    marks = []
    if sensor == "raycast" and hz == 60:
        marks.append(pytest.mark.xfail(strict=True, reason="the compliant-pad depth is ~0 at a 60 Hz PhysX step"))
    return pytest.param(sensor, *params, marks=marks, id="-".join([f"{GRIPPER}-{sensor}", *map(str, params)]))


DEFAULT_RATE_CASES = [_at_60hz(s, obj) for s in SENSORS for obj in ("static", "kinematic")]
FAST_CLOSE_CASES = [_at_60hz(s, hz, hz=hz) for s in SENSORS for hz in (60, 240)]

FORCE_ACTIVE_N = 0.1  # a taxel "carries force" above this
PAD_MIN_N = 1.0  # each pad pushes at least this hard on the box
FOOTPRINT_MARGIN = 0.0025  # m: taxels this close to the box edge are not judged as covered
CLEAR_SHARE = 0.05  # at most this share of the force is on taxels clear of the box
PEAK_BOUND = 1.15  # no taxel carries more than this times the pad force

ACTIVE_MM = 0.05  # a taxel "reads" above this depth
MM_PER_N = (0.003, 0.05)  # depth per newton of pad force, around the pad spring's 0.012
# the TSF-85 soft layer is 3 mm thick: deeper and the object would be through it
MAX_SINK_MM = 3.0
ACTIVE_R = 0.03  # a taxel "reads" above this response
MODEL_TOL = 0.35  # mean |r - Eq. 1 on the ideal footprint| within this fraction of the peak
POWER_TOL = 0.25  # r ratio within 25 % of (force ratio) ** GAMMA
UNIFORM_SLACK = 1.15  # r never exceeds (F / box area) ** GAMMA, within this slack
RIGID_DEPTH_MM = 0.3  # the raycast reader on the rigid pad reads below this
FORCE_CORR = 0.95  # correlation of the two force maps
RAY_RESPONSE_TOL = 0.10  # mean |r raycast - r effective| within this fraction of the peak


def _box() -> tuple[np.ndarray, np.ndarray]:
    """(size, centre) of the fixed box."""
    preset = load_preset(TACTILE_PRESET, fixed=True)
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


def _load(simulation_app, view, sensor: str, physics: str, hz: int | None = PHYSX_HZ, kinematic: bool = False,
          extra: str | None = None) -> Rig:
    """Fresh stage with the box and ``sensor``'s variant. ``extra``: "depth" adds a raycast depth reader
    (on the effective variant's rigid pad), "effective" an effective-response reader (on the raycast
    variant's compliant pad)."""
    model = MODELS[sensor]
    engine = engine_for(physics)
    built = {}

    def setup(stage):
        box = add_fixed_cube(stage, TACTILE_PRESET)
        if kinematic:  # a kinematic rigid body instead of a static collider
            UsdPhysics.RigidBodyAPI.Apply(box.GetPrim()).CreateKinematicEnabledAttr().Set(True)
        if hz and engine == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(hz)
        root = f"/World/Gripper/{GRIPPER}"
        contacts = ContactReader(GRIPPER, root, CUBE_PATH, engine)
        contacts.prepare(stage)
        built["contacts"] = contacts
        built["sensor"] = make_tactile_sensor(stage, root, engine, GRIPPER, CUBE_PATH)  # quantity: force
        assert isinstance(built["sensor"], model.sensor_class), type(built["sensor"])
        if extra == "depth":
            built["extra"] = RaycastTactile(root, "depth")
            built["extra"].prepare(stage)
        elif extra == "effective":
            built["extra"] = EffectiveTactile(root, ContactPointReader(GRIPPER, root, engine, CUBE_PATH), "response")
            built["extra"].prepare(stage)

    scene = load_gripper(simulation_app, GRIPPER, physics, view, setup=setup, variants=model.variants)
    if _plot:
        _plot.show(built["sensor"])
    return Rig(scene, built["sensor"], built["contacts"], built.get("extra"))


@dataclass
class Squeeze:
    force: dict  # {side: (ROWS, COLS)} mean taxel force, N
    pad: dict  # {side: mean pad force of the contact report, N}
    others: dict  # {link: peak force N} of the links other than the pads that touch the box
    angles: np.ndarray  # finger_joint, deg, per frame
    totals: np.ndarray  # force-map total of both pads, per frame
    points: dict  # {side: (ROWS, COLS, 3)} taxel positions at the stall
    maps: dict  # {quantity: {side: grid}}, the ``also`` quantities


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
    others, angles, totals = {}, [], []
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
    mean = lambda d: {s: np.mean(v, axis=0) for s, v in d.items()}
    return Squeeze(mean(force), {s: float(np.mean(v)) for s, v in pad.items()}, others, np.array(angles),
                   np.array(totals), sensor.taxel_points(), {q: mean(m) for q, m in maps.items()})


def _print(label: str, sq: Squeeze) -> None:
    np.set_printoptions(precision=2, suppress=True)
    for s in SIDES:
        g = sq.force[s]
        print(f"{label} {s}: pad force {sq.pad[s]:.1f} N, map {g.sum():.1f} N (x{g.sum() / max(sq.pad[s], 1e-9):.2f}), "
              f"{int((g > FORCE_ACTIVE_N).sum())}/{ROWS * COLS} taxels, peak {g.max():.1f} N\n{g}")


def _finger_limit_deg(stage) -> float:
    joint = next(p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Gripper")) if p.GetName() == DRIVEN)
    return float(joint.GetAttribute("physics:upperLimit").Get())


def _assert_pad_forces(model: Model, sq: Squeeze, label: str) -> None:
    """The common baseline check: the force map carries the pad force of the contact report."""
    for s in SIDES:
        total = float(sq.force[s].sum())
        assert np.isfinite(sq.force[s]).all() and (sq.force[s] >= 0).all(), f"{label} {s}: invalid forces"
        assert sq.pad[s] >= PAD_MIN_N, f"{label} {s}: the pad pushes with only {sq.pad[s]:.2f} N"
        ratio = total / sq.pad[s]
        assert model.sum_band[0] <= ratio <= model.sum_band[1], (
            f"{label} {s}: the force map carries {total:.1f} N of the {sq.pad[s]:.1f} N pad force (x{ratio:.2f}, "
            f"allowed {model.sum_band})")
        assert sq.force[s].max() <= PEAK_BOUND * sq.pad[s], f"{label} {s}: one taxel carries {sq.force[s].max():.1f} N"


def _taxel_masks(pitch_mm, points: np.ndarray, size, centre):
    """(covered, clear): taxels well inside the box, and taxels whose rectangle does not touch it."""
    dx = np.abs(points[..., 0] - centre[0]) - size[0] / 2.0
    dz = np.abs(points[..., 2] - centre[2]) - size[2] / 2.0
    covered = (dx <= -FOOTPRINT_MARGIN) & (dz <= -FOOTPRINT_MARGIN)
    clear = (dx >= pitch_mm[0] / 2000.0) | (dz >= pitch_mm[1] / 2000.0)
    return covered, clear


@pytest.mark.parametrize("sensor", SENSORS, ids=SENSOR_IDS)
def test_tactile_layout(simulation_app, view, sensor: str) -> None:
    model = MODELS[sensor]
    rig = _load(simulation_app, view, sensor, model.physics[0])
    stage = omni.usd.get_context().get_stage()
    for side in SIDES:
        # 7 x 4 taxels, each (row, col) once
        cells = sorted((r, c) for r, c, _rel, _half in rig.sensor.taxels[side])
        assert cells == [(r, c) for r in range(ROWS) for c in range(COLS)], f"{side}: taxels {cells}"
        assert stage.GetPrimAtPath(markers_path(rig.scene.root, side))
        # the pad: compliant where the method reads the depth, rigid where it does not need it
        case = stage.GetPrimAtPath(f"{rig.scene.root}/{side}_fingertip/visuals/tip_tsf85/TSF_85_case")
        assert case, f"no TSF-85 pad collider on the {side} finger"
        mat, _ = UsdShade.MaterialBindingAPI(case).ComputeBoundMaterial("physics")
        k = mat.GetPrim().GetAttribute("physxMaterial:compliantContactStiffness").Get() if mat else None
        assert bool(k and k > 0) == model.pad_compliant, f"{side} pad: compliance {k}, expected {model.pad_compliant}"
    markers = [p for side in SIDES for p in Usd.PrimRange(stage.GetPrimAtPath(markers_path(rig.scene.root, side)))]
    colliding = [str(p.GetPath()) for p in markers if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert not colliding, f"the pad must be the only collider, markers collide: {colliding[:3]}"
    omni.timeline.get_timeline_interface().stop()


@pytest.mark.parametrize(("sensor", "physics"), CASES, ids=CASE_IDS)
def test_tactile_force_footprint(simulation_app, view, sensor: str, physics: str) -> None:
    model = MODELS[sensor]
    rig = _load(simulation_app, view, sensor, physics)
    step(simulation_app, SETTLE_FRAMES)
    for s, g in rig.sensor.read().items():
        assert g.max() <= FORCE_ACTIVE_N, f"{s} carries {g.max():.2f} N with the gripper open"

    sq = _squeeze(simulation_app, rig)
    limit = _finger_limit_deg(omni.usd.get_context().get_stage())
    move_to(simulation_app, rig.scene, rig.scene.dof(DRIVEN), 0.0, settle_frames=HOLD_FRAMES)
    reopened = rig.sensor.read()
    omni.timeline.get_timeline_interface().stop()
    _print(f"{sensor} {physics}", sq)

    assert sq.angles.max() <= limit + 0.5, f"finger_joint {sq.angles.max():.1f} deg past its {limit:.0f} deg limit"
    assert not sq.others, "only the pads should touch the box, also: " + ", ".join(
        f"{k} {v:.1f} N" for k, v in sorted(sq.others.items()))
    _assert_pad_forces(model, sq, f"{sensor} {physics}")
    size, centre = _box()
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


def _two_efforts(simulation_app, view, sensor: str, physics: str, also: tuple = ()):
    """Squeeze with the weak and the strong grip: [Squeeze, Squeeze]."""
    runs = []
    for effort in EFFORTS:
        rig = _load(simulation_app, view, sensor, physics)
        rig.scene.robot.set_dof_max_efforts([effort], dof_indices=[rig.scene.dof(DRIVEN)])
        step(simulation_app, SETTLE_FRAMES)
        sq = _squeeze(simulation_app, rig, also=also)
        omni.timeline.get_timeline_interface().stop()
        _print(f"{sensor} {physics} max effort {effort}", sq)
        runs.append(sq)
    return runs


@pytest.mark.parametrize(("sensor", "physics"), SCALE_CASES, ids=SCALE_IDS)
def test_tactile_force_scales_with_grip_force(simulation_app, view, sensor: str, physics: str) -> None:
    model = MODELS[sensor]
    weak, strong = _two_efforts(simulation_app, view, sensor, physics)
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


def _assert_stable(model: Model, sq: Squeeze, label: str) -> None:
    limit = _finger_limit_deg(omni.usd.get_context().get_stage())
    print(f"{label}: finger_joint peak {sq.angles.max():.1f} deg; force-map total over frames "
          f"{sq.totals.min():.1f}..{sq.totals.max():.1f} N")
    assert sq.angles.max() <= limit + 0.5, f"finger_joint {sq.angles.max():.1f} deg past its {limit:.0f} deg limit"
    assert sq.totals.min() > 0.0, "the force map drops out on some frames"
    _assert_pad_forces(model, sq, label)


@pytest.mark.parametrize(("sensor", "obj"), DEFAULT_RATE_CASES)
def test_tactile_force_stable_at_default_rate(simulation_app, view, sensor: str, obj: str) -> None:
    rig = _load(simulation_app, view, sensor, RATE_PHYSICS, hz=None, kinematic=obj == "kinematic")
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig)
    omni.timeline.get_timeline_interface().stop()
    _print(f"{sensor} @ 60 Hz, {obj} box", sq)
    _assert_stable(MODELS[sensor], sq, f"{sensor} 60 Hz {obj}")


@pytest.mark.parametrize(("sensor", "hz"), FAST_CLOSE_CASES)
def test_tactile_force_stable_on_fast_close(simulation_app, view, sensor: str, hz: int) -> None:
    """The full-close target in one step, no ramp: the finger hits the pad at speed."""
    rig = _load(simulation_app, view, sensor, RATE_PHYSICS, hz=hz)
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, ramp=False)
    omni.timeline.get_timeline_interface().stop()
    _print(f"{sensor} fast close @ {hz} Hz", sq)
    _assert_stable(MODELS[sensor], sq, f"{sensor} fast close {hz} Hz")


@pytest.mark.parametrize("physics", MODELS["raycast"].physics, ids=[f"{GRIPPER}-{p}" for p in MODELS["raycast"].physics])
def test_raycast_depth(simulation_app, view, physics: str) -> None:
    """The depth the force is made from: millimetres per newton of pad force in the spring's range, and
    never through the soft layer."""
    rig = _load(simulation_app, view, "raycast", physics)
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, also=("depth",))
    omni.timeline.get_timeline_interface().stop()
    _print(f"raycast {physics}", sq)
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


@pytest.mark.parametrize("physics", MODELS["effective"].physics, ids=[f"{GRIPPER}-{p}" for p in MODELS["effective"].physics])
def test_effective_response_matches_eq1(simulation_app, view, physics: str) -> None:
    """r (``read_response()``) on the box is Eq. 1 of the box footprint at the measured pad force, and
    never exceeds the uniform-pressure value."""
    rig = _load(simulation_app, view, "effective", physics)
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, also=("response",))
    omni.timeline.get_timeline_interface().stop()
    size, centre = _box()
    for s in SIDES:
        r = sq.maps["response"][s]
        model = _model(sq.points[s], size, centre, sq.pad[s])
        err = float(np.abs(r - model).mean() / max(model.max(), 1e-9))
        bound = UNIFORM_SLACK * (sq.pad[s] / (size[0] * size[2] * 1e6)) ** GAMMA
        print(f"{physics} {s}: peak r {r.max():.3f} (Eq. 1 on the box {model.max():.3f}, uniform bound {bound:.3f}), "
              f"mean |r - model| {err:.1%} of the peak")
        assert err <= MODEL_TOL, f"{s}: r differs from Eq. 1 on the box footprint by {err:.0%} of the peak"
        assert r.max() <= bound, f"{s}: peak r {r.max():.3f} exceeds the uniform-pressure value {bound:.3f}"


@pytest.mark.parametrize(("sensor", "physics"), [c for c, _ in EFFECTIVE_SCALE], ids=[i for _, i in EFFECTIVE_SCALE])
def test_effective_response_power_law(simulation_app, view, sensor: str, physics: str) -> None:
    """r goes as the pad force to the power gamma (0.5), not proportionally like the force map."""
    weak, strong = _two_efforts(simulation_app, view, sensor, physics, also=("response",))
    mean_r = lambda sq: float(np.mean([g[g > ACTIVE_R].mean() for g in sq.maps["response"].values()]))
    pad = lambda sq: float(np.mean(list(sq.pad.values())))
    expected = (pad(strong) / pad(weak)) ** GAMMA
    print(f"r {mean_r(weak):.3f} -> {mean_r(strong):.3f} (x{mean_r(strong) / mean_r(weak):.2f}), pad force "
          f"x{pad(strong) / pad(weak):.2f} (** {GAMMA} = x{expected:.2f})")
    assert mean_r(strong) > mean_r(weak), "a stronger grip should respond more"
    assert abs(mean_r(strong) / mean_r(weak) / expected - 1.0) <= POWER_TOL, (
        f"r ratio {mean_r(strong) / mean_r(weak):.2f} vs (force ratio) ** {GAMMA} = {expected:.2f}")


def test_effective_rigid_pad(simulation_app, view) -> None:
    """The EffectiveResponse pad is rigid: the raycast depth reads (almost) nothing there, the force
    map (the common tests) and r still show the footprint."""
    physics = "Physx_parallel_grip"
    rig = _load(simulation_app, view, "effective", physics, extra="depth")
    step(simulation_app, SETTLE_FRAMES)
    sq = _squeeze(simulation_app, rig, also=("depth",))
    omni.timeline.get_timeline_interface().stop()
    peak = max(g.max() for g in sq.maps["depth"].values())
    print(f"raycast depth peak {peak:.2f} mm on the rigid pad, force map {sum(g.sum() for g in sq.force.values()):.0f} N")
    assert peak <= RIGID_DEPTH_MM, f"the rigid pad should read ~no depth, got {peak:.2f} mm"
    _assert_pad_forces(MODELS["effective"], sq, "effective rigid pad")


def test_sensorsim_variants(simulation_app) -> None:
    """The two methods are variants: Raycast (compliant pad, ray parameters) and EffectiveResponse
    (rigid pad), both tagging the shared taxel grid with their method and the Eq. 1 parameters."""
    asset = Usd.Stage.Open(str(REPO / "grippers" / GRIPPER / f"{GRIPPER}.usda"))
    prim = asset.GetDefaultPrim()
    prim.GetVariantSet("Fingertip").SetVariantSelection("Standard")
    assert "SensorSim" not in prim.GetVariantSets().GetNames(), "SensorSim should only exist under Tactile_TSF85"
    prim.GetVariantSet("Fingertip").SetVariantSelection("Tactile_TSF85")
    assert set(prim.GetVariantSet("SensorSim").GetVariantNames()) == {"None", "Raycast", "EffectiveResponse"}
    expected = {"None": None, "Raycast": "raycast", "EffectiveResponse": "effectiveResponse"}
    for variant, method in expected.items():
        prim.GetVariantSet("SensorSim").SetVariantSelection(variant)
        markers = asset.GetPrimAtPath(f"{prim.GetPath()}/{GRIPPER}/left_inner_finger/tactile_zone_visuals")
        pad = asset.GetPrimAtPath(f"{prim.GetPath()}/{GRIPPER}/Physics/GripMaterial")
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


@pytest.mark.parametrize("physics", MODELS["raycast"].physics, ids=[f"{GRIPPER}-{p}" for p in MODELS["raycast"].physics])
def test_raycast_and_effective_agree(simulation_app, view, physics: str) -> None:
    """Both methods return the same kind of values: on one Raycast-variant simulation (compliant pad)
    the raycast force map and response r match the contact-based ones (Eq. 1 on the contact patch)."""
    rig = _load(simulation_app, view, "raycast", physics, extra="effective")
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
        print(f"{physics} {s}: raycast force x{total:.2f} of the pad force, force-map correlation {f_corr:.2f}; "
              f"response peak raycast {rr.max():.3f} vs effective {er.max():.3f}, mean |difference| {r_err:.1%}")
        assert lo <= total <= hi, f"{s}: the raycast force is x{total:.2f} of the pad force"
        assert f_corr >= FORCE_CORR, f"{s}: the force maps disagree (correlation {f_corr:.2f})"
        assert r_err <= RAY_RESPONSE_TOL, f"{s}: the responses differ by {r_err:.0%} of the peak"
