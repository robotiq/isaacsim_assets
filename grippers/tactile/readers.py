"""Readers of the TSF-85 tactile sensor of the 2F grippers (``Fingertip=Tactile_TSF85`` on the 2F-85,
``Fingertip=Tactile_TSF140`` on the 2F-140), one per ``SensorSim`` variant.

The variant puts a 7 x 4 marker grid under each ``<side>_inner_finger`` and tags it with
``robotiq:tactile:method``. ``make_tactile_sensor(stage, root, engine)`` returns the reader for the stage:

    sensor = make_tactile_sensor(stage, "/World/Gripper/Robotiq_2F_85", "physx")   # before play
    grids = sensor.read()                      # each frame while playing: {"left" | "right": (7, 4)}

Both readers give the same maps: ``read_force()`` (N per taxel, the default ``read()``), ``read_response()``
(the effective contact response r, see effective_response.py) and, ``RaycastTactile`` only, ``read_depth()``
(mm). ``sensor.quantity`` picks what ``read()`` returns.

- ``RaycastTactile`` (``SensorSim=Raycast``, PhysX): a scene-query ray per taxel along the pad normal gives
  the depth of the gripped object past the pad surface; the pad's spring (``padStiffness``, N/mm^3) turns it
  into a pressure, then a force and, through Eq. 1, r. Needs the compliant pad of the variant and a PhysX step
  of at least 240 Hz (the pad is not resolved at 60 Hz).
- ``EffectiveTactile`` (``SensorSim=EffectiveResponse``, PhysX and Newton): the pad's contact points and
  normal forces (contact_points.py) give the force (spread over the contact patch) and r. Contact reports must
  be enabled before play (``prepare``).

- ``PreciseTactile`` (``SensorSim=Precise``, PhysX only): the deformable TSF-85 pad of the ETS CoRo lab and its CNN
  (precise.py). ``read()`` is the network's 7 x 4 map (``quantity="cnn"``), 0 at rest, in its own units.

For the first two, each taxel is placed from its finger link's physics pose times its fixed marker offset, so it does not depend
on USD being updated while playing.
"""

from __future__ import annotations

import numpy as np

ROWS, COLS = 7, 4
SIDES = {"left": 1.0, "right": -1.0}  # the inward contact normal in each inner finger's local frame (+/- Y)
PAD_LINKS = {"left": "left_fingertip", "right": "right_fingertip"}


def markers_path(root: str, side: str) -> str:
    return f"{root}/{side}_inner_finger/tactile_zone_visuals"


class TactileSensor:
    # (heatmap title, unit, smallest full scale so noise does not fill the colours) per quantity
    QUANTITIES = {
        "force": ("TSF-85 tactile force per taxel [N] (simulated, uncalibrated)", "N", 1.0),
        "response": ("TSF-85 effective contact response r (simulated, uncalibrated)", "", 0.05),
        "depth": ("TSF-85 tactile depth [mm] (simulated, uncalibrated)", "mm", 0.5),
        "cnn": ("TSF-85 CNN tactile map (simulated, uncalibrated, network units)", "", 50.0),
    }
    AVAILABLE = ("force", "response")
    DEFAULT_QUANTITY = "force"
    caption = "per taxel (row 0 = fingertip end)"

    def __init__(self, root: str, quantity: str | None = None):
        quantity = quantity or self.DEFAULT_QUANTITY
        if quantity not in self.AVAILABLE:
            raise ValueError(f"{type(self).__name__} has no {quantity!r} map (available: {self.AVAILABLE})")
        self.root = root  # articulation root, e.g. /World/Gripper/Robotiq_2F_85
        self.quantity = quantity
        self.taxels: dict[str, list] = {}  # side -> [(row, col, marker-to-finger matrix, half thickness)]
        self.params: dict = {}
        self._fingers = None

    title = property(lambda self: self.QUANTITIES[self.quantity][0])
    unit = property(lambda self: self.QUANTITIES[self.quantity][1])
    floor = property(lambda self: self.QUANTITIES[self.quantity][2])

    def prepare(self, stage) -> None:
        """Once the gripper is loaded, before play."""
        from pxr import Gf, Usd, UsdGeom

        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        params = {}
        for side in SIDES:
            markers = stage.GetPrimAtPath(markers_path(self.root, side))
            if not markers.IsValid():
                raise ValueError(f"no taxel markers at {markers.GetPath()}: select the TSF fingertip (Tactile_TSF85 / "
                                 "Tactile_TSF140) and a SensorSim")
            finger_inv = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{self.root}/{side}_inner_finger")).GetInverse()
            cells = []
            for prim in markers.GetChildren():
                if not prim.GetName().startswith("zone_"):
                    continue
                r, c = (int(t[1:]) for t in prim.GetName().split("_")[1:3])  # zone_r<row>_c<col>
                rel = cache.GetLocalToWorldTransform(prim) * finger_inv  # marker -> finger (USD row vectors)
                cells.append((r, c, rel, rel.TransformDir(Gf.Vec3d(0, 0.5, 0)).GetLength()))
            self.taxels[side] = cells
            params[side] = {a.GetName().split(":")[-1]: a.Get() for a in markers.GetAttributes()
                            if a.GetName().startswith("robotiq:tactile:")}
        if params["left"] != params["right"]:
            raise ValueError(f"the two fingers' tactile parameters differ: {params}")
        self.params = params["left"]

    def _finger_frames(self) -> dict:
        from isaacsim.core.experimental.prims import RigidPrim
        from pxr import Gf

        if self._fingers is None:
            self._fingers = RigidPrim([f"{self.root}/{s}_inner_finger" for s in SIDES])
        try:
            pos, quat = (a.numpy() for a in self._fingers.get_world_poses())
        except Exception:  # the physics view is gone (Stop -> Play): rebuild on the next read
            self._fingers = None
            raise
        frames = {}
        for i, side in enumerate(SIDES):
            w, x, y, z = (float(v) for v in quat[i])
            m = Gf.Matrix4d().SetRotate(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
            m.SetTranslateOnly(Gf.Vec3d(*(float(v) for v in pos[i])))
            frames[side] = m
        return frames

    def taxel_points(self) -> dict[str, np.ndarray]:
        """World position of each taxel's front face (on the pad surface): ``(ROWS, COLS, 3)``."""
        from pxr import Gf

        out = {}
        for side, finger in self._finger_frames().items():
            pts = np.zeros((ROWS, COLS, 3))
            for r, c, rel, half in self.taxels[side]:
                m = rel * finger
                n = m.TransformDir(Gf.Vec3d(0, SIDES[side], 0)).GetNormalized()
                pts[r, c] = np.array(m.ExtractTranslation() + n * half)
            out[side] = pts
        return out

    def read(self) -> dict[str, np.ndarray]:
        """The map of ``self.quantity``, ``{side: (ROWS, COLS)}``."""
        return {"force": self.read_force, "response": self.read_response, "depth": self.read_depth,
                "cnn": self.read_cnn}[self.quantity]()

    def read_force(self) -> dict[str, np.ndarray]:
        raise NotImplementedError

    def read_response(self) -> dict[str, np.ndarray]:
        raise NotImplementedError

    def read_depth(self) -> dict[str, np.ndarray]:
        raise NotImplementedError(f"{type(self).__name__} has no depth map")

    def read_cnn(self) -> dict[str, np.ndarray]:
        raise NotImplementedError(f"{type(self).__name__} has no CNN map")


class RaycastTactile(TactileSensor):
    AVAILABLE = ("force", "response", "depth")
    DEFAULT_PAD_STIFFNESS = 0.43  # N / mm^3, the TSF-85 soft layer: E / thickness = 1.289 MPa / 3 mm

    def __init__(self, root: str, quantity: str | None = None):
        super().__init__(root, quantity)
        self.pad_force = {s: 0.0 for s in SIDES}  # N, the force map's total at the last read_force()

    def _pressure(self, depth: dict[str, np.ndarray]):
        """Per side ``(pressure N/mm^2, taxel uv mm, pitch mm)``: ``padStiffness * depth``."""
        from .effective_response import pad_plane

        k = float(self.params.get("padStiffness", self.DEFAULT_PAD_STIFFNESS))
        for side, pts in self.taxel_points().items():
            _o, _u, _v, uv, pitch = pad_plane(pts)
            yield side, k * depth[side], uv, pitch

    def read_force(self) -> dict[str, np.ndarray]:
        out = {}
        for side, pressure, _uv, pitch in self._pressure(self.read_depth()):
            out[side] = pressure * pitch[0] * pitch[1]
            self.pad_force[side] = float(out[side].sum())
        return out

    def read_response(self) -> dict[str, np.ndarray]:
        from .effective_response import ETA, GAMMA, SIGMA_MM, response_from_taxel_pressure

        sigma = tuple(float(x) for x in self.params.get("sigmaMm", SIGMA_MM))
        eta, gamma = float(self.params.get("eta", ETA)), float(self.params.get("gamma", GAMMA))
        return {side: response_from_taxel_pressure(pressure, uv, pitch, sigma, eta, gamma)
                for side, pressure, uv, pitch in self._pressure(self.read_depth())}

    def read_depth(self) -> dict[str, np.ndarray]:
        import omni.physx
        from pxr import Gf

        standoff = float(self.params.get("rayStandoff", 0.02))  # m, rays start this far behind the taxel
        max_dist = float(self.params.get("rayMaxDistance", 0.06))
        sq = omni.physx.get_physx_scene_query_interface()
        out = {}
        for side, finger in self._finger_frames().items():
            grid = np.zeros((ROWS, COLS))
            for r, c, rel, half in self.taxels[side]:
                m = rel * finger
                p = m.ExtractTranslation()
                n = m.TransformDir(Gf.Vec3d(0, SIDES[side], 0)).GetNormalized()
                o = p - n * standoff
                hits = []
                sq.raycast_all((o[0], o[1], o[2]), (n[0], n[1], n[2]), max_dist,
                               lambda h: hits.append((h.distance, h.collision)) or True)
                obj = [d for d, col in hits if col and not col.startswith(self.root + "/")]  # skip the gripper
                if obj:
                    grid[r, c] = max(0.0, standoff + half - min(obj)) * 1000.0
            out[side] = grid
        return out


class EffectiveTactile(TactileSensor):
    """``last_force`` is the pad force (N) behind the last read. ``contacts``: a ``ContactPointReader``."""

    def __init__(self, root: str, contacts, quantity: str | None = None):
        super().__init__(root, quantity)
        self.contacts = contacts
        self.last_force = {s: 0.0 for s in SIDES}

    def prepare(self, stage) -> None:
        super().prepare(stage)
        self.contacts.prepare(stage)

    def read_both(self) -> tuple[dict, dict]:
        """``(force, response)`` maps from one contact read."""
        from .effective_response import ETA, GAMMA, SIGMA_MM, pad_maps, pad_plane

        sigma = tuple(float(x) for x in self.params.get("sigmaMm", SIGMA_MM))
        eta, gamma = float(self.params.get("eta", ETA)), float(self.params.get("gamma", GAMMA))
        points = self.contacts.read()
        force, response = {}, {}
        for side, pts in self.taxel_points().items():
            o, u, v, taxel_uv, pitch = pad_plane(pts)
            mine = [p for p in points if p.link == PAD_LINKS[side]]
            self.last_force[side] = float(sum(p.force for p in mine))
            uv = np.array([[np.dot(p.position - o, u), np.dot(p.position - o, v)] for p in mine]).reshape(-1, 2) * 1000.0
            force[side], response[side] = pad_maps(uv, self.last_force[side], taxel_uv, pitch, sigma, eta, gamma)
        return force, response

    def read_force(self) -> dict[str, np.ndarray]:
        return self.read_both()[0]

    def read_response(self) -> dict[str, np.ndarray]:
        return self.read_both()[1]


class PreciseTactile(TactileSensor):
    """``SensorSim=Precise`` (PhysX only): the deformable TSF-85 pad of the ETS CoRo lab and its CNN, see precise.py.

    ``read()`` is the 7 x 4 CNN map per finger (0 at rest, network units, not newtons). The pad nodes are read from USD
    (``simulation_mesh.points``, updated by PhysX while playing) and expressed in the frame of the rigid backing
    body, so the finger's own motion does not count. The compression at the first ``read`` (the pad at rest) is the
    zero: call ``tare()`` to set it again. Needs ``onnxruntime`` and GPU dynamics (``prepare`` switches them on)."""

    AVAILABLE = ("cnn",)
    DEFAULT_QUANTITY = "cnn"
    caption = "CNN map per taxel (row 0 = fingertip end)"
    PAD_SIGN = {"right": 1.0, "left": -1.0}  # +1 when the pad's backing is on the +y side of the finger frame

    def __init__(self, root: str, quantity: str | None = None):
        super().__init__(root, quantity)
        self.pads: dict[str, dict] = {}
        self.cnn = None
        self._backings = None
        self._zero: dict[str, np.ndarray | None] = {s: None for s in SIDES}

    def prepare(self, stage) -> None:
        import carb.settings
        from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

        from . import precise

        super().prepare(stage)
        carb.settings.get_settings().set("/physics/enableDeformableBodies", True)
        scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
        if scene is None:
            raise ValueError("no PhysicsScene: SensorSim=Precise needs one with GPU dynamics")
        api = PhysxSchema.PhysxSceneAPI.Apply(scene)  # deformable bodies run on the GPU
        api.CreateEnableGPUDynamicsAttr(True)
        api.CreateBroadphaseTypeAttr("GPU")
        asset = self.root.rsplit("/", 1)[0]  # the pads sit beside the articulation prim, under the asset root
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for side in SIDES:
            mesh = stage.GetPrimAtPath(f"{asset}/{side}_tsf_pad/simulation_mesh")
            backing = stage.GetPrimAtPath(f"{self.root}/{side}_pad_backing")
            if not mesh.IsValid() or not backing.IsValid():
                raise ValueError(f"no deformable pad at {mesh.GetPath()}: select SensorSim=Precise")
            rest = np.array(mesh.GetAttribute("omniphysics:restShapePoints").Get(), dtype=np.float64)
            pad_to_world = np.array(cache.GetLocalToWorldTransform(mesh))  # USD: row vectors, p_world = [p, 1] @ M
            back = np.array(cache.GetLocalToWorldTransform(backing))
            rest_body = precise.body_frame((np.hstack([rest, np.ones((len(rest), 1))]) @ pad_to_world)[:, :3],
                                           back[:3, :3].T, back[3, :3])
            self.pads[side] = {"mesh": mesh.GetPath().pathString, "to_world": pad_to_world, "rest_body": rest_body,
                               "grid": precise.outer_layer_grid(rest, columns_along_pad_x=side == "right")}
        self.cnn = precise.TactileCnn()

    def _backing_poses(self) -> dict:
        from isaacsim.core.experimental.prims import RigidPrim

        from . import precise

        if self._backings is None:
            self._backings = RigidPrim([f"{self.root}/{s}_pad_backing" for s in SIDES])
        try:
            pos, quat = (a.numpy() for a in self._backings.get_world_poses())
        except Exception:  # the physics view is gone (Stop -> Play): rebuild on the next read
            self._backings = None
            raise
        return {side: (precise.quat_to_matrix(quat[i]), np.asarray(pos[i], dtype=np.float64))
                for i, side in enumerate(SIDES)}

    def tare(self) -> None:
        """Take the pad's present shape as its rest (call it with the gripper open and still)."""
        self._zero = {s: None for s in SIDES}

    def compression_images(self) -> dict[str, np.ndarray]:
        """The CNN's input, per finger: ``(18, 12)`` compression in m (positive = pressed in), tared."""
        from . import precise

        stage = omni_usd_stage()
        poses = self._backing_poses()
        out = {}
        for side, pad in self.pads.items():
            pts = np.array(stage.GetPrimAtPath(pad["mesh"]).GetAttribute("points").Get(), dtype=np.float64)
            world = (np.hstack([pts, np.ones((len(pts), 1))]) @ pad["to_world"])[:, :3]
            rotation, position = poses[side]
            dz = precise.compression(precise.body_frame(world, rotation, position), pad["rest_body"],
                                     self.PAD_SIGN[side])
            if self._zero[side] is None:
                self._zero[side] = dz.copy()
            out[side] = (dz - self._zero[side])[pad["grid"]]
        return out

    def read_cnn(self) -> dict[str, np.ndarray]:
        return {side: self.cnn.read(image) for side, image in self.compression_images().items()}


def omni_usd_stage():
    import omni.usd

    return omni.usd.get_context().get_stage()


def _method(stage, root: str) -> str:
    methods = {}
    for side in SIDES:
        prim = stage.GetPrimAtPath(markers_path(root, side))
        if not prim.IsValid():
            raise ValueError(f"no taxel markers at {prim.GetPath()}: select the TSF fingertip (Tactile_TSF85 / "
                             "Tactile_TSF140) and SensorSim=Raycast, EffectiveResponse or Precise")
        attr = prim.GetAttribute("robotiq:tactile:method")
        methods[side] = attr.Get() if attr else None
    if len(set(methods.values())) != 1:
        raise ValueError(f"the two fingers carry different tactile methods: {methods}")
    return methods["left"]


def make_tactile_sensor(stage, root: str, engine: str = "physx", gripper: str | None = None,
                        object_path: str | None = None, quantity: str | None = None) -> TactileSensor:
    """The reader for the stage's ``SensorSim`` variant, ``prepare``d. ``engine``: "physx" | "newton";
    ``gripper``: e.g. "Robotiq_2F_140", by default the articulation root's name (``root`` ends with it);
    ``object_path``: the one prim whose contacts count, or None for any (``EffectiveTactile``)."""
    gripper = gripper or root.rstrip("/").rsplit("/", 1)[-1]
    method = _method(stage, root)
    if method == "raycast":
        sensor = RaycastTactile(root, quantity)
    elif method == "effectiveResponse":
        from .contact_points import ContactPointReader

        sensor = EffectiveTactile(root, ContactPointReader(gripper, root, engine, object_path), quantity)
    elif method == "precise":
        if engine != "physx":  # deformable bodies are PhysX only
            raise ValueError(f"SensorSim=Precise needs PhysX (deformable bodies), not {engine!r}")
        sensor = PreciseTactile(root, quantity)
    else:
        raise ValueError(f"unknown robotiq:tactile:method {method!r} at {root}")
    sensor.prepare(stage)
    return sensor
