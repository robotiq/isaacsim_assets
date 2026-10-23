"""Readers of the 2F-85's TSF-85 tactile sensor (``Fingertip=Tactile_TSF85``), one per ``SensorSim`` variant.

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

Each taxel is placed from its finger link's physics pose times its fixed marker offset, so it does not depend
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
    }
    AVAILABLE = ("force", "response")
    caption = "per taxel (row 0 = fingertip end)"

    def __init__(self, root: str, quantity: str = "force"):
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
                raise ValueError(f"no taxel markers at {markers.GetPath()}: select Fingertip=Tactile_TSF85 and a SensorSim")
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
        return {"force": self.read_force, "response": self.read_response, "depth": self.read_depth}[self.quantity]()

    def read_force(self) -> dict[str, np.ndarray]:
        raise NotImplementedError

    def read_response(self) -> dict[str, np.ndarray]:
        raise NotImplementedError

    def read_depth(self) -> dict[str, np.ndarray]:
        raise NotImplementedError(f"{type(self).__name__} has no depth map")


class RaycastTactile(TactileSensor):
    AVAILABLE = ("force", "response", "depth")
    DEFAULT_PAD_STIFFNESS = 0.43  # N / mm^3, the TSF-85 soft layer: E / thickness = 1.289 MPa / 3 mm

    def __init__(self, root: str, quantity: str = "force"):
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

    def __init__(self, root: str, contacts, quantity: str = "force"):
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


def _method(stage, root: str) -> str:
    methods = {}
    for side in SIDES:
        prim = stage.GetPrimAtPath(markers_path(root, side))
        if not prim.IsValid():
            raise ValueError(f"no taxel markers at {prim.GetPath()}: select Fingertip=Tactile_TSF85 and "
                             "SensorSim=Raycast or EffectiveResponse")
        attr = prim.GetAttribute("robotiq:tactile:method")
        methods[side] = attr.Get() if attr else None
    if len(set(methods.values())) != 1:
        raise ValueError(f"the two fingers carry different tactile methods: {methods}")
    return methods["left"]


def make_tactile_sensor(stage, root: str, engine: str = "physx", gripper: str = "Robotiq_2F_85",
                        object_path: str | None = None, quantity: str = "force") -> TactileSensor:
    """The reader for the stage's ``SensorSim`` variant, ``prepare``d. ``engine``: "physx" | "newton";
    ``object_path``: the one prim whose contacts count, or None for any (``EffectiveTactile``)."""
    method = _method(stage, root)
    if method == "raycast":
        sensor = RaycastTactile(root, quantity)
    elif method == "effectiveResponse":
        from .contact_points import ContactPointReader

        sensor = EffectiveTactile(root, ContactPointReader(gripper, root, engine, object_path), quantity)
    else:
        raise ValueError(f"unknown robotiq:tactile:method {method!r} at {root}")
    sensor.prepare(stage)
    return sensor
