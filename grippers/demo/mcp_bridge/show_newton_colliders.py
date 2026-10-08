"""Overlay the Newton (MuJoCo-Warp) collision shapes of the mounted gripper in the viewport.

PhysX has a built-in collision-mesh visualization (the viewport's Show -> Physics / the Physics
debug window); the Newton backend has none. This draws the shapes Newton actually collides with --
the convex hulls / decompositions it built from the `mesh_0` colliders -- as a live wireframe, so
you can check what a grasp is really touching.

Nothing is authored on the stage (a stage edit can make Newton rebuild its model); it uses the
`isaacsim.util.debug_draw` line overlay and the model's own shape data, redrawn every frame from
the current body poses. Run it inside Isaac (Script Editor, or `simulation.execute_script` over
the MCP bridge) while the sim is playing under the Newton experience:

    exec(open(".../grippers/demo/mcp_bridge/show_newton_colliders.py").read())

Stop it with:

    import carb; carb._newton_collider_viz["stop"]()

By default it draws the gripper (any shape whose prim path contains "Robotiq_"); set
`MATCH = None` before exec'ing to draw every mesh shape (arm and gripper), or another substring.
"""
import carb
import numpy as np
import omni.kit.app

MATCH = globals().get("MATCH", "Robotiq_")
COLOR = globals().get("COLOR", (0.1, 1.0, 0.2, 1.0))   # RGBA
WIDTH = globals().get("WIDTH", 1.5)

try:
    carb._newton_collider_viz["stop"]()
except Exception:
    pass

import isaacsim.physics.newton as _nx
from newton import ShapeFlags as _ShapeFlags
from isaacsim.util.debug_draw import _debug_draw

_dd = _debug_draw.acquire_debug_draw_interface()
_cache = {}   # shape id -> (local vertices, unique edge index pairs)


def _quat_rotate(q, v):
    """Rotate Nx3 `v` by the xyzw quaternion `q`."""
    x, y, z, w = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _edges(src):
    tri = np.asarray(src.indices).reshape(-1, 3)
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
    return np.unique(np.sort(e, axis=1), axis=0)


def _shapes(model):
    labels = list(model.shape_label)
    flags = model.shape_flags.numpy()
    out = []
    for i, lab in enumerate(labels):
        if not int(flags[i]) & int(_ShapeFlags.COLLIDE_SHAPES):
            continue                       # visual-only shape (e.g. a fingertip's render mesh)
        src = model.shape_source[i]
        if src is None or not hasattr(src, "vertices"):
            continue                       # primitives (box/cylinder/plane) carry no mesh
        if MATCH is not None and MATCH not in lab:
            continue
        if i not in _cache:
            _cache[i] = (np.asarray(src.vertices, dtype=np.float64), _edges(src))
        out.append(i)
    return out


def _draw(_event=None):
    try:
        ns = _nx.acquire_stage()
        model, state = ns.model, ns.state_0
        if model is None or state is None:
            return
        body_q = state.body_q.numpy()
        shape_body = model.shape_body.numpy()
        shape_tf = model.shape_transform.numpy()
        scale = model.shape_scale.numpy()
        starts, ends = [], []
        for i in _shapes(model):
            verts, edges = _cache[i]
            v = verts * scale[i]
            v = _quat_rotate(shape_tf[i][3:7], v) + shape_tf[i][0:3]      # shape -> body
            b = int(shape_body[i])
            if b >= 0:
                v = _quat_rotate(body_q[b][3:7], v) + body_q[b][0:3]      # body -> world
            starts.append(v[edges[:, 0]])
            ends.append(v[edges[:, 1]])
        _dd.clear_lines()
        if starts:
            s = np.concatenate(starts)
            e = np.concatenate(ends)
            _dd.draw_lines(s.tolist(), e.tolist(), [COLOR] * len(s), [WIDTH] * len(s))
    except Exception:
        import traceback
        carb.log_warn("show_newton_colliders: " + traceback.format_exc())


_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    _draw, name="show_newton_colliders")


def _stop():
    global _sub
    _sub = None
    _dd.clear_lines()
    print("show_newton_colliders: stopped")


carb._newton_collider_viz = {"sub": _sub, "stop": _stop}
print("show_newton_colliders: drawing %d shape(s) (MATCH=%r)" % (len(_shapes(_nx.acquire_stage().model)), MATCH))
