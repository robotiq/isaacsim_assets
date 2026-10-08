"""Session-wide Isaac Sim app for the runtime tests.

The app is created in ``pytest_configure`` (before collection) because every
``isaacsim.*`` import in the test modules needs a running Kit. It is closed in
``pytest_unconfigure``.

Options (see README.md):
  --gui        open the Isaac Sim window instead of running headless
  --hold       with --gui, keep the window open after the last test until closed
  --save-view  with --gui --hold, write the camera pose + lights to the last
               test's preset in views/ when the window is closed
"""

from __future__ import annotations

import pytest

_app = None


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption("--gui", action="store_true", default=False, help="run with the Isaac Sim window (default: headless)")
    parser.addoption("--hold", action="store_true", default=False, help="with --gui, keep the window open after the tests")
    parser.addoption(
        "--save-view", nargs="?", const="case", default=None, choices=["case", "test", "gripper", "default"],
        metavar="SCOPE",
        help="with --gui --hold, save the viewport camera + lights of the last test to views/ on window close. "
             "SCOPE picks the file: case (default: this parametrized case), test (every case of the test "
             "function), gripper (every test of that gripper), default (every test)",
    )
    parser.addoption(
        "--screenshot", default=None, metavar="DIR",
        help="capture the viewport to DIR/<test>.png at the end of each test (works headless too)",
    )
    parser.addoption("--no-view", action="store_true", default=False, help="do not apply the saved view presets")
    parser.addoption(
        "--show-joints", action="store_true", default=False,
        help="with --gui, draw the physics joints in the viewport (the eye menu's Physics > Joints)",
    )
    parser.addoption(
        "--port", type=int, default=8777,
        help="with --gui --hold, the localhost port on which the held session accepts test runs (isaac_run.py)",
    )
    parser.addoption(
        "--slow", type=float, default=1.0, metavar="FACTOR",
        help="with --gui, play FACTOR times slower than real time (each 60 Hz frame is held on screen "
             "FACTOR/60 s); the physics is unchanged",
    )


def pytest_configure(config: pytest.Config) -> None:
    global _app
    from isaacsim import SimulationApp

    # Always "RTX - Minimal": the tests are about physics; keep the viewport cheap.
    # minimal_shading_mode 2 (Textured Diffuse) is Kit's own default for
    # /rtx/minimal/mode, i.e. what the viewport menu's "RTX - Minimal" shows in
    # the interactive app. SimulationApp would otherwise force mode 0 ("Real-Time
    # 2.0 reference"), which renders a black viewport here.
    _app = SimulationApp({
        "headless": not config.getoption("--gui"),
        "renderer": "MinimalRendering",
        "minimal_shading_mode": 2,
    })
    # Newton is not loaded by the Python experience: enable the engine and its
    # native tensor backend (what Articulation reads DOFs through). Tests pick
    # the engine per case with SimulationManager.switch_physics_engine.
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.physics.newton", "isaacsim.physics.newton.tensors"):
        ext_manager.set_extension_enabled_immediate(ext, True)
    _app.update()
    # --slow: hold each frame on screen longer. Only the pacing changes; the
    # tests still advance one 60 Hz physics frame per update(). The factor is
    # mutable so the hold server can set it per run.
    import time

    real_update = _app.update
    pacing = {"factor": config.getoption("--slow") if config.getoption("--gui") else 1.0}

    def paced_update() -> None:
        real_update()
        if pacing["factor"] > 1.0:
            time.sleep((pacing["factor"] - 1.0) / 60.0)

    _app.update = paced_update
    config._isaac_pacing = pacing
    if config.getoption("--gui") and config.getoption("--show-joints"):
        _show_joints(True)
    config._isaac_app = _app
    config._isaac_last_nodeid = None


def _show_joints(on: bool) -> None:
    """Draw (or hide) the physics joints in the viewport: PhysX's joint
    visualizer, driven by a persistent setting; it reads the UsdPhysics joint
    prims, so it works on Newton stages too."""
    import carb.settings
    import omni.kit.app

    # The visualizer lives in the physics UI extensions, which the Python
    # experience does not load (the full app gets them via omni.physx.bundle).
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.usdphysics.ui", "omni.physx.ui"):
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)
    _app.update()
    carb.settings.get_settings().set("/persistent/physics/visualizationDisplayJoints", bool(on))
    print(f"joint visualization {'on' if on else 'off'}")


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    # Everything collected, before -k deselects: the hold server runs any of them.
    config._isaac_all_items = list(items)


def pytest_runtest_setup(item: pytest.Item) -> None:
    item.config._isaac_last_nodeid = item.nodeid
    item.config._isaac_last_item = item


def _reload_asset_layers() -> None:
    """Re-read the gripper USD layers from disk. USD keeps opened layers cached
    for the life of the process, so without this a replay after editing an
    asset file would compose the old content."""
    from pathlib import Path

    from pxr import Sdf

    grippers = str(Path(__file__).resolve().parents[2] / "grippers")
    n = 0
    for layer in Sdf.Layer.GetLoadedLayers():
        if layer.realPath and layer.realPath.startswith(grippers):
            layer.Reload(force=True)
            n += 1
    print(f"reloaded {n} asset layer(s) from disk")


def _select(items: list[pytest.Item], expr: str) -> list[pytest.Item]:
    """Items whose id contains every word of ``expr`` (words joined by ' and ';
    'not <word>' excludes). A simplified pytest -k."""
    terms = [t.strip() for t in expr.split(" and ") if t.strip()]
    out = []
    for it in items:
        ok = True
        for t in terms:
            if t.startswith("not "):
                ok &= t[4:].strip() not in it.nodeid
            else:
                ok &= t in it.nodeid
        if ok:
            out.append(it)
    return out


def _run_items(config: pytest.Config, items: list[pytest.Item], slow: float | None) -> str:
    """Run ``items`` in this session (asset layers re-read from disk first) and
    return everything they printed."""
    import io
    import sys

    class Tee(io.TextIOBase):
        def __init__(self, *streams):
            self.streams = streams

        def write(self, s):
            for st in self.streams:
                st.write(s)
            return len(s)

        def flush(self):
            for st in self.streams:
                st.flush()

    buf = io.StringIO()
    pacing = config._isaac_pacing
    previous = pacing["factor"]
    if slow is not None:
        pacing["factor"] = slow
    real_stdout = sys.stdout
    sys.stdout = Tee(real_stdout, buf)
    try:
        _reload_asset_layers()
        for it in items:
            print(f"\n--- {it.nodeid}")
            config.hook.pytest_runtest_protocol(item=it, nextitem=None)
    finally:
        sys.stdout = real_stdout
        pacing["factor"] = previous
    return buf.getvalue()


def _hold(config: pytest.Config) -> None:
    """Keep the window open until it is closed. Meanwhile:
    - R in the Isaac window replays the last test;
    - a line of JSON on 127.0.0.1:<--port> runs any collected tests, e.g.
      {"k": "2F_85 and Newton_compliant", "slow": 5} (see isaac_run.py), and
      gets their output back. Asset layers are re-read from disk before each
      run, so asset edits need no restart (test code edits still do)."""
    import json
    import socket

    import carb.input
    import omni.appwindow

    all_items = getattr(config, "_isaac_all_items", [])
    last = getattr(config, "_isaac_last_item", None)
    pacing = config._isaac_pacing
    hold_factor, pacing["factor"] = pacing["factor"], 1.0  # the hold loop itself stays responsive
    replay = {"requested": False}

    def on_key(event, *_):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input == carb.input.KeyboardInput.R:
            replay["requested"] = True
        return True

    inputs = carb.input.acquire_input_interface()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    sub = inputs.subscribe_to_keyboard_events(keyboard, on_key)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", config.getoption("--port")))
    server.listen(1)
    server.setblocking(False)
    print(f"--hold: serving test runs on 127.0.0.1:{config.getoption('--port')} (tests/runtime/isaac_run.py)")
    try:
        while _app.is_running():
            _app.update()
            if replay["requested"] and last is not None:
                replay["requested"] = False
                print(f"\nreplaying {last.nodeid}")
                _run_items(config, [last], hold_factor)
                continue
            try:
                conn, _ = server.accept()
            except BlockingIOError:
                continue
            with conn:
                conn.settimeout(5.0)
                data = b""
                while not data.endswith(b"\n"):
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                try:
                    req = json.loads(data.decode() or "{}")
                    if "joints" in req:
                        _show_joints(req["joints"])
                    if not req.get("k"):
                        reply = {"output": ""}
                        conn.sendall((json.dumps(reply) + "\n").encode())
                        continue
                    items = _select(all_items, req.get("k", ""))
                    if not items:
                        reply = {"error": f"no test matches {req.get('k', '')!r}", "available": [i.nodeid for i in all_items]}
                    else:
                        reply = {"output": _run_items(config, items, req.get("slow"))}
                        last = items[-1]
                except Exception as e:  # report, keep serving
                    reply = {"error": f"{type(e).__name__}: {e}"}
                conn.sendall((json.dumps(reply) + "\n").encode())
    finally:
        server.close()
        inputs.unsubscribe_to_keyboard_events(keyboard, sub)
        pacing["factor"] = hold_factor


def pytest_unconfigure(config: pytest.Config) -> None:
    global _app
    if _app is None:
        return
    gui = config.getoption("--gui")
    if gui and config.getoption("--hold"):
        msg = "press R in the Isaac Sim window to replay the last test, close the window to finish"
        if config.getoption("--save-view"):
            msg += " and save the view"
        print(f"\n--hold: {msg}.")
        _hold(config)
        if config.getoption("--save-view") and config._isaac_last_nodeid:
            from isaacsim.core.experimental.utils import stage as stage_utils

            from _views import save_view

            scope = config.getoption("--save-view")
            path = save_view(config._isaac_last_nodeid, stage_utils.get_current_stage(), scope)
            print(f"view saved ({scope}): {path}")
    _app.close()
    _app = None


@pytest.fixture(scope="session")
def simulation_app(request: pytest.FixtureRequest):
    return request.config._isaac_app


@pytest.fixture
def view(request: pytest.FixtureRequest):
    """Call ``view()`` once the stage is populated: in --gui (or --screenshot)
    runs it applies the test's saved camera + lights preset (if any); otherwise
    it is a no-op. With --screenshot the viewport is captured after the test."""
    config = request.config
    visual = (config.getoption("--gui") or config.getoption("--screenshot")) and not config.getoption("--no-view")

    def _apply() -> None:
        if not visual:
            return
        from isaacsim.core.experimental.utils import stage as stage_utils

        from _views import apply_view

        path = apply_view(request.node.nodeid, stage_utils.get_current_stage())
        if path:
            print(f"view applied: {path.name}")

    yield _apply

    shot_dir = config.getoption("--screenshot")
    if shot_dir:
        from pathlib import Path

        from _views import preset_paths, capture_viewport

        out = Path(shot_dir) / (preset_paths(request.node.nodeid)[0].stem + ".png")
        capture_viewport(config._isaac_app, out)
        print(f"screenshot: {out}")
