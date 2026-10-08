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
        "--save-view", action="store_true", default=False,
        help="with --gui --hold, save the viewport camera + lights of the last test to views/ on window close",
    )
    parser.addoption(
        "--screenshot", default=None, metavar="DIR",
        help="capture the viewport to DIR/<test>.png at the end of each test (works headless too)",
    )
    parser.addoption("--no-view", action="store_true", default=False, help="do not apply the saved view presets")


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
    config._isaac_app = _app
    config._isaac_last_nodeid = None


def pytest_runtest_setup(item: pytest.Item) -> None:
    item.config._isaac_last_nodeid = item.nodeid


def pytest_unconfigure(config: pytest.Config) -> None:
    global _app
    if _app is None:
        return
    gui = config.getoption("--gui")
    if gui and config.getoption("--hold"):
        msg = "close the Isaac Sim window to finish"
        if config.getoption("--save-view"):
            msg += " and save the view"
        print(f"\n--hold: {msg}.")
        while _app.is_running():
            _app.update()
        if config.getoption("--save-view") and config._isaac_last_nodeid:
            from isaacsim.core.experimental.utils import stage as stage_utils

            from _views import save_view

            path = save_view(config._isaac_last_nodeid, stage_utils.get_current_stage())
            print(f"view saved: {path}")
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
