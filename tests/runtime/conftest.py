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


def pytest_configure(config: pytest.Config) -> None:
    global _app
    from isaacsim import SimulationApp

    _app = SimulationApp({"headless": not config.getoption("--gui")})
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
    """Call ``view()`` once the stage is populated: in --gui runs it applies the
    test's saved camera + lights preset (if any); headless it is a no-op."""
    def _apply() -> None:
        if not request.config.getoption("--gui"):
            return
        from isaacsim.core.experimental.utils import stage as stage_utils

        from _views import apply_view

        path = apply_view(request.node.nodeid, stage_utils.get_current_stage())
        if path:
            print(f"view applied: {path.name}")

    return _apply
