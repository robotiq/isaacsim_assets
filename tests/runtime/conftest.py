"""Session-wide Isaac Sim app for the runtime tests.

The app is created in ``pytest_configure`` (before collection) because every
``isaacsim.*`` import in the test modules needs a running Kit. It is closed in
``pytest_unconfigure``.

Options (see README.md):
  --gui    open the Isaac Sim window instead of running headless
  --hold   with --gui, keep the window open after the last test until closed
"""

from __future__ import annotations

import pytest

_app = None


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption("--gui", action="store_true", default=False, help="run with the Isaac Sim window (default: headless)")
    parser.addoption("--hold", action="store_true", default=False, help="with --gui, keep the window open after the tests")


def pytest_configure(config: pytest.Config) -> None:
    global _app
    from isaacsim import SimulationApp

    _app = SimulationApp({"headless": not config.getoption("--gui")})
    config._isaac_app = _app


def pytest_unconfigure(config: pytest.Config) -> None:
    global _app
    if _app is None:
        return
    if config.getoption("--gui") and config.getoption("--hold"):
        print("\n--hold: close the Isaac Sim window to finish.")
        while _app.is_running():
            _app.update()
    _app.close()
    _app = None


@pytest.fixture(scope="session")
def simulation_app(request: pytest.FixtureRequest):
    return request.config._isaac_app
