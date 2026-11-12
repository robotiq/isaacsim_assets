"""pytest plugin of stamp.py's shards, loaded with ``-p stamp_shard``.

Each shard collects the whole scope itself and keeps its share, so no extra process boots Isaac Sim just to
collect; stamp.py checks that every shard wrote the same collection and that the reports cover it.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup("stamp_shard")
    group.addoption("--stamp-shard", metavar="K/N", help="run the tests whose index in the sorted ids is K modulo N")
    group.addoption("--stamp-collected", metavar="FILE", help="write every collected test id (sorted) to FILE")


def parse_shard(spec: str) -> tuple[int, int]:
    m = re.fullmatch(r"([0-9]+)/([0-9]+)", spec)
    if not m or int(m[1]) >= int(m[2]):
        raise pytest.UsageError(f"--stamp-shard expects K/N with 0 <= K < N, got {spec!r}")
    return int(m[1]), int(m[2])


def pytest_configure(config: pytest.Config) -> None:
    if config.getoption("--stamp-shard"):
        parse_shard(config.getoption("--stamp-shard"))


# trylast: after tests/runtime/conftest.py's --gripper filter (and -k), so the list is the scope's tests.
@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    ordered = sorted(item.nodeid for item in items)
    out = config.getoption("--stamp-collected")
    if out:
        Path(out).write_text("".join(f"{i}\n" for i in ordered))
    spec = config.getoption("--stamp-shard")
    if not spec:
        return
    k, n = parse_shard(spec)
    mine = set(ordered[k::n])
    config.hook.pytest_deselected(items=[i for i in items if i.nodeid not in mine])
    items[:] = [i for i in items if i.nodeid in mine]
