"""Live heatmap window of a tactile sensor reader (readers.py): two 7 x 4 grids, left / right pad.

Used by the --gui runs of the runtime tests and by the demo's make_tactile_sensor_plot.py. The
window reads ``reader.read()`` every frame while the timeline plays; the reader's ``title``,
``caption``, ``unit`` and ``floor`` say what it shows. One window; ``show(reader)`` rebinds it.
"""

from __future__ import annotations

import numpy as np

from .readers import COLS, ROWS, SIDES


def _heat(v: float) -> int:
    """0..1 -> black, red, yellow, white (omni.ui ABGR)."""
    v = min(max(v, 0.0), 1.0)
    r, g, b = (min(max(v * 3.0 - k, 0.0), 1.0) for k in (0.0, 1.0, 2.0))
    return 0xFF000000 | int(b * 255) << 16 | int(g * 255) << 8 | int(r * 255)


class TactilePlot:
    """The heatmap window; ``show(reader)`` rebinds it to another reader."""

    # Cell size in px, in the taxel pitch's proportions (6 mm across, 5.5 mm along the
    # finger), so the 4 x 7 grid has the pad's 23.5 x 38 mm aspect.
    CELL_W, CELL_H, GAP = 40, 37, 3

    def __init__(self):
        import omni.kit.app
        import omni.ui as ui

        self.reader = None
        self._last_error = None
        self.scale = 0.5
        self.cells, self.labels = {}, {}
        self.window = ui.Window("TSF-85 tactile sensor", width=420, height=7 * (self.CELL_H + self.GAP) + 90)
        with self.window.frame:
            with ui.VStack(spacing=6):
                self.caption = ui.Label("", height=16, style={"color": 0xFF888888})
                with ui.HStack(spacing=20):
                    for side in SIDES:
                        with ui.VStack(spacing=self.GAP, width=COLS * (self.CELL_W + self.GAP)):
                            self.labels[side] = ui.Label(f"{side}: --", height=20, style={"color": 0xFF66CCFF})
                            for r in range(ROWS):
                                with ui.HStack(spacing=self.GAP, height=self.CELL_H):
                                    for c in range(COLS):
                                        with ui.ZStack(width=self.CELL_W, height=self.CELL_H):
                                            rect = ui.Rectangle(style=self._style(0.0))
                                            text = ui.Label("", alignment=ui.Alignment.CENTER,
                                                            style={"color": 0xFF808080, "font_size": 12})
                                        self.cells[side, r, c] = (rect, text)
        self._sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="runtime_test_tactile_plot")

    @staticmethod
    def _style(v: float) -> dict:
        return {"background_color": _heat(v), "border_width": 0.5, "border_color": 0xFF202020}

    def show(self, reader) -> None:
        self.reader = reader
        self.scale = reader.floor
        self.caption.text = f"{reader.title}: {reader.caption}"
        self._draw({s: np.zeros((ROWS, COLS)) for s in SIDES})

    def _on_update(self, _event) -> None:
        import omni.timeline

        if self.reader is None or not omni.timeline.get_timeline_interface().is_playing():
            return
        try:
            grids = self.reader.read()
        except (NotImplementedError, ValueError, AttributeError, TypeError, KeyError) as e:  # a wrong setup, not a timing issue
            self._sub = None
            self.caption.text = f"ERROR: {e!r}"
            print(f"[tactile plot] stopped: {e!r}")
            return
        except Exception as e:  # between stages, or before the physics view exists
            if repr(e) != self._last_error:
                self._last_error = repr(e)
                print(f"[tactile plot] waiting for the physics view: {e!r}")
            return
        self._draw(grids)

    def _draw(self, grids: dict[str, np.ndarray]) -> None:
        peak = max(float(g.max()) for g in grids.values())
        self.scale = max(peak, self.scale * 0.98, self.reader.floor)  # rises at once, decays slowly
        for side, g in grids.items():
            for r in range(ROWS):
                for c in range(COLS):
                    rect, text = self.cells[side, r, c]
                    v = float(g[r, c])
                    rect.style = self._style(v / self.scale)
                    text.text = f"{v:.2f}" if v > 0.01 * self.reader.floor else ""
                    text.style = {"color": 0xFF000000 if v > 0.5 * self.scale else 0xFFFFFFFF, "font_size": 12}
            self.labels[side].text = f"{side}: peak {g.max():.2f} {self.reader.unit} (scale {self.scale:.2f})"
