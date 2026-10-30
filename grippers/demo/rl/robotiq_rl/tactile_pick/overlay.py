"""A pressure-map panel for videos: the force on the 7 x 4 taxels of both TSF pads, drawn onto a frame.

Pure numpy / PIL / matplotlib (no Isaac): ``pressure_panel`` makes the RGBA panel, ``overlay_lower_left`` puts it on a frame."""

from __future__ import annotations

import numpy as np
from matplotlib import colormaps
from PIL import Image, ImageDraw, ImageFont

CELL = 30  # px per taxel
GAP = 2  # px between taxels
F_MAX = 10.0  # N at the top of the colour scale (log(1 + F) scale)
TICKS = (0.0, 1.0, 3.0, 10.0)  # N, labelled on the colour bar
_LUT = (colormaps["inferno"](np.linspace(0.0, 1.0, 256))[:, :3] * 255).astype(np.uint8)


def _scale(force: np.ndarray) -> np.ndarray:
    """Force [N] -> 0 .. 1 on a log(1 + F) scale."""
    return np.clip(np.log1p(np.maximum(force, 0.0)) / np.log1p(F_MAX), 0.0, 1.0)


def _pad_image(force: np.ndarray) -> Image.Image:
    """One pad: ``force`` is ``(rows, cols)`` [N], row 0 at the top (wrist side), as a grid of coloured cells."""
    rows, cols = force.shape
    colour = _LUT[(_scale(force) * 255).astype(int)]  # (rows, cols, 3)
    img = np.full((rows * CELL, cols * CELL, 3), 55, np.uint8)  # grey grid lines show the empty taxels
    for r in range(rows):
        for c in range(cols):
            img[r * CELL + GAP // 2 : (r + 1) * CELL - GAP // 2, c * CELL + GAP // 2 : (c + 1) * CELL - GAP // 2] = colour[r, c]
    return Image.fromarray(img)


def pressure_panel(taxel_force: np.ndarray, title: str = "TSF pressure map") -> Image.Image:
    """RGBA panel for both pads: ``taxel_force`` is ``(2, rows, cols)`` [N], left pad first."""
    font = ImageFont.load_default(size=15)
    small = ImageFont.load_default(size=12)
    pads = [_pad_image(f) for f in taxel_force]
    pw, ph = pads[0].size
    pad_gap, margin, head, foot = 26, 14, 30, 58
    width = 2 * pw + pad_gap + 2 * margin
    height = head + ph + foot + margin
    panel = Image.new("RGBA", (width, height), (10, 10, 14, 200))
    draw = ImageDraw.Draw(panel)
    draw.text((margin, 8), title, fill=(235, 235, 235, 255), font=font)
    for k, (name, pad) in enumerate(zip(("left pad", "right pad"), pads)):
        x = margin + k * (pw + pad_gap)
        panel.paste(pad.convert("RGBA"), (x, head))
        draw.text((x + pw // 2 - 24, head + ph + 4), name, fill=(220, 220, 220, 255), font=small)
    draw.text((margin, head + ph + 22), "base at the top, tip at the bottom", fill=(150, 150, 150, 255), font=small)
    # colour bar, log scale
    bar_w, bar_h, bar_y = width - 2 * margin, 10, head + ph + 40
    ramp = _LUT[np.linspace(0, 255, bar_w).astype(int)][None].repeat(bar_h, axis=0)
    panel.paste(Image.fromarray(ramp).convert("RGBA"), (margin, bar_y))
    for t in TICKS:
        x = margin + int(_scale(np.array(t)) * (bar_w - 1))
        draw.line([(x, bar_y + bar_h), (x, bar_y + bar_h + 3)], fill=(200, 200, 200, 255))
        draw.text((min(x - 3, width - margin - 26), bar_y + bar_h + 3), f"{t:g}", fill=(200, 200, 200, 255), font=small)
    draw.text((width - margin - 14, bar_y - 14), "N", fill=(200, 200, 200, 255), font=small)
    return panel


def overlay_lower_left(frame: np.ndarray, panel: Image.Image, margin: int = 24) -> np.ndarray:
    """Alpha-composite ``panel`` onto the lower left corner of ``frame`` (``(H, W, 3)`` uint8) and return a new frame."""
    out = frame.copy()
    p = np.asarray(panel)
    h, w = p.shape[:2]
    y0, x0 = frame.shape[0] - h - margin, margin
    alpha = p[..., 3:4].astype(np.float32) / 255.0
    region = out[y0 : y0 + h, x0 : x0 + w].astype(np.float32)
    out[y0 : y0 + h, x0 : x0 + w] = (region * (1 - alpha) + p[..., :3] * alpha).astype(np.uint8)
    return out
