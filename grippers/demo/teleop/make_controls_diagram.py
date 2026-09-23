#!/usr/bin/env python3
"""Label the control map used by physx_tick_teleop.py.

    python3 make_controls_diagram.py        # writes controls.png next to this file

The base is Zacksly's "Control Screen" template from PS5 Button Icons and
Controls (CC BY 3.0), vendored unmodified under third_party/ps5_icons/. It
already carries the pad, the leader lines and the button chips -- this script
only writes text at the outer end of each line. Nothing is cropped or redrawn,
so the artwork stays as its author made it.

The PNG is committed because Isaac's bundled Python has no matplotlib. Re-run
this after changing a binding and keep CONTROLS in physx_tick_teleop.py in step.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ART = os.path.join(HERE, "third_party", "ps5_icons",
                   "Controls Outline White 4k.png")
OUT = os.path.join(HERE, "controls.png")

BG = "#101014"
TEXT = "#f2f4f6"
SUB = "#aab1b8"
ACCENT = "#7fd4ff"
CREDIT = "#6c7278"

# Where each label goes: (x, y, side) as fractions of the template, so the
# layout survives a re-export at another resolution. Chip positions came from
# the artwork's own alpha blobs rather than being eyeballed.
#
# No control NAMES here on purpose. The template already draws the chip -- a
# d-pad with one arm filled, a circled cross -- so repeating "D-pad up" beside
# it is noise. Each line says only what that control does.
LABELS = [
    # left shoulders
    (0.208, 0.084, "L", "yaw -"),
    # left column: the four d-pad chips, top to bottom = up, right, down, left
    (0.183, 0.366, "L", "move up"),
    (0.183, 0.436, "L", "zoom in"),
    (0.183, 0.505, "L", "move down"),
    (0.183, 0.572, "L", "zoom out"),
    # the two bare leader lines on the left
    (0.232, 0.250, "L", "restart the sim"),
    (0.206, 0.662, "L", "roll / pitch"),
    # right shoulders
    (0.792, 0.084, "R", "yaw +"),
    (0.792, 0.146, "R", "open / close"),
    # right column: cross only -- square, circle and triangle are unbound
    (0.798, 0.373, "R", "point tool down"),
    # the two bare leader lines on the right
    (0.768, 0.250, "R", "show / hide this"),
    (0.793, 0.662, "R", "horizontal move"),
]

GAP = 0.018          # clear of the line end, as a fraction of image width


def main():
    if not os.path.isfile(ART):
        raise SystemExit("missing base artwork: %s" % ART)
    img = mpimg.imread(ART)
    h, w = img.shape[:2]

    fig, ax = plt.subplots(figsize=(15.0, 15.0 * h / w), dpi=130)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, 1)
    # Trims blank canvas below the pad -- the artwork itself is untouched.
    ax.set_ylim(0.87, -0.13)              # y down, with room for title/credit
    ax.axis("off")
    ax.imshow(img, extent=[0, 1, 1, 0], zorder=2, interpolation="bilinear")
    # AFTER imshow: it sets aspect="equal" itself, which would override this.
    # Both axes span 0..1 but map to different pixel counts, so without the
    # correction the artwork is stretched to fill the axes -- it rendered at
    # aspect 1.03 against the source's 1.90, squashing the pad nearly 2x.
    ax.set_aspect(h / float(w))

    ax.text(0.5, -0.105, "UR5e + 2F-85 teleop", ha="center", va="center",
            fontsize=23, color=ACCENT, weight="bold", zorder=4)
    ax.text(0.5, -0.055, "press Options in the sim to show or hide this",
            ha="center", va="center", fontsize=12.5, color=SUB, zorder=4)

    for x, y, side, what in LABELS:
        tx = x - GAP if side == "L" else x + GAP
        ha = "right" if side == "L" else "left"
        ax.text(tx, y, what, ha=ha, va="center", fontsize=16.5, color=TEXT,
                zorder=4)

    ax.text(0.5, 0.845,
            "Controller art: PS5 Button Icons and Controls by Zacksly "
            "(zacksly.itch.io), labels added. Licensed CC BY 3.0 - "
            "creativecommons.org/licenses/by/3.0/",
            ha="center", va="center", fontsize=9.5, color=CREDIT, zorder=4)

    fig.savefig(OUT, facecolor=BG, bbox_inches="tight", pad_inches=0.15)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
