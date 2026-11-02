#!/usr/bin/env python3
"""Build (or check) each gripper's standard SimReady thumbnail on a black background.

SimReady Central's REVIEW.004 wants `.thumbs/256x256/<root>.png` to show the
asset on a black background. The render is already delivered as
`<root>_transparent.png` (straight alpha from the renderer), so the standard
thumbnail is that image composited over pure #000000 -- no re-render needed,
and the lighting, framing and antialiased edges stay exactly as rendered.

    python3 ci/simready/thumbnail_black_bg.py            # regenerate all grippers
    python3 ci/simready/thumbnail_black_bg.py --check    # fail if a background isn't black
    python3 ci/simready/thumbnail_black_bg.py --check grippers/Robotiq_2F_85
"""

import argparse
import sys
from pathlib import Path

from PIL import Image

REPO = Path(__file__).resolve().parents[2]
SIZE = 256
BLACK = (0, 0, 0)


def thumb_paths(gripper_dir: Path) -> tuple[Path, Path]:
    name = gripper_dir.name
    thumbs = gripper_dir / ".thumbs" / f"{SIZE}x{SIZE}"
    return thumbs / f"{name}.usda.png", thumbs / f"{name}.usda_transparent.png"


def composite(transparent: Path, out: Path) -> None:
    rgba = Image.open(transparent).convert("RGBA")
    background = Image.new("RGBA", rgba.size, BLACK + (255,))
    Image.alpha_composite(background, rgba).convert("RGB").save(out, optimize=True)


def check(standard: Path) -> list[str]:
    errors = []
    image = Image.open(standard)
    if image.width < SIZE or image.height < SIZE:
        errors.append(f"{image.width}x{image.height} is smaller than {SIZE}x{SIZE}")
    rgb = image.convert("RGB")
    w, h = rgb.size
    most_common = max(rgb.getcolors(w * h))[1]
    if most_common != BLACK:
        errors.append(f"background is #{'%02x%02x%02x' % most_common}, not #000000")
    corners = [rgb.getpixel(p) for p in ((0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1))]
    if any(c != BLACK for c in corners):
        errors.append(f"corner pixels are not black: {corners}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("grippers", nargs="*", type=Path,
                        help="gripper folders (default: every grippers/Robotiq_* folder)")
    parser.add_argument("--check", action="store_true",
                        help="only verify the standard thumbnails; write nothing")
    args = parser.parse_args()

    grippers = args.grippers or sorted(p for p in (REPO / "grippers").glob("Robotiq_*") if p.is_dir())
    failed = False
    for gripper in grippers:
        standard, transparent = thumb_paths(gripper)
        if not args.check:
            composite(transparent, standard)
        errors = check(standard)
        failed |= bool(errors)
        status = "FAIL: " + "; ".join(errors) if errors else "ok (black background)"
        print(f"{standard.resolve().relative_to(REPO)}: {status}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
