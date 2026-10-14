"""Print what each control of a gamepad reports, to write a profile.

Run it with Isaac Sim's Python (pygame installed there), no Isaac window needed:

    C:\\isaacsim\\python.bat controller_probe.py        (Windows)
    ~/isaacsim/python.sh controller_probe.py           (Linux)

Press each button, push each stick and the d-pad, one at a time, and note the
numbers it prints. Ctrl+C to stop. See CONTROLLER_PROFILES.md.
"""
import os
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame


def main():
    pygame.display.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if not count:
        print("No gamepad found. Plug it in (on Windows, make sure it is not attached to WSL).")
        return
    pads = [pygame.joystick.Joystick(i) for i in range(count)]
    for i, pad in enumerate(pads):
        print("gamepad %d: %r  (%d axes, %d buttons, %d d-pads)"
              % (i, pad.get_name(), pad.get_numaxes(), pad.get_numbuttons(), pad.get_numhats()))
    print("Press buttons, push sticks and the d-pad. Ctrl+C to stop.\n")
    last_axis = {}
    try:
        while True:
            for e in pygame.event.get():
                if e.type == pygame.JOYBUTTONDOWN:
                    print('button %d pressed       -> ("button", %d)' % (e.button, e.button))
                elif e.type == pygame.JOYHATMOTION and e.value != (0, 0):
                    print("d-pad %d direction %s -> (\"hat\", %d, %s)"
                          % (e.hat, e.value, e.hat, e.value))
                elif e.type == pygame.JOYAXISMOTION:
                    # Report a stick once per push, when it passes half way.
                    side = 0 if abs(e.value) < 0.5 else (1 if e.value > 0 else -1)
                    if side and last_axis.get(e.axis) != side:
                        print('axis %d at %+.1f          -> ("axis", %d, sign) or ("trigger", %d)'
                              % (e.axis, e.value, e.axis, e.axis))
                    last_axis[e.axis] = side
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
