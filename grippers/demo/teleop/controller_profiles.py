"""Gamepad profiles for physx_tick_teleop.py.

A profile says which button or stick of YOUR gamepad drives each robot action.
Gamepads are read with pygame, which numbers buttons and sticks the way the
controller reports them; those numbers differ between models, hence one
profile per model. See CONTROLLER_PROFILES.md to add one.

The profile is chosen automatically: the first one whose "match" text appears
in the gamepad's name. Set TELEOP_CONTROLLER=<profile> to force one.

Each action maps to one source:
    ("axis", i, sign)     stick axis i. pygame reports right = +1 and down = +1;
                          sign -1 reverses it.
    ("trigger", i)        analog trigger on axis i (released -1, pressed +1).
    ("button", i)         button i.
    ("hat", i, (x, y))    d-pad i in direction (x, y), e.g. (0, 1) = up.

Actions:
    move_x, move_y        tool left/right and away/toward you, relative to
                          the view (sticks: + = right, + = toward you)
    roll, pitch           tool rotation (sticks)
    up, down              tool up / down
    yaw_left, yaw_right   tool rotation about its axis
    grip                  close the gripper while held (trigger or button)
    faster, slower        speed up / down by one step
    tool_down             point the tool straight down
    zoom_in, zoom_out     move the view closer to / away from the tool
    restart               restart the simulation
    help                  show the control list
Leave an action out to leave it unmapped.
"""

PROFILES = {
    # PS5 DualSense / PS4 DualShock 4, through SDL's PlayStation driver.
    # Not yet checked with controller_probe.py on Windows.
    "dualsense": {
        "match": ["DualSense", "PS5 Controller", "PS4 Controller", "Wireless Controller"],
        "move_x": ("axis", 2, +1), "move_y": ("axis", 3, +1),
        "roll": ("axis", 0, +1), "pitch": ("axis", 1, +1),
        "up": ("button", 11), "down": ("button", 12),
        "zoom_in": ("button", 13), "zoom_out": ("button", 14),
        "yaw_left": ("button", 9), "yaw_right": ("button", 10),
        "grip": ("trigger", 5),
        "faster": ("button", 3), "slower": ("button", 2),
        "tool_down": ("button", 0),
        "restart": ("button", 4), "help": ("button", 6),
    },
    # Elecom wired gamepad (USB 05b8:1004). Button i is the label printed on
    # the controller minus one. Back 5 / back 7 (left) move up / down, back 8
    # (right lower) holds the gripper closed.
    "elecom": {
        "match": ["ELECOM", "JC-U"],
        "move_x": ("axis", 2, +1), "move_y": ("axis", 3, +1),
        "roll": ("axis", 0, +1), "pitch": ("axis", 1, +1),
        "up": ("button", 4), "down": ("button", 6),
        "yaw_left": ("button", 0), "yaw_right": ("button", 3),
        "grip": ("button", 7),
        "faster": ("button", 1), "slower": ("button", 2),
        "tool_down": ("button", 8), "help": ("button", 9),
    },
}

# Used when no profile matches the gamepad's name.
DEFAULT_PROFILE = "dualsense"


def pick(gamepad_name, forced=None):
    """Profile name for this gamepad: `forced` if given, else the first match."""
    if forced:
        if forced not in PROFILES:
            raise ValueError("unknown controller profile %r; choose from %s"
                             % (forced, sorted(PROFILES)))
        return forced
    low = gamepad_name.lower()
    for name, profile in PROFILES.items():
        if any(m.lower() in low for m in profile.get("match", ())):
            return name
    return DEFAULT_PROFILE
