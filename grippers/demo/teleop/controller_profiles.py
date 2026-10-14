"""Gamepad profiles for physx_tick_teleop.py.

A profile says which button or stick of YOUR gamepad drives each robot action.
Gamepads are read with pygame, which numbers buttons and sticks the way each
controller reports them, so the numbers differ between models: one profile per
model. CONTROLLER_PROFILES.md walks through adding one; this header is the
reference.

Which profile is used: the first one whose "match" text appears in the
gamepad's name (case-insensitive), else DEFAULT_PROFILE. Set the environment
variable TELEOP_CONTROLLER=<profile> to force one.

Sources -- what an action reads (run controller_probe.py to find the numbers):
    ("axis", i, sign)     stick axis i, -1..1. pygame reports right = +1 and
                          down = +1; sign -1 reverses the axis.
    ("trigger", i)        analog trigger on axis i: released -1, pressed +1.
    ("button", i)         button i: pressed or not.
    ("hat", i, (x, y))    d-pad i held in direction (x, y): (0, 1) up,
                          (0, -1) down, (-1, 0) left, (1, 0) right.

Actions -- what the robot does:
    move_x       stick: tool right (+) / left (-), relative to the view
    move_y       stick: tool toward you (+) / away (-), relative to the view
    roll, pitch  sticks: rotate the tool about its own axes
    up, down     tool up / down (world vertical)
    yaw_left, yaw_right   rotate the tool about its axis
    grip         close the gripper while held; a trigger closes it in
                 proportion to how far it is pressed
    faster, slower        speed up / down one step (x1.25)
    tool_down    point the tool straight down
    zoom_in, zoom_out     move the view closer to / away from the tool
    restart      restart the simulation
    help         show the control list
Leave an action out to leave it unmapped. "match" is not an action: it lists
pieces of gamepad names this profile is for.
"""

PROFILES = {
    # PS5 DualSense / PS4 DualShock 4, through SDL's PlayStation driver.
    # Layout from SDL's PlayStation mapping, not yet checked with
    # controller_probe.py. On Linux a DualSense is read from /dev/input/js0
    # instead, without a profile.
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
    # Elecom wired gamepad (USB 05b8:1004). Button numbers measured on Linux
    # (button = label printed on the controller minus one); not yet checked
    # with controller_probe.py on Windows. Back 5 / back 7 (left) move up /
    # down, back 8 (right lower) holds the gripper closed.
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
