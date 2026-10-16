# Gamepad profiles: use your own gamepad

The teleop works with any gamepad that Windows or Linux recognises. Gamepads
don't all number their buttons and sticks the same way, so the teleop reads
yours through a **profile**: a short table that says which button or stick
drives each robot action. Profiles live in
[`controller_profiles.py`](controller_profiles.py).

This page shows how to check whether your gamepad already has a profile, and
how to write one if it doesn't. It takes about 15 minutes and needs no
programming beyond editing a table.

## Before you start

- **Install `pygame` into Isaac Sim's Python, once.** The teleop and the
  probe tool use it to read gamepads:
  ```bash
  C:\isaacsim\python.bat -m pip install pygame        # Windows
  ~/isaacsim/python.sh -m pip install pygame          # Linux
  ```
- **Plug the gamepad in before starting Isaac Sim.** Only the first gamepad
  is used.
- **DualSense on Linux:** no profile needed. It's read directly from
  `/dev/input/js0`, with the mapping in the [README](README.md#gamepad).
- **Windows and WSL:** if you passed the gamepad to WSL with `usbipd`, Windows
  no longer sees it. Give it back in PowerShell as Administrator with
  `usbipd unbind --busid <BUSID>`, then unplug and replug it.

## 1. Check which profile your gamepad gets

Start the demo and look at the console. One of these lines tells you what
happened:

| Console line | Meaning |
|---|---|
| `input: gamepad '<name>' with profile 'elecom', plus keyboard` | Your gamepad was found and matched the `elecom` profile. |
| `input: gamepad '<name>' with profile 'dualsense', ...` and the buttons do the wrong things | No profile matched your gamepad's name, so the default (`dualsense`) was used. Write a profile (step 2). |
| `no gamepad (no gamepad found)` | Isaac Sim doesn't see a gamepad: check the cable, WSL (above), or try the probe (step 2). |
| `no gamepad (No module named 'pygame')` | `pygame` isn't installed in Isaac Sim's Python (see Before you start). |

Profiles shipped today:

| Profile | Gamepad | Status |
|---|---|---|
| `dualsense` | PS5 DualSense, PS4 DualShock 4 | Layout from SDL's PlayStation mapping; not yet checked with the probe. |
| `elecom` | Elecom wired gamepad (USB `05b8:1004`) | Button numbers measured on Linux; not yet checked on Windows. |

If you check one of them with the probe, please update its status line in
`controller_profiles.py`.

## 2. See what your gamepad reports

Run the probe with Isaac Sim's Python, from the `teleop` folder. No Isaac
window opens:

```bash
C:\isaacsim\python.bat controller_probe.py        # Windows
~/isaacsim/python.sh controller_probe.py          # Linux
```

It prints your gamepad's name, then one line each time you press a button,
push a stick past half-way, or press the d-pad:

```
gamepad 0: '<your gamepad's name>'  (4 axes, 12 buttons, 1 d-pads)
Press buttons, push sticks and the d-pad. Ctrl+C to stop.

button 7 pressed       -> ("button", 7)
axis 2 at +1.0          -> ("axis", 2, sign) or ("trigger", 2)
d-pad 0 direction (0, 1) -> ("hat", 0, (0, 1))
```

The text after `->` is what you'll put in the profile. Go through every
control, one at a time, and write down what each one prints:

1. **Each stick, in each direction.** Push the right stick right, then up;
   the left stick right, then up. A stick is two axes: note which axis moves
   for right/left and which for up/down, and whether right and up print `+`
   or `-`.
2. **Each trigger.** An analog trigger prints an axis, like a stick. A
   digital trigger prints a button.
3. **Every button**, including shoulder and back buttons.
4. **The d-pad.** On most gamepads it prints `d-pad 0 direction (...)`; on
   some it prints buttons instead.

Note also a piece of the gamepad's name (from the first line): the profile
uses it to recognise your gamepad.

## 3. Write the profile

Open [`controller_profiles.py`](controller_profiles.py), copy one entry of
`PROFILES` (the `elecom` one is a good start), give it a name, and fill it in.

### What each action does

| Action | Robot motion | Typically mapped to |
|---|---|---|
| `move_x` | tool right / left, relative to the view | right stick, left-right axis |
| `move_y` | tool toward / away from you, relative to the view | right stick, up-down axis |
| `roll`, `pitch` | rotate the tool about its own axes | left stick |
| `up`, `down` | tool up / down (always world vertical) | d-pad up/down, or shoulder buttons |
| `yaw_left`, `yaw_right` | rotate the tool about its axis | shoulder buttons |
| `grip` | close the gripper while held | right trigger |
| `faster`, `slower` | speed up / down one step | face buttons |
| `tool_down` | point the tool straight down | a face button |
| `zoom_in`, `zoom_out` | view closer to / away from the tool | d-pad left/right |
| `restart` | restart the simulation | a "select" / "share" button |
| `help` | show the control list | a "start" / "options" button |

Leave out any action you don't need; it simply stays unmapped. The keyboard
works at the same time, so nothing is lost.

### How to write a source

| The probe printed | Write |
|---|---|
| `axis 2 at +1.0` when pushing **right** | `("axis", 2, +1)` for `move_x` or `roll` |
| `axis 2 at -1.0` when pushing **right** | `("axis", 2, -1)`: the sign makes right positive |
| `axis 3 at -1.0` when pushing **up** | `("axis", 3, +1)` for `move_y` or `pitch`: up must be negative, which it already is |
| `axis 3 at +1.0` when pushing **up** | `("axis", 3, -1)` |
| an axis for a **trigger** | `("trigger", 5)` for `grip`: closes in proportion to how far it is pressed |
| `button 7 pressed` | `("button", 7)` |
| `d-pad 0 direction (0, 1)` | `("hat", 0, (0, 1))`: up. Down is `(0, -1)`, left `(-1, 0)`, right `(1, 0)` |

The rule for sticks: after the sign, **right must be +1 and up must be −1**.
That's how pygame reports most sticks already, so `+1` is the usual sign.

### Example

Say the probe showed: right stick on axes 2 (right +) and 3 (up −), left
stick on axes 0 and 1 (same directions), the d-pad as `d-pad 0`, an analog
right trigger on axis 5, shoulder buttons 4 and 5, face buttons 0 to 3, and
start on button 7. The gamepad's name contains "Acme Pad". The profile:

```python
    "acme": {
        "match": ["Acme Pad"],
        "move_x": ("axis", 2, +1), "move_y": ("axis", 3, +1),
        "roll": ("axis", 0, +1), "pitch": ("axis", 1, +1),
        "up": ("hat", 0, (0, 1)), "down": ("hat", 0, (0, -1)),
        "zoom_in": ("hat", 0, (-1, 0)), "zoom_out": ("hat", 0, (1, 0)),
        "yaw_left": ("button", 4), "yaw_right": ("button", 5),
        "grip": ("trigger", 5),
        "faster": ("button", 3), "slower": ("button", 2),
        "tool_down": ("button", 0),
        "help": ("button", 7),
    },
```

Put it inside `PROFILES = { ... }`, next to the other entries, with a comma
after the closing `}`.

## 4. Test it

1. Restart Isaac Sim with the demo. The console should name your profile:
   `input: gamepad '<name>' with profile 'acme'`. To force it regardless of
   the name, set `TELEOP_CONTROLLER=acme` before starting
   (`$env:TELEOP_CONTROLLER = "acme"` in PowerShell).
2. Check each action against the table above, one at a time.
3. Fix what's wrong:

| Problem | Fix |
|---|---|
| A stick moves the tool the wrong way | Flip that axis's sign (`+1` ↔ `-1`). |
| The tool moves on its own with the sticks released | A trigger is mapped as a stick: use `("trigger", i)`, or check the axis numbers. |
| The gripper doesn't follow the trigger, or is partly closed at rest | Your driver reports the trigger at rest as 0 instead of −1: map `grip` to a button instead. |
| A button does nothing | Check its number with the probe; some gamepads number from 1 in their labels but from 0 in pygame. |
| The console names another profile | Your `"match"` text isn't in the gamepad's name, or an earlier profile matches first. Use a more specific piece of the name, or `TELEOP_CONTROLLER`. |

## 5. Share it

If your profile works, add it to the repository so others with the same
gamepad get it automatically: open a pull request that adds your entry to
`PROFILES`, with a comment line saying which gamepad it is (name and, if you
know it, USB vendor:product ID) and on which system you tested it.
