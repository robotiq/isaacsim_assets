"""Empty the box: the trained tactile policy picks a sphere, a scripted routine drops it outside, repeat.

    uv run python <this file> --policy <run>/exported/policy.pt --num_envs 1 --viz kit      # watch it
    uv run python <this file> --policy <run>/exported/policy.pt --num_envs 16               # statistics
    uv run python <this file> --policy <run>/exported/policy.pt --num_envs 1 --video dir    # record it

Per env a small state machine:  PICK (the policy acts) -> CARRY -> RELEASE -> BACK (rise) -> RETURN (above the box centre) -> PICK ...
The hand-over from PICK to CARRY uses only what the robot can sense: both pads press on something and the tool tip is
above the box rim for ``--hold_steps`` steps in a row. The scripted CARRY moves the tool tip above the table beside the
box, keeping the gripper closed; RELEASE opens it. Spheres the gripper cannot reach (near the walls) stay in the box:
an env stops after ``--give_up`` steps without a pick."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.envs.utils.video_recorder_cfg import VideoRecorderCfg
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--policy", required=True)
parser.add_argument("--task", default="Robotiq-Tactile-Sphere-Pick-UR5e-2F85")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=6000, help="policy steps (30 per second)")
parser.add_argument("--hold_steps", type=int, default=10)
parser.add_argument("--give_up", type=int, default=900, help="steps without a pick before an env stops")
parser.add_argument("--layout", choices=("reachable", "floor"), default="reachable", help="reachable: all spheres where the tool tip can go; floor: spread over the whole floor (as in training, about half out of reach)")
parser.add_argument("--slow", type=float, default=1.0, help="scale the policy's tool-tip motion by this near the floor (gentler contact)")
parser.add_argument("--slow_below", type=float, default=0.10, help="tool-tip height below which --slow applies")
parser.add_argument("--seed", type=int, default=None, help="seed the layouts and the noise, so two runs start from the same boxes")
parser.add_argument("--gate", type=float, default=None, help="tactile max_contact_distance [m] (negative: off); default: the task's")
parser.add_argument("--measure_gate", action="store_true", help="count how often the contact-distance gate / the old taxel geometry change the tactile map")
parser.add_argument("--env_completes_picks", action="store_true", help="leave the task's own pick completion on (it teleports a sphere held out of the box and homes the arm); off by default, the supervisor does the carry itself")
parser.add_argument("--regrasp", action="store_true", help="shape task: put the carried piece down, release it and grasp it again, centred, at the same place; the tactile maps of that second grasp are what --recognizer / --record_grasps use")
parser.add_argument("--putback_speed", type=float, default=2.0, help="speed [rad/s] of every joint when an unknown piece is taken to the put-back poses")
parser.add_argument("--no_centre", action="store_true", help="--regrasp: do not shift the second grasp by the offset of the piece on the pads, estimated from the tactile maps of the first")
parser.add_argument("--recognizer", default=None, help="shape task: a recogniser (joblib, scripts/fit_recognizer.py) that names every piece the policy lifts; the accuracy is printed and the name shown in --hq_video")
parser.add_argument("--record_grasps", default=None, help="shape task: save the tactile history and the true class of every piece the policy lifts (for the recogniser) to this .npz")
parser.add_argument("--video", default=None, help="record env 0 into this directory")
parser.add_argument("--hq_video", default=None, help="record env 0 to this .mp4 with an RTX camera, 1920 x 1080")
parser.add_argument("--hq_pressure_map", action="store_true", help="draw the TSF pressure map of both fingertips on the lower left of --hq_video")
parser.add_argument("--hq_eye", type=float, nargs=3, default=(1.15, -0.75, 0.75), help="camera position of --hq_video (env frame, m)")
parser.add_argument("--hq_look", type=float, nargs=3, default=(0.5, 0.0, 0.1), help="the point the camera of --hq_video looks at")
parser.add_argument("--hq_fps", type=int, choices=(120, 60, 30), default=60, help="frame rate of --hq_video: a frame every 120 / hq_fps physics steps (120, 60 or 30)")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab.sensors.camera import CameraCfg  # noqa: E402
from isaaclab.sim import PinholeCameraCfg  # noqa: E402
from isaaclab.utils.math import quat_from_matrix  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.overlay import overlay_lower_left, pressure_panel  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import BOX_CENTER, BOX_INNER, REACH_HALF, RIM_Z, TASK  # noqa: E402

PICK, CARRY, RELEASE, BACK, RETURN, PLACE, ADJUST, REGRASP, PUTBACK = 0, 1, 2, 3, 4, 5, 6, 7, 8
# Putting an unknown piece back in the box: the arm goes to a fixed joint pose, then to a second one, and lets go there. Joint angles
# [rad] of shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 for the tool tip at (x, y, z) in the env frame, pointing down
# (from tactile_pick/start_poses.npz; the box centre is (0.5, 0)).
PUT_BACK_ABOVE = (-0.2534, -1.6529, 1.9059, -2.0038, -1.6055, -0.8434)  # tool tip at (0.5, 0.0, 0.28): above the box centre, the piece clears the rim
PUT_BACK_RELEASE = (-0.2534, -1.5247, 2.1176, -2.3437, -1.6055, -0.8434)  # tool tip at (0.5, 0.0, 0.16): the piece about 10 cm above the floor
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
Z_PLACE = 0.055  # m: tool-tip height at which a carried piece is put down on the table (--regrasp)
Z_GRASP = 0.026  # m: tool-tip height of the second grasp: the pads at the middle of a piece lying on the table
HOME_Z = 0.28  # m: tool-tip height above the box centre where the policy takes over again (inside its training starts)
CARRY_Z = 0.22  # m: tool tip height while carrying (sphere bottom ~ 18 cm: clear of the 11 cm walls)
GAIN = 60.0  # action per metre of error (an action of 1 is 1 cm per step)


def drop_point(k: torch.Tensor, dx: float = 0.07, dy: float = 0.07) -> torch.Tensor:
    """Where the k-th sphere of an env is dropped: beside the box on the +y side, on a grid so they do not pile up."""
    x = BOX_CENTER[0] - 0.14 + dx * (k % 5).float()
    y = BOX_CENTER[1] + BOX_INNER / 2 + 0.10 + dy * (k // 5).float()
    return torch.stack([x, y], dim=-1)


def shape_mdp_maps(env):
    """The shape task's tactile map [N] (what the policy and the recogniser see), ``(E, 56)``."""
    from robotiq_rl.shape_pick import mdp as shape_mdp

    return shape_mdp.tactile_map_shape(env, log_scale=False)


def main():
    caption = {"text": "", "phase": None}  # what the video panel shows: the recogniser's last answer while the piece is carried
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    env_cfg.episode_length_s = 3600.0  # no time-out reset: this is one long run
    if args_cli.seed is not None:
        torch.manual_seed(args_cli.seed)
        env_cfg.seed = args_cli.seed
    if args_cli.gate is not None:
        env_cfg.scene.tactile.max_contact_distance = None if args_cli.gate < 0 else args_cli.gate
    reset = env_cfg.events.reset_spheres.params
    if not args_cli.task.startswith("Robotiq-Tactile-Shape"):
        reset["num_active_range"] = None  # all ten spheres are in the box (the shape task keeps its random 4 - 8 pieces)
    reset["p_zone_only"] = 1.0 if args_cli.layout == "reachable" else 0.0
    total = args_cli.steps
    if args_cli.video:
        env_cfg.video_recorders = [VideoRecorderCfg(source="viz", output_dir=args_cli.video, video_length=total)]
    if args_cli.hq_video:
        env_cfg.sim.render_interval = 120 // args_cli.hq_fps  # a render (and a frame) every this many physics steps
        eye, target = torch.tensor(args_cli.hq_eye), torch.tensor(args_cli.hq_look)  # default: the viewer's camera
        fwd = (target - eye) / (target - eye).norm()
        right = torch.linalg.cross(fwd, torch.tensor([0.0, 0.0, 1.0]))
        right = right / right.norm()
        up = torch.linalg.cross(right, fwd)
        quat = quat_from_matrix(torch.stack([right, up, -fwd], dim=-1)).tolist()  # OpenGL camera: looks along -Z, up +Y
        env_cfg.scene.hq_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/HqCam",
            update_period=0.0,
            height=1080,
            width=1920,
            data_types=["rgb"],
            spawn=PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955, clipping_range=(0.05, 20.0)),
            offset=CameraCfg.OffsetCfg(pos=tuple(eye.tolist()), rot=tuple(quat), convention="opengl"),
        )
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        if args_cli.hq_video:
            import imageio

            hq_cam, hq_writer, hq_render = env.scene["hq_cam"], imageio.get_writer(args_cli.hq_video, fps=args_cli.hq_fps, codec="libx264", quality=9, macro_block_size=1), env.sim.render

            def render_and_capture(*a, **k):
                out = hq_render(*a, **k)
                hq_cam.update(1 / 120, force_recompute=True)
                frame = hq_cam.data.output["rgb"][0, ..., :3].cpu().numpy()
                if args_cli.hq_pressure_map:
                    if args_cli.task.startswith("Robotiq-Tactile-Shape"):  # the map the policy sees (the pieces' footprints)
                        taxels = shape_mdp_maps(env)[0].reshape(2, 7, 4).cpu().numpy()
                    else:
                        taxels = env.scene["tactile"].taxel_force[0].cpu().numpy()  # (2, 7, 4) N, after the sensor's contact-distance gate
                    show = "feeling the piece ..." if caption["phase"] in (ADJUST, REGRASP) and args_cli.regrasp else (caption["text"] if caption["phase"] in (CARRY, RELEASE, BACK, PLACE) else "")
                    frame = overlay_lower_left(frame, pressure_panel(taxels, caption=show))
                hq_writer.append_data(frame)
                return out

            env.sim.render = render_and_capture
        policy = torch.jit.load(args_cli.policy, map_location=env.device)
        obs, _ = env.reset()
        if not args_cli.env_completes_picks:
            # the env's state holds its own copy of TASK: without home_ranges it no longer completes a pick (teleports the
            # sphere, homes the arm) while the supervisor is still carrying it
            tp.get_state(env, TASK).cfg["home_ranges"] = None
        n, dev = env.num_envs, env.device
        phase = torch.full((n,), PICK, device=dev)
        held_count = torch.zeros(n, device=dev)
        timer = torch.zeros(n, device=dev)
        did_regrasp = torch.zeros(n, dtype=torch.bool, device=dev)
        pb_stage = torch.zeros(n, dtype=torch.long, device=dev)  # 0: going to PUT_BACK_ABOVE, 1: going to PUT_BACK_RELEASE
        pb_goal = torch.zeros(n, 6, device=dev)
        pb_active = torch.zeros(n, dtype=torch.bool, device=dev)
        robot = env.scene["robot"]
        arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        # the arm actions are Cartesian: for the envs that put a piece back the joint targets are overwritten right before every physics
        # step, moving each joint towards the goal at --putback_speed (the IK term, with a zero command, holds the pose the rest of the time)
        write_data_to_sim = env.scene.write_data_to_sim

        def write_with_joint_goal():
            if pb_active.any():
                ids = pb_active.nonzero().flatten()
                q = robot.data.joint_pos.torch[ids][:, arm_ids]
                step = args_cli.putback_speed * env.physics_dt
                robot.set_joint_position_target_index(target=q + (pb_goal[ids] - q).clamp(-step, step), joint_ids=arm_ids, env_ids=ids)
            write_data_to_sim()

        env.scene.write_data_to_sim = write_with_joint_goal
        unk = torch.zeros(n, dtype=torch.bool, device=dev)  # the piece in the gripper was "unknown": it is put back in the box, not carried out
        place_xy = torch.zeros(n, 2, device=dev)
        regrasp_off = torch.zeros(n, 2, device=dev)  # horizontal offset of the piece from the pads' centre, from the tactile footprint of the first grasp
        picked = torch.zeros(n, dtype=torch.long, device=dev)
        last_pick = torch.zeros(n, device=dev)
        active = torch.ones(n, dtype=torch.bool, device=dev)
        act = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
        centre = torch.tensor(BOX_CENTER, device=dev)
        meas = {"steps": 0, "contact": 0, "gate_changes": 0, "removed": 0.0, "total": 0.0, "max_gate": 0.0, "max_geom": 0.0}
        if args_cli.measure_gate:
            import numpy as np
            from robotiq_tactile_sensors.sensing import taxel_force_map

            sensor = env.scene["tactile"]
            # the taxel grid as it was when the policy was trained: rounded pitches (5.5 x 6 mm) and origin
            old_local = torch.tensor(
                np.stack(
                    [
                        np.stack([[-0.009 + 0.006 * c, y, 0.1489 - 0.0055 * r] for r in range(7) for c in range(4)])
                        for y in (-0.0416, 0.0416)
                    ]
                ),
                dtype=torch.float32,
                device=dev,
            )

        reach = torch.tensor(REACH_HALF, device=dev)
        HIST = 8  # tactile history kept for the recogniser: the last HIST policy steps
        recognize = None
        if args_cli.recognizer:
            import joblib
            from robotiq_rl.shape_pick import recognizer as recog_lib

            recognize = joblib.load(args_cli.recognizer)
            confusion = torch.zeros(6, 7, dtype=torch.long)  # rows: the piece, columns: the six answers, then "unknown"
            events = []  # (env, slot, step, true class, answer or -1): every time a piece is held and read
            checks, landed = [], []  # where a put-back piece lies 150 steps after it is let go
            carried = []  # distance of the piece to the tool point when the release pose is reached: still in the gripper?
            held_slot = torch.zeros(n, dtype=torch.long, device=dev)  # the slot of the piece in the gripper, at the last reading
        if args_cli.record_grasps or recognize is not None or args_cli.regrasp:
            from robotiq_rl.shape_pick import mdp as shape_mdp
            from robotiq_rl.shape_pick.env_cfg import SLOT_TYPES

            slot_type = torch.tensor(SLOT_TYPES, device=dev)
            hist_maps = torch.zeros(n, HIST, 56, device=dev)
            hist_closure = torch.zeros(n, HIST, device=dev)
            rec = {"maps": [], "closure": [], "label": [], "pad_force": [], "n_picked": []}
            dbg = []

        def spheres_in_box(zone_only=False):
            s = tp.get_state(env, TASK)
            rel = (s.sphere_pos[..., :2] - centre).abs()
            inside = (rel < (reach if zone_only else BOX_INNER / 2)).all(-1) & (s.sphere_pos[..., 2] < RIM_Z + 0.02)
            return inside.sum(-1)

        print(f"start: {spheres_in_box().tolist()} spheres in the box")
        for t in range(total):
            s = tp.get_state(env, TASK)
            f_sph, _ = tp.contact_forces(env, "pad_spheres")
            pad = f_sph.sum(-1)  # (n, 2)
            sensed_hold = (pad > 1.0).all(-1) & (s.tcp[:, 2] > RIM_Z + 0.06)
            with torch.inference_mode():
                act = policy(obs["policy"]).clone()
            tcp = s.tcp
            near_floor = (tcp[:, 2] < args_cli.slow_below)[:, None]
            act = torch.cat([torch.where(near_floor, act[:, 0:3] * args_cli.slow, act[:, 0:3]), act[:, 3:]], dim=-1)  # gentler touch near the floor
            # --- the hand-over logic (sensor-based) and the scripted phases
            held_count = torch.where((phase == PICK) & sensed_hold, held_count + 1, torch.zeros_like(held_count))
            start_carry = (phase == PICK) & (held_count >= args_cli.hold_steps)
            unk_now = torch.zeros(n, dtype=torch.bool, device=dev)
            regrasp_done = (phase == REGRASP) & (timer >= 45)
            trigger = regrasp_done if args_cli.regrasp else start_carry
            if (args_cli.record_grasps or recognize is not None) and bool(trigger.any()):
                ids = trigger.nonzero().flatten()
                nearest = torch.linalg.norm(s.sphere_pos[ids] - s.tcp[ids, None], dim=-1).argmin(-1)  # the piece in the gripper
                if recognize is not None:
                    guess, proba = recog_lib.decide(recognize, hist_maps[ids].cpu().numpy(), hist_closure[ids].cpu().numpy())
                    truth = slot_type[nearest].cpu().numpy()
                    for k, (g, tr) in enumerate(zip(guess, truth)):
                        confusion[int(tr), 6 if g < 0 else int(g)] += 1
                        events.append((int(ids[k]), int(nearest[k]), t, int(tr), int(g)))
                    if not args_cli.regrasp:  # "unknown": not carried out, put back in the box (released where it is, above the box)
                        unknown_ids = ids[torch.tensor(guess < 0, device=dev)]
                        held_slot[ids] = nearest
                        unk_now[unknown_ids] = True
                        unk |= unk_now
                    if bool(trigger[0]):
                        e0 = int((ids == 0).nonzero()[0])
                        best = recog_lib.CLASSES[int(proba[e0].argmax())]
                        if guess[e0] < 0:
                            caption["text"] = f"unknown ({best} {proba[e0].max():.0%}): put back"
                            print(f"t={t:5d} env0: unknown (best guess {best} {proba[e0].max():.0%}; it is a {recog_lib.CLASSES[int(truth[e0])]}): put back in the box")
                        else:
                            caption["text"] = f"recognised: {best}"
                            print(f"t={t:5d} env0: recognised {best} {proba[e0].max():.0%} (it is a {recog_lib.CLASSES[int(truth[e0])]})")
                if True:
                    off_now = (s.sphere_pos[ids, nearest] - s.tcp[ids]).cpu()
                    loaded = (f_sph[ids].sum((-1, -2)) > 1.0).cpu()
                    dbg.append(torch.cat([off_now, loaded[:, None].float(), regrasp_off[ids].cpu()], 1))
                rec["maps"].append(hist_maps[ids].cpu())
                rec["closure"].append(hist_closure[ids].cpu())
                rec["label"].append(slot_type[nearest].cpu())
                rec["pad_force"].append(f_sph[ids].sum(-1).cpu())
                rec["n_picked"].append(picked[ids].cpu())
            if args_cli.regrasp and not args_cli.no_centre and bool(start_carry.any()):
                ids = start_carry.nonzero().flatten()
                m = hist_maps[ids, -4:].mean(1).reshape(-1, 2, 7, 4)  # (k, pad, row, col) [N]
                col = (m.sum(2) * torch.arange(4, device=dev)).sum(-1) / (m.sum((2, 3)) + 1e-6)  # force-weighted column of each pad
                tw = env.scene["tactile"].taxel_positions_w[ids].reshape(-1, 2, 7, 4, 3)
                axis = (tw[:, :, :, 3] - tw[:, :, :, 0]).mean(2)  # (k, pad, 3): the direction of increasing column, in the world
                axis = axis / (axis.norm(dim=-1, keepdim=True) + 1e-9)
                pitch = (tw[:, :, :, 3] - tw[:, :, :, 0]).mean(2).norm(dim=-1) / 3  # m per column
                off = ((col - 1.5) * pitch)[..., None] * axis  # (k, pad, 3)
                w = m.sum((2, 3))
                regrasp_off[ids] = ((off[..., :2] * w[..., None]).sum(1) / (w.sum(1, keepdim=True) + 1e-6)).clamp(-0.03, 0.03)
            phase = torch.where(start_carry, torch.full_like(phase, CARRY), phase)
            phase = torch.where(unk_now, torch.full_like(phase, PUTBACK), phase)  # an unknown piece is not carried out: back into the box
            pb_stage = torch.where(unk_now, torch.zeros_like(pb_stage), pb_stage)
            timer = torch.where(unk_now, torch.zeros_like(timer), timer)
            did_regrasp = did_regrasp & ~start_carry
            target = drop_point(picked, *((0.11, 0.14) if args_cli.regrasp else (0.07, 0.07)))
            to_target = torch.cat([target - tcp[:, :2], torch.zeros(n, 1, device=dev)], dim=-1)
            carry = phase == CARRY
            reached = (to_target[:, :2].norm(dim=-1) < 0.02) & ((tcp[:, 2] - CARRY_Z).abs() < 0.03)
            # carry: rise first, then go sideways at carry height, gripper closed
            low = tcp[:, 2] < CARRY_Z - 0.03
            vxy = torch.where(low[:, None], torch.zeros(n, 2, device=dev), (GAIN * to_target[:, :2]).clamp(-1, 1))
            vz = (GAIN * (CARRY_Z - tcp[:, 2])).clamp(-1, 1)
            scripted = torch.zeros_like(act)
            scripted[:, 0:2] = vxy
            scripted[:, 2] = vz
            scripted[:, 6] = 1.0
            release = phase == RELEASE
            back = phase == BACK
            rel_act = torch.zeros_like(act)
            rel_act[:, 6] = -1.0
            back_act = torch.zeros_like(act)
            back_act[:, 2] = 0.5
            back_act[:, 6] = -1.0
            putback = phase == PUTBACK
            pb_act = torch.zeros_like(act)
            pb_act[:, 6] = 1.0  # the piece stays held; the joint targets are set by write_with_joint_goal
            place, adjust, regrasp = phase == PLACE, phase == ADJUST, phase == REGRASP
            hold_xy = (GAIN * (place_xy + regrasp_off - tcp[:, :2])).clamp(-1, 1)
            place_act = torch.zeros_like(act)
            place_act[:, 0:2] = (GAIN * (target - tcp[:, :2])).clamp(-1, 1)
            place_act[:, 2] = (GAIN * (Z_PLACE - tcp[:, 2])).clamp(-1, 1) * 0.5
            place_act[:, 6] = 1.0  # still holding
            adjust_act = torch.zeros_like(act)
            adjust_act[:, 0:2] = hold_xy
            adjust_act[:, 2] = (GAIN * (Z_GRASP - tcp[:, 2])).clamp(-1, 1) * 0.5
            adjust_act[:, 6] = -1.0  # open
            regrasp_act = adjust_act.clone()
            regrasp_act[:, 6] = 1.0  # close
            ret = phase == RETURN
            ret_act = torch.zeros_like(act)
            ret_act[:, 0:2] = (GAIN * (centre - tcp[:, :2])).clamp(-1, 1)
            ret_act[:, 2] = (GAIN * (HOME_Z - tcp[:, 2])).clamp(-1, 1)
            ret_act[:, 6] = -1.0
            use = lambda m: m[:, None]  # noqa: E731
            act = torch.where(use(carry), scripted, act)
            act = torch.where(use(release), rel_act, act)
            act = torch.where(use(back), back_act, act)
            act = torch.where(use(putback), pb_act, act)
            act = torch.where(use(place), place_act, act)
            act = torch.where(use(adjust), adjust_act, act)
            act = torch.where(use(regrasp), regrasp_act, act)
            act = torch.where(use(ret), ret_act, act)
            act = torch.where(use(~active), torch.zeros_like(act).index_fill_(1, torch.tensor([6], device=dev), -1.0), act)
            # --- phase transitions
            timer = torch.where(carry | release | back | ret | place | adjust | regrasp | putback, timer + 1, torch.zeros_like(timer))
            goal_a = torch.tensor(PUT_BACK_ABOVE, device=dev)
            goal_b = torch.tensor(PUT_BACK_RELEASE, device=dev)
            pb_goal = torch.where((pb_stage == 0)[:, None], goal_a, goal_b)
            q_arm = robot.data.joint_pos.torch[:, arm_ids]
            pb_err = (q_arm - pb_goal).abs().amax(-1)
            at_above = putback & (pb_stage == 0) & (pb_err < 0.03)
            at_release = putback & (pb_stage == 1) & ((pb_err < 0.03) | (timer > 400))
            pb_stage = torch.where(at_above, torch.ones_like(pb_stage), pb_stage)
            phase = torch.where(at_release, torch.full_like(phase, RELEASE), phase)
            timer = torch.where(at_release, torch.zeros_like(timer), timer)
            pb_active = (phase == PUTBACK)
            if recognize is not None:
                for e in at_release.nonzero().flatten().tolist():
                    checks.append((e, int(held_slot[e]), t + 150))
                    carried.append(float((s.sphere_pos[e, int(held_slot[e])] - tcp[e]).norm()))
            if bool(at_release[0]):
                print(f"t={t:5d} env0: put-back pose reached, tool tip at {[round(v, 3) for v in tcp[0].tolist()]}")
            to_release = carry & (reached | (timer > 300))
            to_place = to_release & args_cli.regrasp
            to_release = to_release & ~to_place
            phase = torch.where(to_place, torch.full_like(phase, PLACE), phase)
            phase = torch.where(to_release, torch.full_like(phase, RELEASE), phase)
            timer = torch.where(to_release | to_place, torch.zeros_like(timer), timer)
            placed = place & (((tcp[:, 2] - Z_PLACE).abs() < 0.008) | (timer > 250))
            place_xy = torch.where(placed[:, None], tcp[:, :2], place_xy)
            phase = torch.where(placed, torch.full_like(phase, RELEASE), phase)
            timer = torch.where(placed, torch.zeros_like(timer), timer)
            done_release_all = release & (timer >= 25)
            to_adjust = done_release_all & args_cli.regrasp & ~did_regrasp
            done_release = done_release_all & ~to_adjust  # the final release: the piece stays on the table, outside the box
            phase = torch.where(to_adjust, torch.full_like(phase, ADJUST), phase)
            phase = torch.where(done_release, torch.full_like(phase, BACK), phase)
            timer = torch.where(done_release_all, torch.zeros_like(timer), timer)
            to_regrasp = adjust & (timer >= 25)
            phase = torch.where(to_regrasp, torch.full_like(phase, REGRASP), phase)
            timer = torch.where(to_regrasp, torch.zeros_like(timer), timer)
            phase = torch.where(regrasp_done, torch.full_like(phase, RELEASE), phase)  # after reading the maps: let go
            did_regrasp = did_regrasp | regrasp_done
            timer = torch.where(regrasp_done, torch.zeros_like(timer), timer)
            done_release_count = (done_release & ~unk).long()  # a piece put back in the box does not count
            picked = picked + done_release_count  # the sphere is dropped: count it
            done_back = back & (timer >= 15)
            phase = torch.where(done_back, torch.full_like(phase, RETURN), phase)
            timer = torch.where(done_back, torch.zeros_like(timer), timer)
            home = ((centre - tcp[:, :2]).norm(dim=-1) < 0.05) & ((tcp[:, 2] - HOME_Z).abs() < 0.04)
            done_return = ret & (home | (timer >= 200))
            last_pick = torch.where(done_return & ~unk, torch.full_like(last_pick, float(t)), last_pick)  # only a piece carried out is progress
            unk = unk & ~done_return
            phase = torch.where(done_return, torch.full_like(phase, PICK), phase)
            timer = torch.where(done_return, torch.zeros_like(timer), timer)
            if bool(done_release[0]) and not bool(unk[0]):
                print(f"t={t:5d} env0: sphere {int(picked[0])} dropped; {int(spheres_in_box()[0])} left in the box")
            giving_up = active & (phase == PICK) & ((t - last_pick) > args_cli.give_up)
            if giving_up.any():
                active &= ~giving_up
                print(f"t={t:5d} envs stopped (no pick for {args_cli.give_up} steps): {giving_up.nonzero().flatten().tolist()}")
            if not active.any():
                break
            if args_cli.measure_gate:
                d = sensor.data
                args = (d.pos_w.torch, d.quat_w.torch, d.normal_force_matrix_w.torch, d.contact_pos_w.torch)
                kw = dict(pad_order=sensor._pad_order, force_floor=sensor.cfg.force_floor, sigma=sensor.cfg.sigma)
                gated = taxel_force_map(*args, taxels_local=sensor._taxels_local, max_distance=0.008, **kw)
                plain = taxel_force_map(*args, taxels_local=sensor._taxels_local, max_distance=None, **kw)
                old = taxel_force_map(*args, taxels_local=old_local, max_distance=None, **kw)
                live = active.clone()
                touching = plain.sum((-1, -2)) > 0
                meas["steps"] += int(live.sum())
                meas["contact"] += int((touching & live).sum())
                changed = ((plain - gated).abs().amax((-1, -2)) > 0.01) & live
                meas["gate_changes"] += int(changed.sum())
                meas["removed"] += float(((plain - gated).sum((-1, -2)) * live).sum())
                meas["total"] += float((plain.sum((-1, -2)) * live).sum())
                meas["max_gate"] = max(meas["max_gate"], float((plain - gated).abs().max()))
                meas["max_geom"] = max(meas["max_geom"], float((plain - old).abs().max()))
            if recognize is not None and checks and checks[0][2] <= t:
                due = [c for c in checks if c[2] <= t]
                checks[:] = [c for c in checks if c[2] > t]
                for e, sl, _ in due:
                    landed.append(s.sphere_pos[e, sl].cpu())
            caption["phase"] = int(phase[0])
            obs, *_ = env.step(act)
            if args_cli.record_grasps or recognize is not None or args_cli.regrasp:  # the maps after this step (what the next decision sees): a rolling history
                hist_maps = torch.cat([hist_maps[:, 1:], shape_mdp.tactile_map_shape(env, log_scale=False)[:, None]], dim=1)
                hist_closure = torch.cat([hist_closure[:, 1:], env.scene["robot"].data.joint_pos.torch[:, s.finger_id][:, None]], dim=1)
        left = spheres_in_box()
        s = tp.get_state(env, TASK)
        rel = s.sphere_pos[0, :, :2] - centre
        for i in range(rel.shape[0]):
            r = rel[i]
            inside = bool((r.abs() < BOX_INNER / 2).all() and s.sphere_pos[0, i, 2] < RIM_Z + 0.02)
            if inside:
                in_zone = bool((r.abs() < reach).all())
                print(f"left in box: sphere {i} at x {r[0]:+.3f} y {r[1]:+.3f} from the centre ({'reachable zone' if in_zone else 'out of reach, ' + f'{BOX_INNER / 2 - r.abs().max():.3f} m from the nearest wall'})")
        outside = ((s.sphere_pos[..., :2] - centre).abs() >= BOX_INNER / 2 + 0.02).any(-1)
        if args_cli.measure_gate:
            print(f"RESULT gate: env-steps {meas['steps']}, with any tactile contact {meas['contact']} ({100 * meas['contact'] / max(meas['steps'], 1):.1f} %)")
            print(f"RESULT gate: env-steps where it changes the map by > 0.01 N: {meas['gate_changes']} ({100 * meas['gate_changes'] / max(meas['contact'], 1):.2f} % of the steps with contact)")
            print(f"RESULT gate: share of the total tactile force it removes: {100 * meas['removed'] / max(meas['total'], 1e-9):.3f} %; largest change of one taxel {meas['max_gate']:.2f} N")
            print(f"RESULT geometry: largest difference of one taxel between the old rounded grid and the exact one: {meas['max_geom']:.3f} N")
        if dbg and not args_cli.regrasp:
            dd = torch.cat(dbg)
            dist = dd[:, :3].norm(dim=-1) * 100
            print(f"RESULT hold: distance from the tool point to the piece's centre when the piece is lifted (cm): median {dist.median():.2f}, "
                  f"p90 {dist.quantile(0.9):.2f} over {len(dd)} grasps (|dx| |dy| |dz| medians {(dd[:, :3].abs().median(0).values * 100).tolist()})")
        if args_cli.regrasp and dbg:
            dd = torch.cat(dbg)
            ld = dd[:, 3] > 0.5
            print(f"RESULT regrasp: {int(ld.sum())} of {len(dd)} second grasps hold the piece; offset of the piece from the tool point (cm, world x y z) "
                  f"median |.| of those that hold {(dd[ld, :3].abs().median(0).values * 100).tolist()}, of those that do not {(dd[~ld, :3].abs().median(0).values * 100).tolist() if (~ld).any() else '-'}; "
                  f"median |estimated offset| {(dd[:, 4:].norm(dim=-1).median() * 100):.2f} cm")
        if recognize is not None:
            total = int(confusion.sum())
            answered = int(confusion[:, :6].sum())
            correct = int(confusion[:, :6].diag().sum())
            print(f"RESULT recognition: {total} pieces held and read; {answered} named ({100 * answered / max(total, 1):.1f} %), {total - answered} unknown and put back; "
                  f"of the {answered} named, {correct} correctly ({100 * correct / max(answered, 1):.1f} %); counting unknown as wrong: {100 * correct / max(total, 1):.1f} %")
            print("RESULT                 " + " ".join(f"{n[:5]:>5s}" for n in recog_lib.CLASSES) + "  unkn")
            for i, name in enumerate(recog_lib.CLASSES):
                print(f"RESULT   {name:10s} " + " ".join(f"{int(v):5d}" for v in confusion[i]) + f"   recall when answered {int(confusion[i, i]) / max(int(confusion[i, :6].sum()), 1):.2f}, unknown {int(confusion[i, 6]) / max(int(confusion[i].sum()), 1):.0%}")
            if carried:
                cd = torch.tensor(carried)
                print(f"RESULT put back, at the release pose: {int((cd < 0.06).sum())} of {len(cd)} pieces are still in the gripper (within 6 cm of the tool point), "
                      f"median distance {cd.median() * 100:.1f} cm")
            if landed:
                lp = torch.stack(landed)
                rel = (lp[:, :2] - torch.tensor(BOX_CENTER)).abs()
                in_box = (rel < BOX_INNER / 2).all(-1) & (lp[:, 2] < RIM_Z + 0.02)
                in_zone = (rel < torch.tensor(REACH_HALF)).all(-1) & in_box
                print(f"RESULT put back, 150 steps after the release: {len(lp)} pieces, {int(in_box.sum())} lie in the box, {int(in_zone.sum())} in the tool tip's reach, "
                      f"{int((~in_box).sum())} outside the box; median height of those in the box {lp[in_box, 2].median() * 100 if in_box.any() else float('nan'):.1f} cm "
                      f"(a piece lying on the floor: 1.2 + 2 to 2.5 cm)")
            # what happens to a piece that was put back: is it held again, and named?
            seq = {}
            for e, sl, tt, tr, g in events:
                seq.setdefault((e, sl), []).append((tt, tr, g))
            back = [v for v in seq.values() if v[0][2] < 0]
            again = [v for v in back if len(v) > 1]
            if back:
                outcome = [v[1][2] for v in again]
                right = sum(1 for v in again if v[1][2] == v[1][1])
                print(f"RESULT put back: {len(back)} pieces were put back; {len(again)} of them were held and read again; of those, {right} named correctly, "
                      f"{sum(1 for o in outcome if o >= 0) - right} named wrongly, {sum(1 for o in outcome if o < 0)} unknown again")
        if args_cli.record_grasps:
            import numpy as np

            out = {k: torch.cat(v).numpy() for k, v in rec.items() if v}
            np.savez_compressed(args_cli.record_grasps, **out)
            print("RESULT recorded grasps:", len(out.get("label", [])), "per class", np.bincount(out["label"], minlength=6).tolist() if len(out.get("label", [])) else [], "->", args_cli.record_grasps)
        print("RESULT per-env picked:", picked.tolist())
        completed = tp.get_state(env, TASK).n_picked
        print(f"RESULT picks completed by the task itself, per env (mean/max): {completed.float().mean().item():.2f} / {int(completed.max())} (0 = the supervisor's count is the only one)")
        print("RESULT envs", n, "steps", t + 1)
        print("RESULT spheres lying outside the box per env (mean/min/max):", f"{outside.sum(-1).float().mean().item():.1f} / {int(outside.sum(-1).min())} / {int(outside.sum(-1).max())}")
        print("RESULT spheres picked per env (mean/min/max):", f"{picked.float().mean().item():.1f} / {int(picked.min())} / {int(picked.max())}")
        print("RESULT spheres left in the box per env (mean/min/max):", f"{left.float().mean().item():.1f} / {int(left.min())} / {int(left.max())}")
        print("RESULT spheres left inside the reachable zone per env (mean/min/max):", f"{spheres_in_box(True).float().mean().item():.1f} / {int(spheres_in_box(True).min())} / {int(spheres_in_box(True).max())}")
        print("RESULT envs with an empty box:", f"{(left == 0).float().mean().item():.2f}")
        if args_cli.hq_video:
            hq_writer.close()
        env.close()


if __name__ == "__main__":
    main()
