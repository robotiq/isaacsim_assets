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
parser.add_argument("--video", default=None, help="record env 0 into this directory")
parser.add_argument("--hq_video", default=None, help="record env 0 to this .mp4 with an RTX camera, 1920 x 1080")
parser.add_argument("--hq_pressure_map", action="store_true", help="draw the TSF pressure map of both fingertips on the lower left of --hq_video")
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

PICK, CARRY, RELEASE, BACK, RETURN = 0, 1, 2, 3, 4
HOME_Z = 0.28  # m: tool-tip height above the box centre where the policy takes over again (inside its training starts)
CARRY_Z = 0.22  # m: tool tip height while carrying (sphere bottom ~ 18 cm: clear of the 11 cm walls)
GAIN = 60.0  # action per metre of error (an action of 1 is 1 cm per step)


def drop_point(k: torch.Tensor) -> torch.Tensor:
    """Where the k-th sphere of an env is dropped: beside the box on the +y side, on a grid so they do not pile up."""
    x = BOX_CENTER[0] - 0.14 + 0.07 * (k % 5).float()
    y = BOX_CENTER[1] + BOX_INNER / 2 + 0.10 + 0.07 * (k // 5).float()
    return torch.stack([x, y], dim=-1)


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    env_cfg.episode_length_s = 3600.0  # no time-out reset: this is one long run
    if args_cli.seed is not None:
        torch.manual_seed(args_cli.seed)
        env_cfg.seed = args_cli.seed
    if args_cli.gate is not None:
        env_cfg.scene.tactile.max_contact_distance = None if args_cli.gate < 0 else args_cli.gate
    reset = env_cfg.events.reset_spheres.params
    reset["num_active_range"] = None  # all ten spheres are in the box
    reset["p_zone_only"] = 1.0 if args_cli.layout == "reachable" else 0.0
    total = args_cli.steps
    if args_cli.video:
        env_cfg.video_recorders = [VideoRecorderCfg(source="viz", output_dir=args_cli.video, video_length=total)]
    if args_cli.hq_video:
        env_cfg.sim.render_interval = 120 // args_cli.hq_fps  # a render (and a frame) every this many physics steps
        eye, target = torch.tensor([1.15, -0.75, 0.75]), torch.tensor([0.5, 0.0, 0.1])  # the viewer's camera
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
                    taxels = env.scene["tactile"].taxel_force[0].cpu().numpy()  # (2, 7, 4) N, after the sensor's contact-distance gate
                    frame = overlay_lower_left(frame, pressure_panel(taxels))
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
            phase = torch.where(start_carry, torch.full_like(phase, CARRY), phase)
            target = drop_point(picked)
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
            ret = phase == RETURN
            ret_act = torch.zeros_like(act)
            ret_act[:, 0:2] = (GAIN * (centre - tcp[:, :2])).clamp(-1, 1)
            ret_act[:, 2] = (GAIN * (HOME_Z - tcp[:, 2])).clamp(-1, 1)
            ret_act[:, 6] = -1.0
            use = lambda m: m[:, None]  # noqa: E731
            act = torch.where(use(carry), scripted, act)
            act = torch.where(use(release), rel_act, act)
            act = torch.where(use(back), back_act, act)
            act = torch.where(use(ret), ret_act, act)
            act = torch.where(use(~active), torch.zeros_like(act).index_fill_(1, torch.tensor([6], device=dev), -1.0), act)
            # --- phase transitions
            timer = torch.where(carry | release | back | ret, timer + 1, torch.zeros_like(timer))
            to_release = carry & (reached | (timer > 300))
            phase = torch.where(to_release, torch.full_like(phase, RELEASE), phase)
            timer = torch.where(to_release, torch.zeros_like(timer), timer)
            done_release = release & (timer >= 25)
            phase = torch.where(done_release, torch.full_like(phase, BACK), phase)
            timer = torch.where(done_release, torch.zeros_like(timer), timer)
            done_release_count = done_release.long()
            picked = picked + done_release_count  # the sphere is dropped: count it
            done_back = back & (timer >= 15)
            phase = torch.where(done_back, torch.full_like(phase, RETURN), phase)
            timer = torch.where(done_back, torch.zeros_like(timer), timer)
            home = ((centre - tcp[:, :2]).norm(dim=-1) < 0.05) & ((tcp[:, 2] - HOME_Z).abs() < 0.04)
            done_return = ret & (home | (timer >= 200))
            last_pick = torch.where(done_return, torch.full_like(last_pick, float(t)), last_pick)
            phase = torch.where(done_return, torch.full_like(phase, PICK), phase)
            timer = torch.where(done_return, torch.zeros_like(timer), timer)
            if bool(done_release[0]):
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
            obs, *_ = env.step(act)
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
