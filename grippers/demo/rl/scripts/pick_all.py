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
parser.add_argument("--video", default=None, help="record env 0 into this directory")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
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
    reset = env_cfg.events.reset_spheres.params
    reset["num_active_range"] = None  # all ten spheres are in the box
    reset["p_zone_only"] = 1.0 if args_cli.layout == "reachable" else 0.0
    total = args_cli.steps
    if args_cli.video:
        env_cfg.video_recorders = [VideoRecorderCfg(source="viz", output_dir=args_cli.video, video_length=total)]
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        policy = torch.jit.load(args_cli.policy, map_location=env.device)
        obs, _ = env.reset()
        n, dev = env.num_envs, env.device
        phase = torch.full((n,), PICK, device=dev)
        held_count = torch.zeros(n, device=dev)
        timer = torch.zeros(n, device=dev)
        picked = torch.zeros(n, dtype=torch.long, device=dev)
        last_pick = torch.zeros(n, device=dev)
        active = torch.ones(n, dtype=torch.bool, device=dev)
        act = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
        centre = torch.tensor(BOX_CENTER, device=dev)

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
        print("RESULT envs", n, "steps", t + 1)
        print("RESULT spheres lying outside the box per env (mean/min/max):", f"{outside.sum(-1).float().mean().item():.1f} / {int(outside.sum(-1).min())} / {int(outside.sum(-1).max())}")
        print("RESULT spheres picked per env (mean/min/max):", f"{picked.float().mean().item():.1f} / {int(picked.min())} / {int(picked.max())}")
        print("RESULT spheres left in the box per env (mean/min/max):", f"{left.float().mean().item():.1f} / {int(left.min())} / {int(left.max())}")
        print("RESULT spheres left inside the reachable zone per env (mean/min/max):", f"{spheres_in_box(True).float().mean().item():.1f} / {int(spheres_in_box(True).min())} / {int(spheres_in_box(True).max())}")
        print("RESULT envs with an empty box:", f"{(left == 0).float().mean().item():.2f}")
        env.close()


if __name__ == "__main__":
    main()
