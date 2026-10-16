# Known uses of the Robotiq grippers in Isaac Sim

A running list of published work that uses our gripper assets in simulation. For
each one we record where to find it, how the gripper was used, and how it was
tuned, so we can learn what users had to change and feed it back into the assets.

If you use these assets in your work, please open an issue or PR to add a section.

## Template

Copy this block for a new entry.

```markdown
## <Short title> (<Authors / Org>, <Year>)

- **Link:** <paper / repo / project page>
- **Gripper(s):** <2F-85 | 2F-140 | Hand-E>, <robot arm it is mounted on>
- **Variant:** <grip mode / asset variant used, and any mechanical lock such as dowel pins on the real gripper>
- **Simulator / physics engine:** <Isaac Sim / Isaac Lab version, PhysX | Newton | MuJoCo>
- **Asset source:** <this repo, another repo, or unknown>

### How the gripper was used
<Task, control interface, open/close command, observations that involve the gripper.>

### How the asset was tuned
<Friction and combine mode, drives, joint limits, collision geometry, anything patched.>

### Findings / feedback for us
<What worked, what did not, what we should change in the asset.>
```

---

## A Balanced Data Diet (SGS) (Zhang, Guaman Castro, Yin et al., University of Washington / NVIDIA, 2026)

- **Link:** [arXiv:2610.12465](https://arxiv.org/abs/2610.12465), [project page](https://sgs-rl.github.io/)
- **Gripper(s):** Robotiq 2F-85 on a UR5e (the Franka tasks use its stock parallel-jaw gripper)
- **Variant:** parallel grip. The real gripper was fitted with dowel pins that lock it in the
  parallel grip mode.
- **Simulator / physics engine:** not stated in the paper. It builds on the OmniReset pipeline. Probably Physx
- **Asset source:** not stated. The paper does not say it uses this repo's asset.

### How the gripper was used
- Six contact-rich UR5e assembly tasks (threading, precise insertion, connector mating,
  gear alignment) on the NIST Assembly Taskboard, trained with RL using massively parallel
  simulation and diverse simulator resets.
- Control follows OmniReset's relative-Cartesian operational-space interface.
- The gripper is commanded with a single binary open/close signal. Reset generation
  randomizes the gripper aperture.
- A point-cloud teacher is trained in simulation, then distilled with DAgger into an RGB
  student that is transferred zero-shot to real hardware (no real-world data, no fine-tuning).

### How the asset was tuned
Pad friction was the only gripper parameter the paper reports tuning (Appendix C.2):
- OmniReset gave the 2F-85 pads a friction coefficient of **µ = 100** with the **max**
  friction-combine mode. This makes grasping easy and training fast, but the authors found
  it to be a significant source of sim-to-real gap: any grasp that applies normal force
  succeeds, because the object stays frozen between the pads even in physically
  implausible grasps.
- Lower friction removes those grasps, but policies trained from scratch at low friction
  failed to learn at all. To find the lowest friction that still trains, they trained
  rod-in-hole insertion from scratch at fixed pad friction µ = 0.5, 1, 1.5 and 2, keeping
  the max combine mode and all other settings identical.
- Only **µ = 2** trained stably, so they used **µ = 2 with max combine** for all runs.
- Other simulation modeling was light: Gaussian noise on the point-cloud observations, object
  masses randomized around measured values, and OmniReset's visual randomization.
  The paper reports no changes to joint, drive or collision parameters.

### Findings / feedback for us
- Very high pad friction (µ = 100) is a known sim-to-real hazard. Consider documenting a
  recommended pad friction and combine mode for RL users. µ ≈ 2 with max combine is a
  citable lower bound for stable training in this setup.
- Open question: which 2F-85 asset OmniReset uses and how its drives and linkage are
  modeled. We have not checked.
