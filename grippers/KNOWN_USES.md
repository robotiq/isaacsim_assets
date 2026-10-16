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

---

## Factorized Tactile Representation and Control for Sim-to-Real Manipulation (Shang, Aumann, Brady, Migdal, Padır, Amazon / UT Austin, 2026)

- **Link:** [arXiv:2610.10510](https://arxiv.org/pdf/2610.10510)
- **Gripper(s):** Robotiq 2F-85 on a UR5e, with two TSF-85 capacitive tactile pads
- **Variant:** not stated (presumably parallel grip, as the tactile pads are used for flat-face and object grasps)
- **Simulator / physics engine:** Isaac Sim 5.1.0 and Isaac Lab 2.3.2 (PPO with an LSTM actor, 4096 environments per A100, 10 Hz policy with 12 physics steps per policy step). The physics engine is not named.
- **Asset source:** per Robotiq, the work uses the TSF-85 and the ÉTS Isaac Sim digital twin. The paper itself does not say this. It cites the ÉTS TSF-85 digital twin as [10] (De la Cruz Sánchez and Roberge, [Applied Sciences 16(15):7708, 2026](https://www.mdpi.com/2076-3417/16/15/7708)) but says its own approach avoids reconstructing a deformation field, and it does not say which 2F-85 asset it used.

### How the gripper was used
- Three sim-to-real RL tasks, transferred zero-shot to hardware: Tactile Alignment (hold an 80 mm cylinder and align it in-hand), Grasp-Force Regulation, and Adversarial Peg Insertion.
- Grasp-Force Regulation drives the 2F-85 in **position mode through its integer 0-255 position register**. One tick is about 3.42 mrad of finger motion. The action is an incremental change to the position setpoint at each policy step. Targets span 2 N to 40 N.
- Training uses 2048 environments and 404 object meshes from EGAD! and YCB. Each object is two plates coupled by a prismatic spring with stiffness randomized over a 25x range, so gripper position is an unreliable proxy for force.
- The gripper is used as the mount for the tactile pads. The contribution is the tactile representation and control, not the gripper model.

### How the asset was tuned
- **Command latency:** system identification on hardware measured about **100 ms of gripper-command latency**, which is reproduced in simulation.
- **Randomization:** gripper dynamics, friction, object mass and grasp pose are randomized. The ranges are not given in the text I read.
- **Pad contact model:** the tactile pads are modeled with a compliant pad-contact model of stiffness k = 5 x 10^4 N/m, validated on 92 grasps with flat punches and spheres (about 30-count RMSE, median response deviation under 3 %, within 10 % RMSE across a 20x stiffness range). The authors note precise stiffness identification is not required.
- The paper does not report changes to the 2F-85 joint, drive, linkage or collision parameters.

### Findings / feedback for us
- Real users drive the gripper through the **0-255 position register**. A sim asset that exposes the same command (position setpoint, about 3.42 mrad per tick) and a realistic command latency would remove the need to patch these in the training environment.
- The 100 ms command latency is a measured number we could document or offer as an optional delay in the asset or demo.
- Follow up on the ÉTS digital twin: how it relates to the pad contact model used in this paper, and whether it builds on our 2F-85 asset.
- Open questions: which 2F-85 asset they used, the gripper-dynamics and friction ranges they randomized, and how the TSF-85 pads were attached to the asset.
