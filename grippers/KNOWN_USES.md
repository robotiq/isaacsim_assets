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

---

## Astranomy: An Empirical Study of GPT-6 Astra as a Native Robot Policy (Yang, Khetan, Zhang, NVIDIA Seattle Robotics Lab, 2026)

- **Link:** [project page](https://research.nvidia.com/labs/srl/projects/astranomy/) (arXiv link listed as coming soon). The page's Code link points to NVIDIA's internal GitLab, which is probably not public. The same policy code is in the public [NVlabs/RoboLab](https://github.com/NVlabs/RoboLab) repo under `policies/ai_native`.
- **Gripper(s):** Robotiq 2F-85 on the DROID robot (Franka Panda arm)
- **Variant:** not stated
- **Simulator / physics engine:** [RoboLab](https://github.com/NVlabs/RoboLab), built on Isaac Lab (Isaac Sim 5.1, with an Isaac Sim 6 / Isaac Lab 3.0 beta path), run on NVIDIA L40s GPUs. PhysX by default (an Isaac Lab `ArticulationCfg`). Control runs at 15 Hz with simulation paused during inference.
- **Asset source:** not this repo's asset. RoboLab ships its own `assets/robots/franka_robotiq_2f_85_flattened.usd` (a flattened Franka + Robotiq 2F-85), plus an Isaac Sim 6 derivative `franka_robotiq_2f_85_isaac60.usd`. The repo's notices license `assets/robots` as CC BY-NC-SA 4.0 (c) NVIDIA. Its Kinova Gen3 + 2F-85 asset derives from `ros2_kortex` and PickNik `ros2_robotiq_gripper` (BSD-3).

### How the gripper was used
- A general-purpose multimodal language model is evaluated as a native robot policy for language-conditioned manipulation, without robot-specific finetuning. Three policy configurations are compared on 120 tasks (RoboLab-120): a direct policy (independent API calls, no context kept between decisions), an agentic policy (a persistent Codex session with interaction history and optional tools), and an orchestrated policy (the model acts as a monitor reviewing actions proposed by Cosmos3-Nano-Policy, and can intervene).
- Three synchronized RGB views (left shoulder, right shoulder, wrist) at 384x216.
- The policy outputs absolute end-effector poses in the robot-root frame, `[x, y, z, qw, qx, qy, qz, gripper]`, with a binary open/close gripper command. The chunk length depends on the configuration: the direct policy chooses dynamic chunks of 3-15 targets per call, the agentic policy executes each target for 1-5 control steps before the next observation, and the orchestrated policy uses fixed 15-step proposals from the learned model (Astra can override with shorter segments). The measured gripper state is a normalized closure `gripper_pos[0]` (0 = open, 1 = closed).
- Motion is limited to 0.02 m translation and 0.1 rad rotation per step.
- Reported success on RoboLab-120: agentic 74.8 %, orchestrated 66.5 %, direct 66.3 %.
- The gripper is described to the model in the prompt: finger-pad offsets, jaw geometry, and interpolated geometry tables that map closure to pad separation distance. This is prompt-side modeling of the gripper, not a change to the asset.
- **Gripper geometry prompt (section 3.1.2 of the page):** the prompt variants add gripper geometry, camera geometry, or both, and the gripper geometry text defines:
  - Pose frame: the action pose and `ee_pos`/`ee_quat` locate the Robotiq `base_link` mounting flange. The finger pads extend along flange-local +X, and the two opposing pads are separated along flange-local Y. Closing reduces that separation.
  - Closure: `g` is `gripper_pos[0]`, normalized closure (0 = open, 1 = fully closed), distinct from the binary gripper action that requests open or closed.
  - Approximate linkage geometry table: `d` is the midpoint between the pad centers along flange-local X, in metres, and the gap is the inner-face gap.

    | g | d (m) | inner-face gap (m) |
    |---|---|---|
    | 0.00 | 0.13010 | 0.0850 |
    | 0.25 | 0.13651 | 0.0666 |
    | 0.50 | 0.14101 | 0.0461 |
    | 0.75 | 0.14343 | 0.0242 |
    | 1.00 | 0.14366 | 0.0018 |

  - The prompt calls these "approximate linkage geometry" and "geometric reference values, not exact contact locations or a grasp detector", since contact can deflect the linkage. Intermediate openings are interpolated.
  - Pad pose: `p_pad = p_flange + R(ee_quat) * t_pad` with `t_pad = [d, 0, 0]`, and the flange target is `p_flange_target = desired_pad_position - R(target_quaternion) * t_pad`.

### How the asset was tuned
- The project page describes no simulation tuning of the gripper. The public RoboLab code (`robolab/robots/droid.py`, read via the GitHub API, not run) shows this:
  - **Actuator:** a single Isaac Lab `ImplicitActuatorCfg` on `finger_joint` only, with `stiffness=None` and `damping=None` (so the USD drive values are kept) and `velocity_limit=5.0`. The effort limit and alternative stiffness/damping values are commented out, which suggests they were tried. The other 2F-85 joints (`right_outer.*`, `left_inner.*`, `right_inner.*`) have no actuator and start at 0.
  - **Solver:** articulation self-collisions disabled, 64 position iterations, 0 velocity iterations; rigid bodies with gravity disabled and `max_depenetration_velocity=5.0`; contact sensors activated. Contact detection is on the `left_inner_finger` link.
  - **Asset patch for Isaac Sim 6 (rendering only):** their docs say Isaac Sim 6 "can otherwise omit [the Robotiq visuals] from RTX renders" while the visual prototype roots stay instanceable. They de-instanced exactly eight Robotiq visual prototype roots and left bodies, joints, collision prims and meshes unchanged, locked by `tests/test_droid_isaac60_asset.py`. The repo does not explain the root cause.
  - The docs also note NVIDIA's Isaac Lab 3.0 beta release notes list a known Robotiq 2F-85 issue, so the DROID and Kinova grippers need extra smoke testing.
- The gripper geometry given to the model in the prompt is not an asset change. It is described under "How the gripper was used".

### Findings / feedback for us
- The geometry table shows the pad midpoint moving out along the flange X axis as the gripper closes (d grows from 0.130 m to 0.144 m at g = 1.00), while the gap shrinks from 85 mm to 1.8 mm. `d` is strongly nonlinear in g (steps of 6.4, 4.5, 2.4 and 0.2 mm per 0.25 of closure), while the gap is close to linear (18.4, 20.5, 21.9 and 22.4 mm per 0.25). We could check our asset's pad frames against these numbers.
- The policy reasons about pad separation as a function of normalized closure. An asset that exposes closure (0-1) and pad separation consistently would make that table easy to derive and verify.
- Users hit a rendering problem with instanceable Robotiq visuals on Isaac Sim 6. Our default composition has the same structure: 11 instanceable `visuals` prims (one per link, including the fingertips) over 6 prototypes, with one mesh each. We rendered it in Isaac Sim 6.2.0 (standalone `SimulationApp`, default RTX, default composition) with the visuals instanceable and again with all 11 de-instanced: both show all meshes, and the images differ by only 662 of 480,000 pixels (render noise). So the problem did not reproduce on 6.2.0. It was not tested on other Isaac Sim 6 builds (6.0.x, 6.1) or inside Isaac Lab 3.0 beta, where RoboLab saw it.
- They leave the finger-joint drive gains to the USD (`stiffness=None`) and only cap velocity, so the drive values authored in the asset matter directly for them.
- Open questions: the friction and contact material on the pads, and which grip mode the flattened USD uses. I did not inspect the USD, and I did not read the `ai_native` policy code beyond the robot config.
