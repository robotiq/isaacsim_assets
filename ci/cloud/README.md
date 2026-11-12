# Runtime stamps on a GCP GPU

`stamp_on_gcp.py` creates an on-demand `n1-standard-64` with 4 T4 GPUs (SSD boot disk) in your GCP project from the
`isaacsim-6-2` image, runs the sharded runtime tests there, copies the stamps back and deletes the VM. The default shard
map is `Robotiq_2F_85=3`, `Robotiq_2F_140=3`, `Robotiq_Hand_E=1`, `Robotiq_Hand_E_C10=1`: 8 pytest processes, pinned
round-robin to the GPUs (2 per T4).

Options:

- `--gpus 1|2|4` (default 4): T4s on the VM; shard `i` gets `CUDA_VISIBLE_DEVICES=i mod N`.
- `--machine-type n1-standard-16|n1-standard-32|n1-standard-64` (default `n1-standard-64`). Pair a smaller machine with
  fewer GPUs, e.g. `--machine-type n1-standard-16 --gpus 1`.
- `--shards SCOPE=N` (repeatable, N from 1 to 32) overrides a scope's shard count.
- `--spot`: a Spot VM, cheaper but GCP can preempt it.
- `--zone`, `--scope`, `--base`, `--target`: see `--help`.

A sampler on the VM appends RAM, load and, per GPU, memory and utilization every 10 s to `~/out/resources.log`
(fetched with the rest of `~/out`); the launcher prints the peak RAM and load, then each GPU's peak memory and
average utilization.

## Measured

A full 4-gripper stamp on `n1-standard-64` + 4 T4 (on-demand, us-central1) takes about 12 min to results: create
~2 min, SSH and code shipping ~1 min, tests ~9 min, fetch ~15 s. The tests are GPU-bound: one T4 is saturated whatever
the number of processes on it, so more GPUs is what shortens a run, not more shards. The machine costs about $4.4/h
on-demand, about $1 per run.

## One-time setup

### Tools

- A GCP project with Compute Engine, T4 quota, and a VPC network `isaac-runtime-stamp` with a subnet
  `isaac-runtime-stamp-us-central1`, reserved for these VMs: no peering, VPN or router, no other VMs, egress to
  private ranges denied. The launcher takes the project from `--project <your-project>` or
  `ISAAC_STAMP_GCP_PROJECT=<your-project>`; the image is looked up in the same project.
- `gcloud`, logged in, and Application Default Credentials for Packer (`gcloud auth application-default login`).
- `roles/iap.tunnelResourceAccessor` on the project (DevOps grants it to the maintainers' group): SSH to the VMs, the
  launcher's and Packer's, only goes through an IAP TCP tunnel (`gcloud compute start-iap-tunnel`).
- `git-lfs`, `ssh`, `scp` and `ssh-keygen`; the LFS objects of the commit to stamp present locally (`git lfs pull`).
- Packer as a user binary (no sudo): download `packer_<version>_linux_amd64.zip` and `packer_<version>_SHA256SUMS` from
  `https://releases.hashicorp.com/packer/<version>/`, check with `sha256sum -c --ignore-missing`, unzip `packer` into
  `~/.local/bin`. Check: `packer version`.

### GPU quota

`NVIDIA_T4_GPUS` (and `PREEMPTIBLE_NVIDIA_T4_GPUS` for `--spot`) in `us-central1`: at least 4 for a default run, plus
1 for the image build.

### Image

Packer boots an `n1-standard-8` with a T4 and no service account from the Deep Learning VM base
(`common-cu129-ubuntu-2204-nvidia-580`), runs `packer/provision.sh` (git, git-lfs, ROS 2 Humble `ros-humble-ros-base`,
Isaac Sim 6.2.0 in `/opt/isaacsim` with pytest, a headless warm-up that fills the shader caches, and `/opt/repo`: a
clone of this repo's `main` from GitHub with its LFS objects, then a run of `test_gripper_closes` for Robotiq_Hand_E in
`/opt/repo` that compiles the Newton warp kernels into `/opt/isaac-home/.cache/warp`, ~2 min each run's first Newton
test would otherwise spend) and saves family `isaacsim-6-2`. Rebuild it when Isaac Sim or the ROS packages change, and
when `main` has drifted far from the image's `/opt/repo`: a run ships the git bundle and LFS objects its commit adds to
it, which grows with the drift.

```bash
export ISAAC_STAMP_GCP_PROJECT=<your-project>
cd ci/cloud/packer && packer init . && packer build -var project=<your-project> -var owner=<your-name> .
```

The build connects with Packer's temporary metadata SSH key, not OS Login. If your ADC quota project is another project, prefix the build with `GOOGLE_CLOUD_QUOTA_PROJECT=<your-project>`.

On a T4 stockout in us-central1-b, rebuild with `-var zone=us-central1-a` (or `us-central1-c`, `us-central1-f`).

Afterwards `gcloud compute instances list --project <your-project> --filter=labels.purpose=isaac-runtime-stamp` must be empty.

## Running a stamp

```bash
python3 ci/cloud/stamp_on_gcp.py                       # scopes changed against origin/main
python3 ci/cloud/stamp_on_gcp.py --scope Robotiq_Hand_E --target ../other/worktree
python3 ci/cloud/stamp_on_gcp.py --spot                # a Spot VM: cheaper, but GCP can preempt it
python3 ci/cloud/stamp_on_gcp.py --machine-type n1-standard-16 --gpus 1 --scope Robotiq_Hand_E
```

The stamps are written into the target's `ci/runtime/stamps` and staged; commit them.

When some tests fail on the VM, the run keeps a partial result per failing scope (`stamp.py --keep-partial`), fetched
to `~/.cache/isaac-stamp/<vm>/out/partial/`, and the launcher prints the command that finishes it locally: `stamp.py
--complete <partial>` re-runs only the failed tests in one Isaac process and writes the stamp if they pass (see
`ci/runtime/README.md`).

How a run goes: a per-run SSH key pair is made in a temp dir and put in the VM's metadata for user `stamp` (deleted
with the temp dir; project-wide SSH keys are blocked on the VM); the launcher reaches the VM with plain `ssh`/`scp`
through an IAP tunnel (`ProxyCommand=gcloud compute start-iap-tunnel <vm> 22 --listen-on-stdin`). The VM keeps an
external IP for its internet egress (the network has no Cloud NAT). On the VM, `/opt/repo` is cloned to `~/repo`;
the laptop sends a git bundle of `<image main>..HEAD` (the whole `HEAD` if the target does not contain the image's
`main`) and only the LFS objects `~/repo` lacks, then checks out `HEAD` and refuses to go on if an LFS file is
still a pointer or `git status --porcelain` is not empty. Zones are tried in the order us-central1-b, -a, -c, -f on a stockout.
With `--spot`, a preemption while the VM is being created is retried the same way, across the zones (6 attempts in all).

Timings: the launcher prints `[stamp-on-gcp] <time> +<s>s <phase>` for start, vm-created, ssh-ready, code-shipped,
remote-run-done, fetched and deleted; the VM's `stamp.py` prints `[stamp] <time> <event>` for each shard's
start/end (the shards collect their tests themselves, see `ci/runtime/README.md`). The VM's log is kept in `~/.cache/isaac-stamp/<vm>/out/stamp.log`.

## Costs and guards

- `--max-run-duration=2h` with `--instance-termination-action=DELETE`: GCP deletes the VM after two hours even if the
  launcher dies.
- The launcher deletes the VM synchronously, after printing the results, on success, failure, Ctrl-C, SIGTERM and
  SIGHUP (a closed terminal), in every zone a create was tried in; a failed delete prints the manual command and makes
  the launcher exit non-zero.
- One stamp VM at a time: the launcher refuses to start while a VM labelled `purpose=isaac-runtime-stamp` exists.
- Every gcloud, ssh and scp command has a timeout (the remote test run: 1 h), so a hung step ends in the delete.
- No service account and no scopes on the VM (nor on the Packer build VM). SSH only with the run's temporary key,
  through IAP: the `isaac-runtime-stamp-ssh` firewall rule (managed in Terraform) admits port 22 on the
  `isaac-runtime-stamp` tag only from IAP's range `35.235.240.0/20`, the only ingress rule on the network.
- Everything carries the labels `purpose=isaac-runtime-stamp` and `owner=<user>`, network `isaac-runtime-stamp`, tag
  `isaac-runtime-stamp`.

## Running Isaac Sim locally

Running Isaac Sim locally can saturate a laptop GPU and its memory. Keep a local run to a handful of tests in one
Isaac process, one run at a time, and run `stamp.py --complete` under resource limits, as the command the launcher
prints does, e.g.:

```bash
systemd-run --user -P -p MemoryMax=16G -p MemorySwapMax=0 -p CPUWeight=20 -p IOWeight=20 \
  python3 ci/runtime/stamp.py --complete <partial>
```
