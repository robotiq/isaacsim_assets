#!/usr/bin/env bash
set -euo pipefail

printf 'Acquire::ForceIPv4 "true";\nAcquire::Retries "5";\n' | sudo tee /etc/apt/apt.conf.d/99isaac-stamp >/dev/null

ISAAC_URL="https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.2.0-linux-x86_64.zip"
ISAAC_MD5="dde88af0b6423dd3135727385a344fb3"
ISAAC_DIR=/opt/isaacsim
export DEBIAN_FRONTEND=noninteractive

nvidia-smi

APT="sudo apt-get -o DPkg::Lock::Timeout=300 -y"
$APT update
$APT install git git-lfs curl unzip ca-certificates software-properties-common \
  libgl1 libglu1-mesa libegl1 libglx0 libopengl0 libxrandr2 libxinerama1 libxcursor1 libxi6 libxt6 \
  libsm6 libice6 libxext6 libx11-6 libgomp1 libvulkan1
sudo git lfs install --system --skip-repo
sudo add-apt-repository -y universe
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" |
  sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
$APT update
$APT install ros-humble-ros-base

sudo mkdir -p "$ISAAC_DIR"
zip=/opt/isaac-sim.zip
sudo curl -fSL --retry 5 -o "$zip" "$ISAAC_URL"
echo "$ISAAC_MD5  $zip" | md5sum -c -
sudo unzip -q "$zip" -d "$ISAAC_DIR"
sudo rm -f "$zip"
cd "$ISAAC_DIR"
sudo ./post_install.sh
sudo ./python.sh -m pip install pytest
test -f "$ISAAC_DIR/VERSION"
head -n1 "$ISAAC_DIR/VERSION"
sudo chmod -R a+rX "$ISAAC_DIR"

# The launcher runs stamp.py with HOME=/opt/isaac-home so whichever user runs it reuses these caches.
sudo mkdir -p /opt/isaac-home
ldconfig -p | grep -q 'libGL.so.1' || { echo "libGL.so.1 not found: GL libraries missing" >&2; exit 1; }
warmup_log=/tmp/isaac-warmup.log
sudo env HOME=/opt/isaac-home OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y timeout 1500 \
  "$ISAAC_DIR/python.sh" -c 'from isaacsim import SimulationApp; SimulationApp({"headless": True}).close()' \
  >"$warmup_log" 2>&1 || true
cat "$warmup_log"
if grep -q 'cannot open shared object file' "$warmup_log"; then
  echo "warm-up hit missing shared libraries" >&2
  exit 1
fi
[ -n "$(sudo find /opt/isaac-home -type f | head -n1)" ] || { echo "warm-up left no cache in /opt/isaac-home" >&2; exit 1; }
sudo chmod -R a+rwX /opt/isaac-home "$ISAAC_DIR"

# Each run clones this and only ships what its commit adds, LFS objects included.
sudo env GIT_LFS_SKIP_SMUDGE=1 git clone -q --branch main https://github.com/robotiq/isaacsim_assets.git /opt/repo
sudo git -C /opt/repo lfs pull
[ -n "$(sudo find /opt/repo/.git/lfs/objects -type f | head -n1)" ] || { echo "no LFS object in /opt/repo" >&2; exit 1; }

# The SimulationApp warm-up never runs Newton: without this every run's first Newton test spends ~2 min compiling
# the warp kernels. A failing test does not matter here, but the cache must grow and a Newton case must pass.
warp_count() { sudo find /opt/isaac-home/.cache/warp -type f 2>/dev/null | wc -l; }
warp_before=$(warp_count)
(cd /opt/repo && sudo env HOME=/opt/isaac-home OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y \
  timeout 1800 "$ISAAC_DIR/python.sh" -m pytest -v tests/runtime --gripper Robotiq_Hand_E -k "test_gripper_closes and not fixed") \
  >/tmp/isaac-newton-warmup.log 2>&1 || true
tail -n 40 /tmp/isaac-newton-warmup.log
warp_after=$(warp_count)
[ "$warp_after" -gt "$warp_before" ] ||
  { echo "the warm-up did not add warp kernels to /opt/isaac-home/.cache/warp ($warp_before -> $warp_after)" >&2; exit 1; }
grep -Eq 'Newton.*PASSED|PASSED.*Newton' /tmp/isaac-newton-warmup.log ||
  { echo "no Newton test case passed in the warm-up" >&2; exit 1; }
sudo chmod -R a+rwX /opt/isaac-home "$ISAAC_DIR"

sudo chmod -R a+rX /opt/repo
# The run's user is not the owner of /opt/repo.
sudo git config --system --add safe.directory /opt/repo
git -C /opt/repo rev-parse HEAD
