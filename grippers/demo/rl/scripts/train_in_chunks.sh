#!/usr/bin/env bash
# Train the tactile pick in chunks, resuming each from the last checkpoint, and after every chunk export the policy,
# evaluate it on 64 envs and record a single-env video. Training holds the GPU lock the whole time, so this is how
# to look at the policy while a long run goes on.
#
#   train_in_chunks.sh NAME CHUNK_ITERS CHUNKS [NUM_ENVS] [FIRST_CHECKPOINT]
#
# With PICK_ALL=1 each chunk is also scored by scripts/pick_all.py (the policy empties a box, 16 envs): the number of
# spheres picked and left, in <run>/pick_all.txt.
#
# Runs and results: $ISAACLAB/logs/rsl_rl/tactile_sphere_pick/<date>_NAME_c<k>/{eval.txt,eval_video/}
# Environment:
#   ISAACLAB    the Isaac Lab checkout (default ~/robotiq/devel/IsaacLab: an example, set your own)
#   ISAAC_LOCK  optional wrapper that queues GPU jobs on a shared machine, called as `$ISAAC_LOCK run --wait --label X -- cmd`;
#               when it is unset or not executable the commands run directly (our internal tool is not part of this repo)
#   TRAIN_LOGS  where the per-chunk training logs go (default ~/.isaaclab/logs)
set -u
NAME=${1:?run name}; CHUNK=${2:?iterations per chunk}; CHUNKS=${3:?number of chunks}; ENVS=${4:-2048}; CKPT=${5:-}
ISAACLAB=${ISAACLAB:-$HOME/robotiq/devel/IsaacLab}
LOCK=${ISAAC_LOCK:-}
LOGDIR=${TRAIN_LOGS:-$HOME/.isaaclab/logs}
mkdir -p "$LOGDIR" || exit 1
# locked LABEL CMD...: CMD through the lock when there is one
locked() { local label=$1; shift; if [ -x "$LOCK" ]; then "$LOCK" run --wait --label "$label" -- "$@"; else "$@"; fi; }
HERE=$(cd "$(dirname "$0")" && pwd)
TASK=Robotiq-Tactile-Sphere-Pick-UR5e-2F85
export OMNI_KIT_ACCEPT_EULA=YES
cd "$ISAACLAB" || exit 1
PY=.venv/bin/python
LOGS=logs/rsl_rl/tactile_sphere_pick

for k in $(seq 1 "$CHUNKS"); do
  args=(--rl_library rsl_rl --task $TASK --num_envs "$ENVS" --max_iterations "$CHUNK" --run_name "${NAME}_c$k")
  [ -n "$CKPT" ] && args+=(--checkpoint "$CKPT")
  # TRAIN_ARGS: extra train.py / hydra overrides, e.g. "agent.algorithm.entropy_coef=0.0"
  # shellcheck disable=SC2206
  args+=(${TRAIN_ARGS:-})
  # a run directory of this name from an earlier run would be picked up below as this chunk's result
  [ -n "$(ls -d $LOGS/*_"${NAME}_c$k" 2>/dev/null)" ] && { echo "run name ${NAME}_c$k already exists under $LOGS, use another NAME"; exit 1; }
  echo "[$(date +%T)] chunk $k: training ${args[*]}"
  locked "tactile-$NAME-train-c$k" $PY scripts/reinforcement_learning/train.py "${args[@]}" > "$LOGDIR/tp-$NAME-c$k.log" 2>&1
  run=$(ls -td $LOGS/*_"${NAME}_c$k" 2>/dev/null | head -1)
  [ -z "$run" ] && { echo "chunk $k: no run directory, stopping"; exit 1; }
  CKPT=$(ls -v "$run"/model_*.pt 2>/dev/null | tail -1)
  [ -z "$CKPT" ] && { echo "chunk $k: training left no checkpoint (see the log), stopping"; exit 1; }
  echo "[$(date +%T)] chunk $k done: $CKPT"
  grep -E "Mean reward|Mean episode length|Episode_Reward/(enter|coverage|touch|grasp|stable|lift|success)" "$LOGDIR/tp-$NAME-c$k.log" | tail -10 | sed 's/^ *//' | tee "$run/train_tail.txt"
  # export the policy (play.py writes it at start, then loops: stop it after the export)
  locked "tactile-$NAME-export-c$k" timeout 200 $PY scripts/reinforcement_learning/play.py --rl_library rsl_rl --task $TASK --num_envs 1 --checkpoint "$CKPT" > /dev/null 2>&1
  [ -f "$run/exported/policy.pt" ] || { echo "chunk $k: policy export failed"; continue; }
  locked "tactile-$NAME-eval-c$k" $PY "$HERE/eval_tactile.py" --policy "$run/exported/policy.pt" --num_envs 64 > "$run/eval_full.txt" 2>&1
  grep "^RESULT" "$run/eval_full.txt" | tee "$run/eval.txt"
  if [ "${PICK_ALL:-0}" = 1 ]; then
    locked "tactile-$NAME-pickall-c$k" $PY "$HERE/pick_all.py" --policy "$run/exported/policy.pt" --num_envs 16 --steps 6000 --layout reachable > "$run/pick_all_full.txt" 2>&1
    grep -E "^RESULT" "$run/pick_all_full.txt" | tee "$run/pick_all.txt"
  fi
  locked "tactile-$NAME-video-c$k" $PY "$HERE/eval_tactile.py" --policy "$run/exported/policy.pt" --num_envs 1 --video "$run/eval_video" > "$run/eval_video.txt" 2>&1
done
echo "[$(date +%T)] all chunks done"
