#!/usr/bin/env bash
# Reproduce the native cross-simulator benchmark (FRP.md section 3):
# all three paper scenarios on every natively-runnable stack, then the
# comparison table. Gazebo/ArduPilot run separately under WSL2 (see
# sim/gazebo/README.md).
#
# Usage:  bash sim/run_all.sh [--seeds "0 1 2"] [--scenarios "step sine flip"]
#
# Python selection, first match wins per stack:
#   MuJoCo:            $PY_MUJOCO, .venv310/Scripts/python.exe, python3
#   gym-pybullet:      $PY_GPD,    mmenv/python.exe,            python3
# (On the team Windows box: .venv310 has mujoco; mmenv — micromamba py312 +
#  conda-forge pybullet — has gym-pybullet-drones. See sim/README.md.)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

SEEDS="0"
SCENARIOS="step sine flip"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seeds) SEEDS="$2"; shift 2 ;;
    --scenarios) SCENARIOS="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

pick_py() { # pick_py <varname> <import-to-test> <extra-candidates...>
  local var="$1" mod="$2"; shift 2
  local cand
  for cand in "$@"; do
    if "$cand" -c "import $mod" >/dev/null 2>&1; then
      printf -v "$var" '%s' "$cand"
      return 0
    fi
  done
  echo "[run_all] no python found with '$mod' -- stack skipped" >&2
  printf -v "$var" '%s' ""
}

pick_py PY_MUJOCO mujoco "${PY_MUJOCO:-}" .venv310/Scripts/python.exe .venv310/bin/python python3
pick_py PY_GPD gym_pybullet_drones "${PY_GPD:-}" mmenv/python.exe python3

for s in $SCENARIOS; do
  for seed in $SEEDS; do
    [[ -n "$PY_MUJOCO" ]] && echo "[run_all] mujoco $s seed$seed" && \
      "$PY_MUJOCO" -m sim.mujoco.run --scenario "$s" --seed "$seed" --noise 0.1
    [[ -n "$PY_GPD" ]] && echo "[run_all] gym_pybullet $s seed$seed" && \
      "$PY_GPD" -m sim.gym_pybullet.run --scenario "$s" --seed "$seed" --noise 0.1
  done
done

echo "[run_all] comparison table:"
exec "$PY_MUJOCO" -m sim.compare
