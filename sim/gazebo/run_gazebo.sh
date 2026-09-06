#!/usr/bin/env bash
# One-shot Gazebo run of one paper scenario (see README.md for prerequisites).
# Usage:  bash sim/gazebo/run_gazebo.sh [step|sine|flip] [connection]
set -euo pipefail
SCENARIO="${1:-step}"
CONNECTION="${2:-tcp:127.0.0.1:5760}"

: "${GZ_SIM_SYSTEM_PLUGIN_PATH:?export GZ_SIM_SYSTEM_PLUGIN_PATH (see sim/gazebo/README.md)}"
: "${GZ_SIM_RESOURCE_PATH:?export GZ_SIM_RESOURCE_PATH (see sim/gazebo/README.md)}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

echo "[gazebo] launching world (server-only, headless-safe)"
gz sim -s -r -v4 sim/gazebo/worlds/paper_attitude.world &
GZ_PID=$!
sleep 5

echo "[gazebo] launching ArduPilot SITL (gazebo backend)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
( cd "$ARDUPILOT_DIR" && Tools/autotest/sim_vehicle.py -v ArduCopter \
    -f gazebo-iris --no-mavproxy -L home --model JSON ) &
AP_PID=$!
sleep 20

echo "[gazebo] flying scenario: $SCENARIO"
python3 -m sim.ardupilot.bridge_node --scenario "$SCENARIO" --connection "$CONNECTION"

echo "[gazebo] done -- landing/shutting down"
kill $AP_PID $GZ_PID 2>/dev/null || true
mkdir -p results/sim_gazebo && mv results/sim_ardupilot/*.csv results/sim_ardupilot/*.png results/sim_gazebo/ 2>/dev/null || true
echo "[gazebo] results in results/sim_gazebo/"
