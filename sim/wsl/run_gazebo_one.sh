#!/bin/bash
# run_gazebo_one.sh -- one GAZEBO-physics flight: gz-sim (iris world) +
# ArduPilot SITL (gazebo-iris frame, JSON interface) + our frozen bridge.
# Params via WSLENV: SCEN, DUR, SEED, NOISE
set -u
SCEN="${SCEN:-step}"; DUR="${DUR:-15}"; SEED="${SEED:-0}"; NOISE="${NOISE:-0.1}"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds
REPO=/root/drones

echo "[gz-run] scenario=$SCEN duration=$DUR"
pkill -f arducopter 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
pkill -f 'gz sim' 2>/dev/null; pkill -f ruby 2>/dev/null; sleep 2

cd /root/ardupilot_gazebo
nohup timeout 400 gz sim -s -r -v3 worlds/iris.sdf > /root/gz.log 2>&1 &
echo "[gz-run] gazebo starting..."
sleep 12
if ! pgrep -f 'gz sim' >/dev/null; then
  echo "[gz-run] GAZEBO FAILED TO START"; tail -6 /root/gz.log; exit 1
fi
echo "[gz-run] gazebo up"

cd /root/ardupilot
nohup timeout 400 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
    --no-mavproxy -w > /root/sitl_gz.log 2>&1 &
echo "[gz-run] SITL (gazebo-iris) booting..."
sleep 30
if ! pgrep -f arducopter >/dev/null; then
  echo "[gz-run] SITL FAILED"; tail -6 /root/sitl_gz.log; exit 1
fi
ss -ltn | grep 576 || echo "[gz-run] WARNING: no 576x listener yet"

cd "$REPO"
timeout 200 python3 -m sim.ardupilot.bridge_node \
    --scenario "$SCEN" --duration "$DUR" --seed "$SEED" --noise "$NOISE" \
    --connection tcp:127.0.0.1:5760
RC=$?

pkill -f arducopter 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
pkill -f 'gz sim' 2>/dev/null; pkill -f ruby 2>/dev/null
echo "[gz-run] bridge exit=$RC"
ls -la "$REPO/results/" | grep -E "$SCEN.*seed" | tail -2
exit $RC
