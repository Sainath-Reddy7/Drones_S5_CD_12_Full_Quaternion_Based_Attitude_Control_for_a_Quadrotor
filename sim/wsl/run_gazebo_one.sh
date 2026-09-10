#!/bin/bash
# run_gazebo_one.sh -- GAZEBO-physics flight of the frozen paper controller.
#
# Working recipe (established by probing -- see sim/wsl/probe*.sh history):
#   1. arducopter binary DIRECTLY (no sim_vehicle), --model JSON, serial0=tcp:5760
#   2. THEN gz sim (server-only) with ardupilot_gazebo's iris_runway world
#      -- startup order matters: SITL must listen on 9002 before the plugin
#      starts or the plugin exhausts its connection retries
#   3. bridge on tcp:127.0.0.1:5760 (its default)
#
# Params via WSLENV: SCEN, DUR, SEED, NOISE
set -u
SCEN="${SCEN:-step}"; DUR="${DUR:-15}"; SEED="${SEED:-0}"; NOISE="${NOISE:-0.1}"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds
REPO=/root/drones

echo "[gz-run] scenario=$SCEN duration=$DUR seed=$SEED"
pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
pkill -f gz-sim 2>/dev/null; pkill -f 'ruby.*gz' 2>/dev/null
sleep 3

cd /root/ardupilot
rm -f /root/ap_direct.log
(nohup timeout 400 ./build/sitl/bin/arducopter --model JSON --speedup 1 \
    --serial0=tcp:5760 -I0 > /root/ap_direct.log 2>&1 &)
echo "[gz-run] SITL(JSON) up, listening 9002/5760"
sleep 8

(nohup timeout 380 gz sim -s -r -v4 /root/ardupilot_gazebo/worlds/iris_runway.sdf \
    > /root/gz.log 2>&1 &)
echo "[gz-run] gazebo starting..."
sleep 20
if ! pgrep -f 'gz sim' >/dev/null; then
  echo "[gz-run] GAZEBO FAILED"; tail -5 /root/gz.log; pkill -f arducopter; exit 1
fi
if ! pgrep -f arducopter >/dev/null; then
  echo "[gz-run] SITL DIED"; tail -5 /root/ap_direct.log; pkill -f 'gz sim'; exit 1
fi
echo "[gz-run] stack up: FDM=$(ss -tn | grep -c 9002) 5760=$(ss -ltn | grep -c 5760)"

# the gz plugin can take tens of seconds before its first sensor frame; the
# EKF then needs ~15 s more. Gate on it or the bridge wastes its arm window.
echo "[gz-run] waiting for gz sensor stream..."
W=0
while [ $W -lt 75 ]; do
  if grep -q "JSON received" /root/ap_direct.log 2>/dev/null; then break; fi
  sleep 5; W=$((W+5))
done
if grep -q "JSON received" /root/ap_direct.log 2>/dev/null; then
  echo "[gz-run] sensors flowing after ~${W}s; giving EKF 25 s"
  sleep 25
else
  echo "[gz-run] WARNING: no sensor frames in 75 s"
fi

cd "$REPO"
timeout 250 python3 -m sim.ardupilot.bridge_node \
    --scenario "$SCEN" --duration "$DUR" --seed "$SEED" --noise "$NOISE" \
    --connection tcp:127.0.0.1:5760
RC=$?

pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
echo "[gz-run] bridge exit=$RC"
exit $RC
