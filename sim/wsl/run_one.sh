#!/bin/bash
# run_one.sh -- one ArduPilot SITL + bridge flight.
# Params arrive via WSLENV (set from the Windows caller):
#   SCEN (step|sine|flip), DUR (seconds), SEED, NOISE
# Usage from Windows:
#   SCEN=step DUR=15 SEED=0 WSLENV=SCEN/u:DUR/u:SEED/u:NOISE/u \
#     wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/<repo>/sim/wsl/run_one.sh
set -u
SCEN="${SCEN:-step}"; DUR="${DUR:-15}"; SEED="${SEED:-0}"; NOISE="${NOISE:-0.1}"
REPO=/root/drones

echo "[run_one] scenario=$SCEN duration=$DUR seed=$SEED"
pkill -f arducopter 2>/dev/null; pkill -f sim_vehicle 2>/dev/null; sleep 2

cd /root/ardupilot
rm -f /root/sitl.log
nohup timeout 400 Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
    --no-mavproxy -w > /root/sitl.log 2>&1 &
echo "[run_one] SITL booting (25 s for EKF/GPS)..."
sleep 25
if ! pgrep -f arducopter >/dev/null; then
  echo "[run_one] SITL FAILED TO START"; tail -5 /root/sitl.log; exit 1
fi
echo "[run_one] SITL up: $(ss -ltn | grep -c 5760) port-5760 listener(s)"

cd "$REPO"
timeout 200 python3 -m sim.ardupilot.bridge_node \
    --scenario "$SCEN" --duration "$DUR" --seed "$SEED" --noise "$NOISE" \
    --connection tcp:127.0.0.1:5760
RC=$?

pkill -f arducopter 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
echo "[run_one] bridge exit=$RC; artifacts:"
ls -la "$REPO/results/sim_ardupilot/" 2>/dev/null | tail -4
exit $RC
