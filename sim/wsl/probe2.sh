#!/bin/bash
# probe2.sh -- MAVLink reachability of SITL under gz, results to /root/probe.txt
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds:/root/drones/sim/gazebo/worlds
OUT=/root/probe.txt
: > "$OUT"
pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null; sleep 2

(timeout 120 gz sim -s -r -v3 /root/drones/sim/gazebo/worlds/paper_attitude.world > /root/gz.log 2>&1 &)
sleep 10
cd /root/ardupilot
(timeout 110 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy -w > /root/sitl_gz.log 2>&1 &)
sleep 30

{
  echo "listeners:"; ss -ltn | grep -E '576|9002'
  echo "--- mavlink probe (8 s):"
  timeout 14 python3 - <<'PYEOF' 2>&1
from pymavlink import mavutil
import time
try:
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
    m.wait_heartbeat(timeout=8)
    print("HEARTBEAT OK sysid", m.target_system)
    n = 0
    t0 = time.time()
    while time.time() - t0 < 3:
        msg = m.recv_match(blocking=False)
        if msg:
            n += 1
        time.sleep(0.01)
    print("messages in 3 s:", n)
except Exception as e:
    print("PROBE FAILED:", type(e).__name__, e)
PYEOF
} >> "$OUT" 2>&1

pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
cat "$OUT"
