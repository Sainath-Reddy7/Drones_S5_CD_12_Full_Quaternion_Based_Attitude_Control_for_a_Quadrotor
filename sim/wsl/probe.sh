#!/bin/bash
# probe.sh -- diagnose the gz + SITL(JSON) + MAVLink chain
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds:/root/drones/sim/gazebo/worlds
pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null; sleep 2

(timeout 90 gz sim -s -r -v3 /root/drones/sim/gazebo/worlds/paper_attitude.world > /root/gz.log 2>&1 &)
sleep 10
echo "[probe] gz up: $(pgrep -fc 'gz sim')"

cd /root/ardupilot
(timeout 80 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy -w > /root/sitl_gz.log 2>&1 &)
sleep 25
echo "[probe] sitl up: $(pgrep -fc arducopter)"
echo "[probe] listeners:"; ss -ltn | grep -E '576|9002' || echo "  none on 576x/9002"
echo "[probe] RiTW line: $(grep RiTW /root/sitl_gz.log | tail -1)"
echo "[probe] fdm/json mentions:"; grep -iE 'json|fdm' /root/sitl_gz.log | tail -3
grep -iE 'json|fdm|connect' /root/gz.log | tail -3

timeout 12 python3 - <<'PYEOF'
from pymavlink import mavutil
try:
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
    m.wait_heartbeat(timeout=6)
    print("[probe] HEARTBEAT OK sysid", m.target_system)
    for _ in range(3):
        msg = m.recv_match(blocking=True, timeout=2)
        print("[probe] got:", msg.get_type() if msg else None)
except Exception as e:
    print("[probe] FAILED:", e)
PYEOF

pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null
echo "[probe] done"
