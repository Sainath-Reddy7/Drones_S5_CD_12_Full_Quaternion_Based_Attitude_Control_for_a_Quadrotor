#!/bin/bash
# diag.sh -- what state is the gz-driven vehicle actually in?
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds
pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f gz-sim 2>/dev/null; sleep 3

cd /root/ardupilot
rm -f /root/ap_direct.log
(nohup timeout 240 ./build/sitl/bin/arducopter --model JSON --speedup 1 \
    --serial0=tcp:5760 -I0 > /root/ap_direct.log 2>&1 &)
sleep 8
(nohup timeout 220 gz sim -s -r -v4 /root/ardupilot_gazebo/worlds/iris_runway.sdf > /root/gz.log 2>&1 &)
echo "[diag] stack starting..."
sleep 30

timeout 100 python3 - <<'PYEOF' 2>&1
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=10)
print("[diag] heartbeat sysid", m.target_system)

for mid, hz in ((0, 10), (24, 10), (30, 20), (32, 10), (25, 10)):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6/hz, 0,0,0,0,0)

# set arming check off like the bridge does
try:
    m.param_set_send("ARMING_CHECK", 0, m.target_system, m.target_component)
except TypeError:
    m.param_set_send("ARMING_CHECK", 0)

gps_fix = ekf = None
t0 = time.time()
armed_seen = False
next_arm = 0
while time.time() - t0 < 45:
    msg = m.recv_match(blocking=False)
    if msg is None:
        time.sleep(0.02)
        continue
    t = msg.get_type()
    if t == "GPS_RAW_INT":
        gps_fix = msg.fix_type
    elif t == "EKF_STATUS_REPORT":
        ekf = (msg.flags, msg.velocity_variance, msg.pos_horiz_variance)
    elif t == "STATUSTEXT":
        print("[diag] STATUSTEXT:", msg.text)
    elif t == "HEARTBEAT":
        armed_seen = bool(msg.base_mode & 128)
    if time.time() - t0 > next_arm:
        m.arducopter_arm()
        next_arm += 10
    if armed_seen:
        print(f"[diag] ARMED at t={time.time()-t0:.0f}s GPSfix={gps_fix} EKF={ekf}")
        break

print(f"[diag] final: armed={armed_seen} GPSfix={gps_fix} EKFflags={ekf}")
import subprocess
print("[diag] sitl tail:", subprocess.run(
    ["tail", "-2", "/root/ap_direct.log"], capture_output=True, text=True).stdout.strip())
PYEOF

pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f gz-sim 2>/dev/null
