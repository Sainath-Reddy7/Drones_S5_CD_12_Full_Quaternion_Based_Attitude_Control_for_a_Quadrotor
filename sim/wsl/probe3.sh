#!/bin/bash
# probe3.sh -- direct binary: gz + arducopter(JSON). Diagnoses: 9002 FDM link,
# raw UDP byte flow on 14550, and MAVLink parse.
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds
OUT=/root/probe3.txt; : > "$OUT"
pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; pkill -f sim_vehicle 2>/dev/null; sleep 2

# ORDER MATTERS: SITL must listen on 9002 BEFORE gz's plugin starts
# (the plugin gives up after connectionTimeoutMaxCount retries)
cd /root/ardupilot
(nohup timeout 100 ./build/sitl/bin/arducopter --model JSON --speedup 1 \
    --serial0=tcp:5760 -I0 > /root/ap_direct.log 2>&1 &)
sleep 8
(timeout 90 gz sim -s -r -v4 /root/ardupilot_gazebo/worlds/iris_runway.sdf > /root/gz.log 2>&1 &)
sleep 15

{
  echo "--- port 9002 (FDM):"; ss -tn | grep -E "9002" || echo "  NO 9002 connection"
  echo "--- gz ardupilot lines:"; grep -iE "ArduPilotPlugin" /root/gz.log | tail -4
echo "--- sitl log tail:"; tail -5 /root/ap_direct.log
  echo "--- raw byte + mavlink probe:"
  timeout 20 python3 - <<'PYEOF' 2>&1
import socket, time
from pymavlink import mavutil

# raw byte count first
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 14551))
s.settimeout(0.5)
# SITL sends to 14550; probe both by asking it to resend: connect mavlink below

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
t0 = time.time()
hb = None
while time.time() - t0 < 8:
    msg = m.recv_match(blocking=False)
    if msg and msg.get_type() == "HEARTBEAT":
        hb = msg
        break
    time.sleep(0.05)
if hb:
    print("HEARTBEAT sysid", hb.get_srcSystem(), "compid", hb.get_srcComponent(),
          "type", hb.type, "autopilot", hb.autopilot)
else:
    print("NO HEARTBEAT in 8s")
n, t0 = 0, time.time()
while time.time() - t0 < 3:
    msg = m.recv_match(blocking=False)
    if msg:
        n += 1
        if n <= 3:
            print("msg:", msg.get_type())
    time.sleep(0.01)
print("messages in 3s:", n)
PYEOF
} >> "$OUT" 2>&1

pkill -f arducopter 2>/dev/null; pkill -f 'gz sim' 2>/dev/null
cat "$OUT"
