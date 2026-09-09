"""Mock ArduPilot SITL endpoint for protocol-level integration testing.

WHAT THIS IS: a MAVLink-speaking stand-in for ArduPilot SITL that implements
exactly the protocol subset sim/ardupilot/bridge_node.py uses -- heartbeats
with copter custom_modes (GUIDED=15, LAND=9), COMMAND_ACKs, arming/takeoff,
ATTITUDE_QUATERNION + LOCAL_POSITION_NED telemetry, and SET_ATTITUDE_TARGET
handling with a simple 6-DOF response model.

WHAT THIS IS NOT: ArduPilot firmware. It validates the bridge's wire
behavior (connection, mode transitions, message rates, setpoint math,
logging, plots) machine-independently. The firmware run itself happens under
WSL2/Linux via sim/gazebo/README.md and sim/ardupilot/ -- this harness
de-risks it, it does not replace it.

Usage (standalone):
    python -m sim.ardupilot.mock_sitl --listen tcpin:127.0.0.1:5760
    # then, in another terminal:
    python -m sim.ardupilot.bridge_node --scenario step --connection tcp:127.0.0.1:5760
"""
from __future__ import annotations

import argparse
import threading
import time

import numpy as np

from quat_sitl import quaternion as quat

# ArduCopter custom_mode numbers (ardupilotmega dialect)
MODE_GUIDED = 15
MODE_LAND = 9
MAV_MODE_FLAG_CUSTOM_BIT = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

G = 9.81
HOVER_THRUST_NORM = 0.25  # bridge sends thrust/(4*Tmax): hover ~ m*g/(4*Tmax)


class MockQuad:
    """Point-mass + kinematic-attitude response model. Good enough to
    exercise the bridge's math; not a flight model."""

    def __init__(self) -> None:
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # FRD->NED, scalar-first
        self.omega = np.zeros(3)  # FRD body rates
        self.pos_ned = np.zeros(3)
        self.vel_ned = np.zeros(3)
        self.thrust = HOVER_THRUST_NORM
        self.q_cmd = np.array([1.0, 0.0, 0.0, 0.0])
        self.taking_off = False

    def step(self, q_cmd: np.ndarray, thrust_norm: float, dt: float) -> None:
        # attitude: first-order tracking of the commanded quaternion via the
        # error axis (rate proportional to error angle, capped at 3 rad/s)
        q_err = quat.mul(quat.normalize(q_cmd), quat.conj(quat.normalize(self.q)))
        axis = np.array(q_err[1:4])
        angle = 2.0 * float(np.arccos(np.clip(abs(q_err[0]), 0.0, 1.0)))
        if q_err[0] < 0.0:
            axis, angle = -axis, 2.0 * float(np.arccos(np.clip(-q_err[0], 0.0, 1.0)))
        n = float(np.linalg.norm(axis))
        rate_cmd = np.zeros(3) if n < 1e-6 or angle < 1e-3 else (
            axis / n * min(6.0 * angle, 6.0)
        )
        self.omega += (rate_cmd - self.omega) * min(1.0, dt / 0.05)

        zero_w = np.concatenate([[0.0], self.omega])
        self.q = quat.normalize(self.q + dt * 0.5 * quat.mul(self.q, zero_w))

        # altitude: thrust_norm 0..1 with hover at ~0.25; takeoff ramp
        if self.taking_off:
            z_target = 2.0  # matches the bridge's takeoff altitude
            self.pos_ned[2] += (min(z_target, self.pos_ned[2] + dt) - self.pos_ned[2])
            self.vel_ned[2] = 0.0
        else:
            accel_up = (thrust_norm / HOVER_THRUST_NORM - 1.0) * G
            self.vel_ned[2] -= accel_up * dt
            self.pos_ned[2] += self.vel_ned[2] * dt
            self.pos_ned[2] = min(self.pos_ned[2], 0.0)  # ground at z_ned=0


def serve(connection: str, duration_s: float = 120.0, verbose: bool = True) -> None:
    from pymavlink import mavutil

    mav = mavutil.mavlink_connection(connection, dialect="ardupilotmega")
    vehicle = MockQuad()
    mode = 4  # LOITER-ish default; becomes GUIDED on command
    armed = False
    t0 = time.time()
    last_telem = 0.0
    last_hb = 0.0
    sysid, compid = 1, 1

    if verbose:
        print(f"[mock-sitl] listening on {connection}, waiting for GCS...")

    while time.time() - t0 < duration_s:
        now = time.time() - t0

        # ---- receive (non-blocking drain) --------------------------------
        msg = mav.recv_match(blocking=False)
        while msg is not None:
            mtype = msg.get_type()
            if mtype == "COMMAND_LONG":
                cmd = msg.command
                # SET_MESSAGE_INTERVAL (511): stream rates -- accepted
                if cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    mode = int(msg.param2)
                elif cmd == getattr(mavutil.mavlink, "MAV_CMD_COMPONENT_ARM", 400):  # legacy arm cmd id
                    armed = bool(msg.param1 > 0)
                elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    vehicle.taking_off = True
                elif verbose:
                    print(f"[mock-sitl] t={now:5.1f}s COMMAND {cmd} accepted")
                mav.mav.command_ack_send(
                    cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED
                )
            elif mtype == "SET_ATTITUDE_TARGET":
                vehicle.q_cmd = np.array(msg.q, dtype=np.float64)
                vehicle.thrust = float(msg.thrust)
            msg = mav.recv_match(blocking=False)

        # ---- telemetry at 50 Hz ------------------------------------------
        if now - last_telem >= 0.02:
            dt = now - last_telem if last_telem else 0.02
            vehicle.step(vehicle.q_cmd, vehicle.thrust, dt)
            last_telem = now
            mav.mav.attitude_quaternion_send(
                int(now * 1e6), vehicle.q[0], vehicle.q[1], vehicle.q[2], vehicle.q[3],
                vehicle.omega[0], vehicle.omega[1], vehicle.omega[2],
            )
            mav.mav.local_position_ned_send(
                int(now * 1e3),
                vehicle.pos_ned[0], vehicle.pos_ned[1], vehicle.pos_ned[2],
                vehicle.vel_ned[0], vehicle.vel_ned[1], vehicle.vel_ned[2],
            )

        # ---- heartbeat at 1 Hz --------------------------------------------
        if now - last_hb >= 1.0:
            last_hb = now
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                (MAV_MODE_FLAG_CUSTOM_BIT | (MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)),
                mode,  # custom_mode -- what mode_string_by_number() reads
                mavutil.mavlink.MAV_STATE_ACTIVE,
                3,
            )
        time.sleep(0.005)

    if verbose:
        print("[mock-sitl] duration elapsed, closing")


class MockSitlServer(threading.Thread):
    """Thread wrapper for in-process use by tests."""

    def __init__(self, connection: str = "tcpin:127.0.0.1:5770", duration_s: float = 90.0):
        super().__init__(daemon=True)
        self.connection = connection
        self.duration_s = duration_s

    def run(self) -> None:
        serve(self.connection, self.duration_s, verbose=False)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--listen", default="tcpin:127.0.0.1:5760")
    ap.add_argument("--duration", type=float, default=120.0)
    args = ap.parse_args()
    serve(args.listen, args.duration)
