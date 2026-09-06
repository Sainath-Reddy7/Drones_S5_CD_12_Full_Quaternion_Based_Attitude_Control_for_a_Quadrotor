"""ArduPilot SITL bridge: the paper's controller flying real flight-stack
firmware over MAVLink, exactly as a companion computer would fly hardware.

Why outer-loop (and that this was predicted):

ArduPilot does not accept raw body torques over MAVLink -- its own rate loop
owns torque generation (that is the point of running production firmware).
The deployment therefore sends SET_ATTITUDE_TARGET setpoints at 50 Hz:

    q_cmd    = conj(q_ref_paper)        # ArduPilot's quaternion is body->world,
                                        # the paper's is its conjugate (bridge.py)
    omega_des = -(Pq/Pw) * axis_err     # rate-setpoint equivalence of eq. (21):
                                        # the omega that zeroes the law's torque
    thrust   = AltitudeHold(Tc) / thrust_max_total

and logs the paper law's own eq. (21) torque alongside for comparison. FRP.md
section 7 anticipated exactly this: on ArduPilot the controller behaves as an
outer-loop attitude reference generator, exposing what a hardware port needs.
The fast-pole/12.3 kHz bound does not bind here because ArduPilot's inner rate
loop (>= 400 Hz on real rate targets, faster in SITL) closes the torque loop.

Targets ArduCopter SITL (`sim_vehicle.py -v ArduCopter -f quad`) and the
ardupilot_gazebo backend for the Gazebo item (sim/gazebo/README.md). Runtime
validation requires the WSL2/Linux runbook; the state conventions it relies on
are guarded machine-independently by tests/test_sim_bridge.py.

Usage (SITL must already be running):
    python -m sim.ardupilot.bridge_node --scenario step --duration 15
    python -m sim.ardupilot.bridge_node --scenario flip --connection udp:127.0.0.1:14551
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from quat_sitl import quaternion as quat
from quat_sitl.controller import ControllerGains, NonlinearP2Controller
from quat_sitl.dynamics import torque_saturate
from sim.common import AltitudeHold, PAPER_VEHICLE, RunLog, summarize
from sim.common.plots import plot_run
from sim.common.scenarios import default_duration, reference_quat, shortest_path

CONTROL_HZ = 50.0
Z_REF = 2.0  # m, scenario hover altitude (takeoff target)
TAKEOFF_WAIT_S = 8.0
VEHICLE_MASS = 0.2  # kg, pysitl-derived paper vehicle

GAINS = ControllerGains()  # paper values, Pq=20, Pw=4


def _q_conj_scalar_first(q: tuple) -> np.ndarray:
    q = np.array(q, dtype=np.float64)
    return q * np.array([1.0, -1.0, -1.0, -1.0])


class ArduBridge:
    def __init__(self, connection: str, scenario: str, seed: int, noise: float, out_root: Path):
        from pymavlink import mavutil

        self.mavutil = mavutil
        self.conn_str = connection
        self.scenario = scenario
        self.rng = np.random.default_rng(seed)
        self.noise = noise

        self.log = RunLog(simulator="ardupilot", scenario=scenario)
        self.out_dir = out_root or (Path.cwd() / "results" / "sim_ardupilot")

        # measured state (paper convention) from the flight stack
        self.q_meas_paper = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega_meas = np.zeros(3)  # FRD body rates
        self.omega_meas_noisy = np.zeros(3)
        self.z_up = 0.0
        self.vz_up = 0.0
        self.pos_ned = np.zeros(3)

    # -- connection / mode management -------------------------------------
    def connect(self):
        self.master = self.mavutil.mavlink_connection(self.conn_str)
        self.master.wait_heartbeat()
        print(f"[bridge] heartbeat: sysid {self.master.target_system} "
              f"compid {self.master.target_component}")

        # high-rate attitude + position streams for the control loop
        for msg_id, hz in ((self.mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100),
                           (self.mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                self.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0.0, 0.0, 0.0, 0.0, 0.0,
            )

    def _wait_mode(self, name: str, timeout: float = 20.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            if m is None:
                continue
            if self.mavutil.mode_string_by_dialect(m.custom_mode) == name:
                print(f"[bridge] mode {name} confirmed")
                return
        raise TimeoutError(f"mode {name} not reached within {timeout}s")

    def arm_and_takeoff(self) -> None:
        mav, mu = self.master, self.mavutil
        mav.set_mode("GUIDED")
        self._wait_mode("GUIDED")
        mav.arducopter_arm()
        print("[bridge] armed")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mu.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, Z_REF,
        )
        print(f"[bridge] takeoff to {Z_REF} m")
        time.sleep(TAKEOFF_WAIT_S)

    def disarm_and_land(self) -> None:
        self.master.set_mode("LAND")
        print("[bridge] LAND")

    # -- state pump ---------------------------------------------------------
    def pump_state(self) -> None:
        while True:
            m = self.master.recv_match(
                type=["ATTITUDE_QUATERNION", "LOCAL_POSITION_NED"], blocking=False
            )
            if m is None:
                return
            if m.get_type() == "ATTITUDE_QUATERNION":
                q_ap = np.array([m.q1, m.q2, m.q3, m.q4])  # FRD->NED, scalar-first
                self.omega_meas = np.array([m.rollspeed, m.pitchspeed, m.yawspeed])
                noisy = q_ap + self.rng.uniform(-self.noise, self.noise, 4)
                self.q_meas_paper = quat.conj(quat.normalize(noisy))  # paper convention
                noisy_w = self.omega_meas + self.rng.uniform(-self.noise, self.noise, 3)
                self.omega_meas_noisy = noisy_w
            else:  # LOCAL_POSITION_NED
                self.pos_ned = np.array([m.x, m.y, m.z])
                self.z_up, self.vz_up = -m.z, -m.vz

    # -- main loop ------------------------------------------------------------
    def run(self, duration: float) -> dict:
        altitude = AltitudeHold()
        law = NonlinearP2Controller(shortest_path=shortest_path(self.scenario))

        self.arm_and_takeoff()
        print(f"[bridge] scenario {self.scenario} starting")

        period = 1.0 / CONTROL_HZ
        t0 = time.time()
        next_tick = t0
        while True:
            now = time.time()
            t = now - t0
            if t >= duration:
                break
            next_tick += period

            self.pump_state()

            q_ref = reference_quat(self.scenario, t)  # paper convention

            # eq. (20) error for the rate-setpoint equivalence
            q_err = quat.mul(q_ref, self.q_meas_paper)
            axis_err = np.array(q_err[1:4])
            if shortest_path(self.scenario) and q_err[0] < 0.0:
                axis_err = -axis_err
            omega_des = -(GAINS.Pq / GAINS.Pw) * axis_err

            tau_log, sat = torque_saturate(
                law.compute_torque(q_ref, self.q_meas_paper, self.omega_meas_noisy), 4.0
            )

            # tilt cosine + collective thrust (FRD/NED: level-up => R[2,2]=1)
            R_bw = quat.to_dcm(quat.conj(self.q_meas_paper))
            tilt_cos = float(np.clip(R_bw[2, 2], -1.0, 1.0))
            thrust_n = altitude.thrust(
                Z_REF, self.z_up, self.vz_up, tilt_cos, VEHICLE_MASS,
                PAPER_VEHICLE.thrust_max_total, period,
            )
            thrust_norm = float(
                np.clip(thrust_n / PAPER_VEHICLE.thrust_max_total, 0.0, 1.0)
            )

            q_cmd_ap = quat.conj(q_ref)  # back to ArduPilot's body->world convention
            self.master.mav.set_attitude_target_send(
                int(t * 1000),
                self.master.target_system, self.master.target_component,
                0,  # use quaternion, rates, and thrust
                q_cmd_ap.tolist(),
                float(omega_des[0]), float(omega_des[1]), float(omega_des[2]),
                thrust_norm,
            )

            self.log.add(
                t, q_ref, self.q_meas_paper, self.omega_meas_noisy, tau_log, sat.astype(float),
                thrust_n, 1.0, np.array([self.pos_ned[0], self.pos_ned[1], self.z_up]),
                _error_angle(q_ref, self.q_meas_paper),
            )

            sleep = next_tick - time.time()
            if sleep > 0:
                time.sleep(sleep)

        self.disarm_and_land()

        stamp = f"{self.scenario}_seed{self.rng.integers(0, 1 << 30)}"
        self.log.write(self.out_dir / f"{stamp}.csv")
        plot_run(self.log, self.out_dir)
        return summarize(self.log)


def _error_angle(q_ref, q_m) -> float:
    """Geodesic attitude error [rad], same metric as QuatBridge.attitude_error_angle."""
    dot = abs(float(np.dot(q_ref, q_m)))
    return 2.0 * float(np.arccos(np.clip(dot, -1.0, 1.0)))


def run(connection: str, scenario: str, duration: float | None, seed: int,
        noise: float, out_root: Path | None = None) -> dict:
    bridge = ArduBridge(connection, scenario, seed, noise, out_root or Path.cwd() / "results")
    return bridge.run(duration if duration is not None else default_duration(scenario))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connection", default="tcp:127.0.0.1:5760",
                    help="MAVLink endpoint of the running SITL")
    ap.add_argument("--scenario", choices=["step", "sine", "flip"], default="step")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.1)
    args = ap.parse_args()
    print(run(args.connection, args.scenario, args.duration, args.seed, args.noise))
