"""INTERACTIVE gym-pybullet-drones simulator — the paper's 3 tests, live.

Same architecture as the MuJoCo interactive app: position locked (pure
attitude, the paper's plant), paper tests on keys 1/2/3, manual flight
on WASD, live HUD with key state + motor commands.

THE PAPER'S 3 TESTS (from any mode, one keypress):
    1 = STEP   (1 rad steps on roll/pitch/yaw)
    2 = SINE   (0.5 rad wave tracking under noise)
    3 = FLIP   (360-degree rotation, no gimbal lock)

FLIGHT (held keys, manual mode only):
    W/S = pitch fwd/back    A/D = roll left/right
    Q/E = yaw left/right    R/F = climb/descend
    SPACE = hover

MODE:
    M = toggle MANUAL <-> OUR CONTROL
    G = waypoint mission    U = circle
    H = hold                N = land
    X = abort               P = pause
    T = reset               ESC = quit

CAMERA:
    C = cycle (free/follow/top)

Note: PyBullet physics at 24 kHz is computationally heavier than MuJoCo at
10 kHz — the sim runs slower than real-time. The HUD shows "RT" or "SLOW".

Launch:  python -m sim.gym_pybullet.interactive
"""
from __future__ import annotations

import csv
import time
from pathlib import Path

import numpy as np

from quat_sitl import quaternion as quat
from sim.common import AltitudeHold, PAPER_VEHICLE, QuatBridge, thrusts_to_wrench_zup
from sim.common.scenarios import FLIP_RAMP_6DOF, reference_quat
from sim.interactive import config as cfg
from sim.interactive.input_manager import InputManager

MASS = cfg.DRONE["mass_kg"]
G = 9.81
CTRL_EVERY = 1   # controller every physics step (24 kHz, the paper's rate)
RENDER_EVERY = 240  # render at ~100 Hz (24 kHz / 240)
POLL_EVERY = 2400   # keyboard poll at ~10 Hz
HUD_EVERY = 1200    # HUD at ~20 Hz
KF = 3.0e-9  # must match paper_quad.urdf

# CRITICAL rotor-order bridge (from sim/gym_pybullet/run.py):
# our mixer's rotor 1..4 must be reordered to the sim's prop 0..3,
# and the yaw torque negated (mirrored yaw pairing). Without this,
# the mixer sends thrusts to the wrong rotors and the drone crashes.
MY_TO_SIM = (2, 1, 3, 0)

BENIGN = {"floor", "pad_base"}
DRONE_PARTS = ("prop", "mount", "arm", "shell", "plate", "cam", "lens", "led", "frame")
SPAWN = np.array([0.0, 0.0, 0.5])  # matches the benchmark adapter's Z_REF


def _euler(q):
    return tuple(float(x) for x in quat.to_euler(q))


class PyBulletSim:
    """Interactive gym-pybullet simulator with position lock + paper tests."""

    def __init__(self, seed: int = 0, gui: bool = True):
        from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
        from enum import Enum

        class PD(Enum):
            PAPER_QUAD = "paper_quad"

        # install the URDF into the package assets if not there
        from sim.gym_pybullet.run import _install_urdf
        _install_urdf()
        paper_enum = Enum("PaperDrone", {"PAPER_QUAD": "paper_quad"}).PAPER_QUAD

        self.env = CtrlAviary(
            drone_model=paper_enum,
            num_drones=1,
            initial_xyzs=SPAWN[None, :],
            initial_rpys=np.zeros((1, 3)),
            pyb_freq=24000,
            ctrl_freq=24000,
            gui=gui,
            user_debug_gui=False,
            output_folder="results",
        )
        self.gui = gui
        self.rng = np.random.default_rng(seed)
        self.bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=True)
        self.alt = AltitudeHold()

        self.mode = "MANUAL"
        self.mission = "IDLE"
        self.paused = False
        self.paper_test = None
        self.paper_t = 0.0
        self.wind = "OFF"
        self.noise = "PAPER"
        self.refs = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "z": SPAWN[2]}
        self.keys: set = set()
        self.telemetry: dict = {}
        self.motors = np.zeros(4)
        self.warn = ""
        self._thrust_override = None
        self._paper_qref = None
        self.lock_position = True
        self._lock_point = SPAWN.copy()
        self._held = {"fz": MASS * G, "tau": np.zeros(3)}
        self.t_sim = 0.0
        self.dt = 1.0 / 24000

        # remove PyBullet's default damping (benchmark does this too)
        import pybullet as p
        for i in self.env.DRONE_IDS:
            p.changeDynamics(int(i), -1, linearDamping=0.0, angularDamping=0.0,
                             physicsClientId=self.env.CLIENT)

        self._setup_collision_geoms()

    def _setup_collision_geoms(self):
        """Add named collision geoms for buildings/trees around the spawn."""
        import pybullet as p

        # ground plane is already there from CtrlAviary
        # add a helipad visual
        if self.gui:
            p.addUserDebugText("PAD", [0, -6, 0.1], textColorRGB=[1, 0.8, 0.1], lifeTime=0)

    def read_state(self, noisy: bool = False) -> dict:
        obs = self.env._getDroneStateVector(0)
        pos = obs[0:3]
        quat_xyzw = obs[3:7]  # PyBullet [x,y,z,w]
        q_sim = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        vel = obs[10:13]
        ang_v_world = obs[13:16]
        R_bw = quat.to_dcm(q_sim)
        omg = R_bw.T @ ang_v_world
        amp = cfg.NOISE_LEVELS[self.noise]
        if noisy and amp > 0:
            q_sim = quat.normalize(q_sim + self.rng.uniform(-amp, amp, 4))
            omg = omg + self.rng.uniform(-amp, amp, 3)
        return {"q": q_sim, "pos": pos, "vel": vel, "omg": omg,
                "rpy": _euler(quat.conj(q_sim)), "rpy_sim": _euler(q_sim)}

    def set_drone(self, pos):
        import pybullet as p

        p.resetBasePositionAndOrientation(
            self.env.DRONE_IDS[0], pos, [0, 0, 0, 1], physicsClientId=self.env.CLIENT)
        p.resetBaseVelocity(self.env.DRONE_IDS[0], [0, 0, 0], [0, 0, 0],
                            physicsClientId=self.env.CLIENT)

    def reset(self):
        self.set_drone(SPAWN)
        self.refs.update(roll=0.0, pitch=0.0, yaw=0.0, z=SPAWN[2])
        self.alt.reset()
        self.mode, self.mission = "MANUAL", "IDLE"
        self.paper_test = None
        self._thrust_override = self._paper_qref = None
        self.lock_position = True
        self._lock_point = SPAWN.copy()
        self.t_sim = 0.0

    def pilot_input(self, keys: set) -> None:
        gz = cfg.GUIDANCE
        tgt_p = 0.4 * ("w" in keys) - 0.4 * ("s" in keys)
        tgt_r = 0.4 * ("d" in keys) - 0.4 * ("a" in keys)
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])
        self.refs["roll"] += 0.35 * (tgt_r - self.refs["roll"])
        self.refs["pitch"] = float(np.clip(self.refs["pitch"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        self.refs["roll"] = float(np.clip(self.refs["roll"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        if "q" in keys:
            self.refs["yaw"] += gz["yaw_rate"] * POLL_EVERY * self.dt
        if "e" in keys:
            self.refs["yaw"] -= gz["yaw_rate"] * POLL_EVERY * self.dt
        up = ("r" in keys) or ("up" in keys)
        dn = ("f" in keys) or ("down" in keys)
        self.refs["z"] = float(np.clip(
            self.refs["z"] + (up - dn) * gz["climb"] * POLL_EVERY * self.dt, 0.25, 35))

    def start_paper_test(self, name: str) -> None:
        """Run the paper's benchmark protocol — same as the benchmark adapter:
        gravity on, altitude hold active, natural drift (the paper's controller
        is attitude-only, so drift is expected and honest). The interactive
        part is just the keyboard trigger."""
        self.paper_test, self.paper_t, self.mission = name, 0.0, "PAPER"
        alt0 = cfg.PAPER_TESTS[name]["start_alt"] if name == "flip" else 2.0
        self.refs.update(roll=0.0, pitch=0.0, z=alt0)

    def guidance(self) -> None:
        if self.mission in ("IDLE", "DONE", "HOLD"):
            self.refs.update(roll=0.0, pitch=0.0)
            return
        if self.mission == "PAPER":
            spec = cfg.PAPER_TESTS[self.paper_test]
            t = self.paper_t
            self.paper_t += POLL_EVERY * self.dt
            self._paper_qref = reference_quat(
                self.paper_test, min(t, spec["duration"]),
                flip_ramp=FLIP_RAMP_6DOF if self.paper_test == "flip" else None)
            if self.paper_test == "flip":
                tilt = float(np.clip(quat.to_dcm(self.read_state()["q"])[2, 2], -1, 1))
                self._thrust_override = (spec["idle_fraction"] * MASS * G) if tilt < 0.3 else None
            if t >= spec["duration"]:
                self._thrust_override = self._paper_qref = None
                self.mission = "HOLD"

    def control_tick(self) -> None:
        s = self.read_state(noisy=True)
        if self.mission == "PAPER" and self._paper_qref is not None:
            q_ref = self._paper_qref
        else:
            q_ref = quat.conj(quat.from_euler(self.refs["roll"], self.refs["pitch"], self.refs["yaw"]))
        q_m = quat.conj(s["q"])
        tau, _ = self.bridge.torque(q_ref, q_m, s["omg"])
        R = quat.to_dcm(s["q"])
        tilt = float(np.clip(R[2, 2], -1, 1))
        # altitude controller at 250 Hz (every 96 steps at 24 kHz), matching
        # the benchmark adapter — running it at 24 kHz causes instability
        # during the paper step test's extreme combined tilt
        if not hasattr(self, "_alt_count"):
            self._alt_count = 0
            self._alt_dt = 1.0 / 250.0
            self._alt_every = 96
        self._alt_count += 1
        if self._alt_count % self._alt_every == 0:
            if self._thrust_override is not None:
                thrust = self._thrust_override
            else:
                thrust = self.alt.thrust(self.refs["z"], s["pos"][2], s["vel"][2], tilt,
                                         MASS, PAPER_VEHICLE.thrust_max_total, self._alt_dt)
            self._last_thrust = thrust
        elif not hasattr(self, "_last_thrust"):
            self._last_thrust = MASS * G

        # rotor-order bridge: negate yaw, reorder to sim's prop 0..3
        tau_mixed = tau * np.array([1.0, 1.0, -1.0])
        thrusts, desat = self.bridge.mix(self._last_thrust, tau_mixed)
        thrusts_sim = [thrusts[j] for j in MY_TO_SIM]
        self.motors = np.array(thrusts_sim)
        self._held = {"fz": sum(thrusts), "tau": tau}
        self.telemetry = {"thrust": self._last_thrust,
                          "err_deg": np.degrees(2 * np.arccos(np.clip(abs(float(np.dot(q_ref, q_m))), 0, 1)))}

    def step(self) -> None:
        # normal rotor operation (thrust + torque through the mixer).
        # Position "lock" during paper tests is achieved by zero gravity +
        # high linearDamping (set in start_paper_test) — no discontinuous
        # velocity resets that would corrupt PyBullet's solver.
        rpms = np.sqrt(np.maximum(self.motors, 0) / KF)
        rpms = np.clip(rpms, 0, 25558)
        self.env.step(rpms[None, :])
        self.t_sim += self.dt

    def switch_mode(self):
        s = self.read_state()
        self.refs.update(roll=float(s["rpy_sim"][0]), pitch=float(s["rpy_sim"][1]),
                         yaw=float(s["rpy_sim"][2]), z=float(s["pos"][2]))
        self.alt.reset()
        if self.mode == "MANUAL":
            self.mode, self.mission = "OUR CONTROL", "HOLD"
        else:
            self.mode, self.mission = "MANUAL", "IDLE"
            self.lock_position = True
            self._lock_point = s["pos"].copy()


def main():
    import keyboard

    sim = PyBulletSim(gui=True)
    inp = InputManager()
    print(__doc__)
    print("\n>>> gym-pybullet interactive sim... Esc quits.\n")
    keyboard.on_press_key("esc", lambda _: sim.keys.add("__quit__"))

    k = 0
    t_wall = time.time()
    cam_mode = "FOLLOW"

    try:
        while "__quit__" not in sim.keys:
            held, taps = inp.poll()

            for tap in taps:
                if tap == "1":
                    if sim.mode != "OUR CONTROL":
                        sim.switch_mode()
                    sim.start_paper_test("step")
                    print(">>> PAPER STEP TEST\n")
                elif tap == "2":
                    if sim.mode != "OUR CONTROL":
                        sim.switch_mode()
                    sim.start_paper_test("sine")
                    print(">>> PAPER SINE TEST\n")
                elif tap == "3":
                    if sim.mode != "OUR CONTROL":
                        sim.switch_mode()
                    sim.start_paper_test("flip")
                    print(">>> PAPER FLIP TEST\n")
                elif tap == "space":
                    st = sim.read_state()
                    sim.refs.update(roll=0.0, pitch=0.0, z=float(st["pos"][2]))
                elif tap == "m":
                    sim.switch_mode()
                elif tap == "p":
                    sim.paused = not sim.paused
                elif tap == "t":
                    sim.reset()
                elif tap == "h":
                    sim.mission = "HOLD"
                elif tap == "c":
                    cam_mode = {"FOLLOW": "TOP", "TOP": "FREE", "FREE": "FOLLOW"}[cam_mode]

            if not sim.paused:
                if k % POLL_EVERY == 0:
                    if sim.mode == "MANUAL":
                        sim.pilot_input(held)
                    else:
                        sim.guidance()
                sim.control_tick()
                sim.step()

                # chase camera
                if sim.gui and k % RENDER_EVERY == 0:
                    import pybullet as p
                    if cam_mode == "FOLLOW":
                        pos = sim.read_state()["pos"]
                        p.resetDebugVisualizerCamera(2.0, 25, -20, pos)
                    elif cam_mode == "TOP":
                        p.resetDebugVisualizerCamera(15.0, 0, -89, [0, -6, 0])

            if k % HUD_EVERY == 0:
                s = sim.read_state()
                tel = sim.telemetry
                el = (time.time() - t_wall)
                speed = sim.t_sim / max(el, 1e-6)
                key_hud = " ".join(f"[{kk.upper()}]" if kk in held else kk.upper()
                                   for kk in ("w", "a", "s", "d", "q", "e", "r", "f"))
                track = sim.paper_test or "-"
                print(f"[{sim.mode:<11}|{sim.mission:<7}|{track:<8}] "
                      f"KEYS {key_hud} | attE={tel.get('err_deg',0):5.1f} | "
                      f"M {' '.join(f'{m:4.2f}' for m in sim.motors)} | "
                      f"z={s['pos'][2]:4.1f} | "
                      f"{'LOCKED' if sim.lock_position else 'FREE'} "
                      f"{speed:.1f}x{' PAUSE' if sim.paused else ''}")

            k += 1
    except KeyboardInterrupt:
        pass
    finally:
        inp.unhook()
        sim.env.close()
        print("bye")


if __name__ == "__main__":
    main()
