"""INTERACTIVE gym-pybullet-drones simulator — the paper's 3 tests, live.

Same architecture as the MuJoCo interactive app: position PINNED (pure
attitude, the paper's plant — no translational states), paper tests on
X/Z/B, attitude flight on IJKL, live HUD + reference triad.

THE PAPER'S 3 TESTS (from any mode, one keypress — same as the MuJoCo app):
    X = STEP   (1 rad steps on roll/pitch/yaw)
    Z = SINE   (0.5 rad wave tracking under noise)
    B = FLIP   (360-degree rotation, no gimbal lock)

FLIGHT (held keys, manual mode only). The drone is PINNED at one point —
attitude only, exactly like the MuJoCo app: tilting rotates it in place.
WASD/arrow keys are PyBullet's own GUI camera controls, so flight lives
on the IJKL cluster:
    I/K = pitch fwd/back    J/L = roll left/right
    U/O = yaw left/right    SPACE = hover (level)

MODE:
    M = toggle MANUAL <-> OUR CONTROL
    H = hold                P = pause
    T = reset               ESC = quit

CAMERA + VISIBILITY (what you see while a test runs):
    C = cycle ORBIT / FOLLOW / TOP / FREE (mouse works in FREE)
    ORBIT (default) slowly circles the drone so every angle of the three
    paper tests is visible. A live HUD floats above the drone (mode, test,
    progress, attitude error, motor commands) and an RGB triad shows the
    paper's REFERENCE attitude: R = front/+x, G = +y, B = +z/down-thrust.
    The drone itself is color-coded the same way: RED arms/props front,
    BLUE rear — so perfect tracking = red arms aligned with the red triad
    line, blue with blue. Each test ends with a printed rms/max error
    summary on the console.

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
PIN_EVERY = 24      # position-pin refresh at 1 kHz (see PyBulletSim.step)
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

        # in-window HUD / triad item ids (-1 = not created yet) and the
        # per-test error statistics printed when a paper test finishes
        self._hud_l1 = -1
        self._hud_l2 = -1
        self._triad = [-1, -1, -1]
        self._orbit_yaw = 35.0
        self._err_n = 0
        self._err_ssq = 0.0
        self._err_max = 0.0
        self._pin_count = 0

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
        self.bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=True)
        self.lock_position = True
        self._lock_point = SPAWN.copy()
        self.t_sim = 0.0

    def pilot_input(self, keys: set) -> None:
        gz = cfg.GUIDANCE
        tgt_p = 0.4 * ("i" in keys) - 0.4 * ("k" in keys)
        tgt_r = 0.4 * ("l" in keys) - 0.4 * ("j" in keys)
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])
        self.refs["roll"] += 0.35 * (tgt_r - self.refs["roll"])
        self.refs["pitch"] = float(np.clip(self.refs["pitch"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        self.refs["roll"] = float(np.clip(self.refs["roll"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        if "u" in keys:
            self.refs["yaw"] += gz["yaw_rate"] * POLL_EVERY * self.dt
        if "o" in keys:
            self.refs["yaw"] -= gz["yaw_rate"] * POLL_EVERY * self.dt
        up = "r" in keys
        dn = "f" in keys
        self.refs["z"] = float(np.clip(
            self.refs["z"] + (up - dn) * gz["climb"] * POLL_EVERY * self.dt, 0.25, 35))

    def start_paper_test(self, name: str) -> None:
        """Run the paper's benchmark protocol, MuJoCo-style: the drone is
        PINNED at one point (the paper's attitude-only plant) so the test
        is pure attitude tracking you can watch in place — step, sine and
        flip all happen without drift or altitude margin. The pinned plant
        keeps the bridge's shortest_path=True for all three tests: with the
        position pin the flip never stalls past 180 deg of lag, so the
        short-path law coasts over the top and converges back (measured:
        peak roll 180 deg, final err <10 deg). The free-fall benchmark
        adapter (run.py) instead uses shortest_path=False for its flip."""
        self.paper_test, self.paper_t, self.mission = name, 0.0, "PAPER"
        self._err_n, self._err_ssq, self._err_max = 0, 0.0, 0.0
        self.lock_position = True
        self._lock_point = np.asarray(self.read_state()["pos"], dtype=float).copy()
        self.refs.update(roll=0.0, pitch=0.0, z=float(self._lock_point[2]))

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
                if self._err_n:
                    rms = (self._err_ssq / self._err_n) ** 0.5
                    print(f">>> {self.paper_test.upper()} done: rms err "
                          f"{rms:.2f} deg, max {self._err_max:.2f} deg "
                          f"over {spec['duration']:g} s\n")
                self._thrust_override = self._paper_qref = None
                self.mission = "HOLD"

    def current_qref(self):
        """The reference attitude the paper law is tracking right now
        (world->body convention, same as control_tick consumes)."""
        if self.mission == "PAPER" and self._paper_qref is not None:
            return self._paper_qref
        return quat.conj(quat.from_euler(self.refs["roll"], self.refs["pitch"], self.refs["yaw"]))

    def draw_hud(self, p, pos, cam_mode: str, speed: float) -> None:
        """In-window HUD above the drone + RGB triad of the reference
        attitude (R=front/+x, G=+y, B=+z). The colored arms show the actual
        attitude: tracking error = arms vs triad mismatch."""
        if not self.gui:
            return
        if self.mission == "PAPER" and self.paper_test:
            dur = cfg.PAPER_TESTS[self.paper_test]["duration"]
            track = f"{self.paper_test.upper()} {min(self.paper_t, dur):4.1f}/{dur:.0f}s"
        else:
            track = self.paper_test or "-"
        l1 = f"[{self.mode} | {self.mission} | {track}]  cam {cam_mode}  {speed:.1f}x"
        l2 = (f"attE {self.telemetry.get('err_deg', 0.0):5.1f} deg   "
              f"M {' '.join(f'{m:4.2f}' for m in self.motors)}   z {pos[2]:4.1f} m")
        a = np.asarray(pos, dtype=float)
        cid = self.env.CLIENT
        if self._hud_l1 >= 0:
            p.removeUserDebugItem(self._hud_l1, physicsClientId=cid)
            p.removeUserDebugItem(self._hud_l2, physicsClientId=cid)
        self._hud_l1 = p.addUserDebugText(
            l1, (a + np.array([0.0, 0.0, 0.55])).tolist(),
            textColorRGB=[1, 1, 1], textSize=1.3, physicsClientId=cid)
        self._hud_l2 = p.addUserDebugText(
            l2, (a + np.array([0.0, 0.0, 0.40])).tolist(),
            textColorRGB=[1, 0.85, 0.2], textSize=1.2, physicsClientId=cid)
        R = quat.to_dcm(quat.conj(self.current_qref()))  # body->world, ref
        cols = ([1, 0.1, 0.1], [0.1, 1, 0.1], [0.15, 0.35, 1])
        for i in range(3):
            self._triad[i] = p.addUserDebugLine(
                a.tolist(), (a + R[i] * 0.38).tolist(), lineColorRGB=cols[i],
                lineWidth=3, physicsClientId=cid,
                replaceItemUniqueId=self._triad[i])

    def control_tick(self) -> None:
        s = self.read_state(noisy=True)
        q_ref = self.current_qref()
        q_m = quat.conj(s["q"])
        tau, _ = self.bridge.torque(q_ref, q_m, s["omg"])
        R = quat.to_dcm(s["q"])
        tilt = float(np.clip(R[2, 2], -1, 1))
        if self.lock_position:
            # MuJoCo parity: the pin handles gravity/position, so the
            # altitude controller is unnecessary and harmful (its integrator
            # fights the pin and eats the mixer's thrust headroom during
            # 1-rad tilts). Hover thrust + full torque authority = the
            # paper's pure attitude plant.
            self._last_thrust = MASS * G
            if self._thrust_override is not None:
                self._last_thrust = self._thrust_override
        else:
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
        if self.mission == "PAPER":
            e = float(self.telemetry["err_deg"])
            self._err_n += 1
            self._err_ssq += e * e
            self._err_max = max(self._err_max, e)

    def step(self) -> None:
        # normal rotor operation (thrust + torque through the mixer)
        rpms = np.sqrt(np.maximum(self.motors, 0) / KF)
        rpms = np.clip(rpms, 0, 25558)
        self.env.step(rpms[None, :])
        self.t_sim += self.dt

        # PIN POSITION (MuJoCo parity): the paper's plant is attitude-ONLY
        # (no translational states). Pin the base to the lock point and zero
        # ONLY the linear velocity — orientation and angular velocity pass
        # through untouched, so the attitude dynamics remain fully
        # solver-integrated (this is what makes the flip honest). Refreshed
        # at 1 kHz: worst-case excursion between pins is ~4g*(1ms)^2/2
        # ~ 20 micrometers, invisible at any camera distance.
        if self.lock_position:
            self._pin_count += 1
            if self._pin_count % PIN_EVERY == 0:
                import pybullet as p

                cid = self.env.CLIENT
                bid = int(self.env.DRONE_IDS[0])
                # read angular velocity BEFORE the pose reset: this PyBullet
                # build zeroes base velocities inside resetBasePositionAnd-
                # Orientation — restoring from a post-reset read would write
                # back zeros and damp the attitude dynamics to a standstill
                _, ang = p.getBaseVelocity(bid, physicsClientId=cid)
                _, orn = p.getBasePositionAndOrientation(bid, physicsClientId=cid)
                p.resetBasePositionAndOrientation(
                    bid, self._lock_point.tolist(), orn, physicsClientId=cid)
                p.resetBaseVelocity(bid, [0.0, 0.0, 0.0], ang,
                                    physicsClientId=cid)

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
    cam_mode = "ORBIT"

    try:
        while "__quit__" not in sim.keys:
            held, taps = inp.poll()

            for tap in taps:
                if tap == "x":
                    if sim.mode != "OUR CONTROL":
                        sim.switch_mode()
                    sim.start_paper_test("step")
                    print(">>> PAPER STEP TEST\n")
                elif tap == "z":
                    if sim.mode != "OUR CONTROL":
                        sim.switch_mode()
                    sim.start_paper_test("sine")
                    print(">>> PAPER SINE TEST\n")
                elif tap == "b":
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
                    cam_mode = {"ORBIT": "FOLLOW", "FOLLOW": "TOP",
                                "TOP": "FREE", "FREE": "ORBIT"}[cam_mode]

            if not sim.paused:
                if k % POLL_EVERY == 0:
                    if sim.mode == "MANUAL":
                        sim.pilot_input(held)
                    else:
                        sim.guidance()
                sim.control_tick()
                sim.step()

                # camera + in-window HUD: close, drone-centric views so the
                # paper tests read clearly (the vehicle is only 0.34 m across)
                if sim.gui and k % RENDER_EVERY == 0:
                    import pybullet as p
                    pos = sim.read_state()["pos"]
                    if cam_mode == "FOLLOW":
                        p.resetDebugVisualizerCamera(1.0, 25, -22, pos,
                                                     physicsClientId=sim.env.CLIENT)
                    elif cam_mode == "ORBIT":
                        sim._orbit_yaw = (sim._orbit_yaw + 0.35) % 360.0
                        p.resetDebugVisualizerCamera(1.15, sim._orbit_yaw, -14, pos,
                                                     physicsClientId=sim.env.CLIENT)
                    elif cam_mode == "TOP":
                        p.resetDebugVisualizerCamera(1.6, 0, -89, pos,
                                                     physicsClientId=sim.env.CLIENT)
                    speed = sim.t_sim / max(time.time() - t_wall, 1e-6)
                    sim.draw_hud(p, pos, cam_mode, speed)

            if k % HUD_EVERY == 0:
                s = sim.read_state()
                tel = sim.telemetry
                el = (time.time() - t_wall)
                speed = sim.t_sim / max(el, 1e-6)
                key_hud = " ".join(f"[{kk.upper()}]" if kk in held else kk.upper()
                                   for kk in ("i", "k", "j", "l", "u", "o", "r", "f"))
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
