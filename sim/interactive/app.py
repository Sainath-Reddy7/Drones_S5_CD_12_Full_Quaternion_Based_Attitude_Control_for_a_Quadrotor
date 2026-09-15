"""INTERACTIVE MUJOCO DRONE SIMULATOR -- manual + our-control, one law.

Two modes, ONE pilot: the paper's unmodified quaternion controller (Pq=20,
Pw=4, +/-4 N*m) closes the attitude loop at 1 kHz in BOTH modes. The mode
only decides WHO commands the reference:

  MANUAL      you (keyboard -> attitude/altitude reference)
  OUR CONTROL the mission state machine (takeoff -> waypoints over the city
              -> return -> land); a small guidance layer converts position
              error into references -- scenario scaffolding, exactly like
              the step/sine/flip generators, NOT part of the paper law

Keys (global -- work while the 3D window has focus):

  W/S     pitch fwd/back       M      switch MANUAL <-> OUR CONTROL
  A/D     roll left/right      G      start mission (OUR CONTROL)
  Q/E     yaw left/right       H      hold position (OUR CONTROL)
  Up/Dn   climb / descend      L      land (OUR CONTROL)
  R/F     climb / descend      Space  hover: zero references (both modes)
  1       free camera          2      follow camera      3      top camera
  C       cycle cameras        Esc    quit

Physics: 10 kHz (dt 1e-4), empirically stable for the paper gains per the
repo's own fast-pole analysis (6150*1e-4 = 0.62 << 1.6); the benchmark
adapters keep 20 kHz + the strict 12.3 kHz assertion. Controller + wrench
refresh at 1 kHz (ZOH between ticks, like real flight code); rendering and
keyboard at ~60/100 Hz. Collisions with named environment geoms are
detected, shown in the HUD, and logged to sim/interactive/collisions.csv.

Launch:  python -m sim.interactive.app        (needs: pip install keyboard)
"""
from __future__ import annotations

import csv
import time
from pathlib import Path

import numpy as np

from quat_sitl import quaternion as quat
from sim.common import AltitudeHold, PAPER_VEHICLE, QuatBridge, thrusts_to_wrench_zup

XML = Path(__file__).with_name("world_city.xml")
LOG = Path(__file__).with_name("collisions.csv")

MASS = 0.2
CTRL_EVERY = 10          # 1 kHz control ticks at 10 kHz physics
POLL_EVERY = 100         # 100 Hz input/guidance
HUD_EVERY = 40           # ~25 Hz HUD
MAX_TILT = 0.45          # rad of pilot/mission authority
YAW_RATE = 1.3           # rad/s
CLIMB = 1.0              # m/s
SPAWN = np.array([0.0, -6.0, 1.2])

# mission waypoints (x, y, z) -- flown over the road corridor, past the
# buildings, trucks and trees; nothing here fakes success, it is a script
WAYPOINTS = [
    np.array([0.0, 6.0, 3.0]),
    np.array([0.0, 26.0, 4.5]),
    np.array([7.0, 30.0, 5.5]),
    np.array([0.0, -2.0, 3.5]),
    np.array([0.0, -6.0, 3.0]),   # over the pad, then land
]
POS_K, VEL_K, ACC_MAX = 1.1, 1.6, 2.8

BENIGN = {"floor", "pad_base"}     # ground/pad touches are landings, not crashes


class DroneSim:
    """Core simulation (viewer/keyboard-free) -- also used by the tests."""

    def __init__(self, seed: int = 0):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(XML))
        self.data = mujoco.MjData(self.model)
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.dt = float(self.model.opt.timestep)
        self.rate = 1.0 / self.dt

        self.bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=True)
        self.alt = AltitudeHold(max_thrust_factor=2.2)

        self.mode = "MANUAL"           # or OUR CONTROL
        self.mission = "IDLE"          # IDLE/TAKEOFF/WAYPOINT/RETURN/LAND/DONE/HOLD
        self.wp = 0
        self.refs = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "z": SPAWN[2]}
        self.keys = set()
        self.collisions: list[dict] = []
        self.telemetry: dict = {}
        self._held = {"R": np.eye(3), "fz": MASS * 9.81, "tau": np.zeros(3), "q": np.eye(4)}

        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.data.qpos[:] = [*SPAWN, 1, 0, 0, 0]
        self.data.qvel[:] = 0
        self.refs.update(roll=0.0, pitch=0.0, yaw=0.0, z=SPAWN[2])
        self.alt.reset()
        self.mode = "MANUAL"
        self.mission = "IDLE"
        self.wp = 0

    def read_state(self) -> dict:
        q = np.array(self.data.qpos[3:7])
        pos = np.array(self.data.qpos[0:3])
        vel = np.array(self.data.qvel[0:3])
        omg = np.array(self.data.qvel[3:6])
        return {"q": q, "pos": pos, "vel": vel, "omg": omg,
                "rpy": quat.to_euler(quat.conj(q))}

    # ------------------------------------------------------------------ input
    def pilot_input(self, keys: set) -> None:
        """MANUAL mode: keys -> references (50-100 Hz)."""
        tgt_p = 0.4 * ("w" in keys) - 0.4 * ("s" in keys)
        tgt_r = 0.4 * ("d" in keys) - 0.4 * ("a" in keys)
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])
        self.refs["roll"] += 0.35 * (tgt_r - self.refs["roll"])
        self.refs["roll"] = float(np.clip(self.refs["roll"], -MAX_TILT, MAX_TILT))
        self.refs["pitch"] = float(np.clip(self.refs["pitch"], -MAX_TILT, MAX_TILT))
        if "q" in keys:
            self.refs["yaw"] += YAW_RATE / self.rate * POLL_EVERY
        if "e" in keys:
            self.refs["yaw"] -= YAW_RATE / self.rate * POLL_EVERY
        up = ("up" in keys) or ("r" in keys)
        dn = ("down" in keys) or ("f" in keys)
        self.refs["z"] = float(np.clip(self.refs["z"] + (up - dn) * CLIMB / self.rate * POLL_EVERY, 0.25, 40))

    # ---------------------------------------------------------------- mission
    def guidance(self) -> None:
        """OUR CONTROL: mission state machine -> references.
        Position error -> desired velocity -> desired acceleration -> tilt
        reference in the yaw frame (scenario scaffolding; the paper law
        tracks the reference)."""
        s = self.read_state()
        if self.mission in ("IDLE", "DONE", "HOLD"):
            self.refs.update(roll=0.0, pitch=0.0)      # hold position/attitude
            return
        if self.mission == "TAKEOFF":
            self.refs["z"] = min(3.0, self.refs["z"] + 0.8 / self.rate * POLL_EVERY)
            if s["pos"][2] > 2.5:
                self.mission, self.wp = "WAYPOINT", 0
            return
        if self.mission == "LAND":
            self.refs["z"] = max(0.15, self.refs["z"] - 0.5 / self.rate * POLL_EVERY)
            self.refs.update(roll=0.0, pitch=0.0)
            if s["pos"][2] < 0.2 and np.linalg.norm(s["vel"]) < 0.4:
                self.mission = "DONE"
            return
        # WAYPOINT / RETURN
        tgt = WAYPOINTS[min(self.wp, len(WAYPOINTS) - 1)]
        err = tgt - s["pos"]
        if np.linalg.norm(err[:2]) < 0.8 and abs(err[2]) < 0.5 and np.linalg.norm(s["vel"]) < 1.0:
            self.wp += 1
            if self.wp >= len(WAYPOINTS):
                self.mission = "LAND"
                return
            tgt = WAYPOINTS[self.wp]
            err = tgt - s["pos"]
        v_des = POS_K * err
        v_des[2] = np.clip(v_des[2], -1.2, 1.2)
        n = np.linalg.norm(v_des[:2])
        if n > 2.5:
            v_des[:2] *= 2.5 / n
        a_cmd = VEL_K * (v_des - s["vel"])
        a_cmd[:2] = np.clip(a_cmd[:2], -ACC_MAX, ACC_MAX)
        a_cmd[2] = 0  # altitude handled by AltitudeHold
        g = 9.81
        # world accel -> tilt in the yaw frame (small-angle standard mapping)
        psi = self.refs["yaw"]
        pitch_w = np.clip(a_cmd[0] / g, -0.4, 0.4)
        roll_w = np.clip(-a_cmd[1] / g, -0.4, 0.4)
        self.refs["pitch"] = float(pitch_w * np.cos(psi) + roll_w * np.sin(psi))
        self.refs["roll"] = float(roll_w * np.cos(psi) - pitch_w * np.sin(psi))
        # face the travel direction
        if np.linalg.norm(err[:2]) > 1.5:
            self.refs["yaw"] = float(np.arctan2(err[0], err[1]))
        self.refs["z"] = float(tgt[2])

    # ------------------------------------------------------------- controller
    def control_tick(self) -> None:
        """1 kHz: the paper law + altitude hold + mixer (the adapter to
        MuJoCo is the same proven bridge as every other stack)."""
        s = self.read_state()
        q_ref = quat.from_euler(self.refs["roll"], self.refs["pitch"], self.refs["yaw"])
        q_m = quat.conj(s["q"])
        tau, _ = self.bridge.torque(q_ref, q_m, s["omg"])
        R = quat.to_dcm(s["q"])
        tilt = float(np.clip(R[2, 2], -1.0, 1.0))
        thrust = self.alt.thrust(
            self.refs["z"], s["pos"][2], s["vel"][2], tilt, MASS,
            PAPER_VEHICLE.thrust_max_total, 1.0 / self.rate * CTRL_EVERY,
        )
        thrusts, desat = self.bridge.mix(thrust, tau)
        fz, tau_a = thrusts_to_wrench_zup(thrusts, PAPER_VEHICLE)
        self._held = {"R": R, "fz": fz, "tau": tau_a, "q": q_ref, "thrust": thrust, "desat": desat}
        self.telemetry = {
            "tau_cmd": tau, "thrust": thrust, "desat": desat,
            "err_deg": np.degrees(2 * np.arccos(np.clip(abs(float(np.dot(q_ref, q_m))), 0, 1))),
        }

    # --------------------------------------------------------------- physics
    def step(self) -> None:
        R, fz, tau = self._held["R"], self._held["fz"], self._held["tau"]
        self.data.xfrc_applied[self.bid, 0:3] = R @ np.array([0.0, 0.0, fz])
        self.data.xfrc_applied[self.bid, 3:6] = R @ tau
        self.mj.mj_step(self.model, self.data)

    DRONE_PARTS = ("prop", "mount", "arm", "shell", "plate", "cam",
                   "lens", "led", "frame")

    def check_collisions(self, t: float) -> list[str]:
        """Report environment objects in contact with the drone (deduped).
        Ground/pad touches are landings, not crashes; drone-internal pairs
        cannot occur (single collision geom on a free body)."""
        mjn = self.mj.mj_id2name
        geom = self.mj.mjtObj.mjOBJ_GEOM
        hits: set[str] = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            for me, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                name = mjn(self.model, geom, me)
                if name is None or name in BENIGN:
                    continue
                if name.startswith(self.DRONE_PARTS):
                    oname = mjn(self.model, geom, other)
                    if oname is None or oname in BENIGN or oname.startswith(self.DRONE_PARTS):
                        continue  # drone touching ground/pad = landing
                    hits.add(oname)
                else:
                    # environment geom touching the drone (order-independent)
                    oname = mjn(self.model, geom, other)
                    if oname is not None and oname.startswith(self.DRONE_PARTS):
                        hits.add(name)
        for h in hits:
            self.collisions.append({"t": round(t, 2), "object": h})
        return sorted(hits)

    # ------------------------------------------------------------- main loop
    def run(self) -> None:
        import mujoco.viewer
        import keyboard

        print(__doc__)
        print(">>> launching 3D window... Esc quits, M switches mode.\n")
        keyboard.on_press_key("esc", lambda _: self.keys.add("__quit__"))

        t_wall = time.time()
        k = 0
        warn, warn_age = "", -10.0
        slow_warned = False
        fps = 0.0
        t_fps = t_wall

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            cam_mode = "FREE"
            while viewer.is_running() and "__quit__" not in self.keys:
                # -------- input (100 Hz) -----------------------------------
                if k % POLL_EVERY == 0:
                    ks = set()
                    for key in ("w", "a", "s", "d", "q", "e", "r", "f", "up", "down"):
                        if keyboard.is_pressed(key):
                            ks.add(key)
                    if keyboard.is_pressed("space"):
                        self.refs.update(roll=0.0, pitch=0.0)
                    if keyboard.is_pressed("m"):
                        self.switch_mode()
                        time.sleep(0.2)
                    for cmd, act in (("g", "GO"), ("h", "HOLD"), ("l", "LAND")):
                        if keyboard.is_pressed(cmd) and self.mode == "OUR CONTROL":
                            self.mission_cmd(act)
                            time.sleep(0.2)
                    if keyboard.is_pressed("c"):
                        cam_mode = {"FREE": "FOLLOW", "FOLLOW": "TOP", "TOP": "FREE"}[cam_mode]
                        time.sleep(0.2)
                    for num, cm in (("1", "FREE"), ("2", "FOLLOW"), ("3", "TOP")):
                        if keyboard.is_pressed(num):
                            cam_mode = cm
                    if self.mode == "MANUAL":
                        self.pilot_input(ks)
                    else:
                        self.guidance()

                # -------- control + physics (10 kHz) ------------------------
                if k % CTRL_EVERY == 0:
                    self.control_tick()
                self.step()
                hits = self.check_collisions(k * self.dt)
                if hits:
                    warn, warn_age = f"COLLISION: {hits[0]}", k * self.dt

                # -------- render + cameras (~ every 10 ms) ------------------
                if k % 100 == 0:
                    if cam_mode == "FOLLOW":
                        with viewer.lock():
                            viewer.cam.lookat[:] = self.data.qpos[0:3]
                            viewer.cam.distance = 4.0
                    elif cam_mode == "TOP":
                        with viewer.lock():
                            viewer.cam.lookat[:] = [0, 6, 0]
                            viewer.cam.distance = 45.0
                            viewer.cam.elevation = -90
                            viewer.cam.azimuth = 90
                    viewer.sync()

                # -------- HUD (25 Hz) --------------------------------------
                if k % HUD_EVERY == 0:
                    s = self.read_state()
                    tel = self.telemetry
                    now = time.time()
                    if now - t_fps > 0.5:
                        fps = int(HUD_EVERY * (0.5 / max(1e-9, now - t_fps)) * 20)
                        t_fps = now
                    rt = "SLOW" if now - t_wall > k * self.dt * 1.5 + 2 else "RT"
                    print(
                        f"[{self.mode:^11}] {self.mission:^8} cam={cam_mode:6} "
                        f"z={s['pos'][2]:5.2f} r={np.degrees(s['rpy'][0]):+6.1f} "
                        f"p={np.degrees(s['rpy'][1]):+6.1f} y={np.degrees(s['rpy'][2]):+7.1f} "
                        f"v={np.linalg.norm(s['vel']):4.1f} thr={tel.get('thrust', 0):4.2f}N "
                        f"err={tel.get('err_deg', 0):5.1f}deg {rt} "
                        + (f"| !! {warn}" if k * self.dt - warn_age < 2 else "")
                    )

                # -------- real-time pacing ----------------------------------
                if k % 1000 == 0 and k > 0:
                    target = k * self.dt
                    delta = target - (time.time() - t_wall)
                    if delta > 0:
                        time.sleep(min(delta, 0.05))
                    elif delta < -3 and not slow_warned:
                        print("  (machine below real-time; sim continues slower)")
                        slow_warned = True
                k += 1

        self.shutdown()

    # ------------------------------------------------------------- helpers
    def switch_mode(self) -> None:
        """Safe handoff: new mode's references initialize from the CURRENT
        state -- no reference jumps, altitude hold keeps integrating."""
        s = self.read_state()
        self.refs["roll"], self.refs["pitch"] = float(s["rpy"][0]), float(s["rpy"][1])
        self.refs["yaw"] = float(s["rpy"][2])
        self.refs["z"] = float(s["pos"][2])
        if self.mode == "MANUAL":
            self.mode, self.mission = "OUR CONTROL", "HOLD"
        else:
            self.mode, self.mission = "MANUAL", "IDLE"
        print(f"\n== MODE -> {self.mode} (refs initialized from current state) ==\n")

    def mission_cmd(self, act: str) -> None:
        s = self.read_state()
        self.refs["z"] = float(s["pos"][2])
        if act == "GO":
            self.mission, self.wp = "TAKEOFF", 0
        elif act == "HOLD":
            self.mission = "HOLD"
        elif act == "LAND":
            self.mission = "LAND"

    def shutdown(self) -> None:
        if self.collisions:
            with open(LOG, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["t", "object"])
                w.writeheader()
                w.writerows(self.collisions)
            print(f"\ncollision log -> {LOG}")
        print("bye")


def main() -> None:
    DroneSim().run()


if __name__ == "__main__":
    main()
