"""INTERACTIVE MUJOCO DRONE SIMULATOR -- pro edition, fixed input pipeline.

CONTROL PIPELINE (every stage real, every stage tested):
  KEYBOARD -> InputManager (event hook) -> pilot refs -> paper P^2 law ->
  priority mixer -> 4 rotor thrusts -> MuJoCo wrench -> dynamics -> state

MODES:  M manual (you fly)  |  O our-control (autopilot / paper tests)
OUR CONTROL:  G mission  U circle  8 figure8  V straight
              6 PAPER-STEP  7 PAPER-SINE  9 PAPER-FLIP (base paper, live)
              H hold  L land  K abort
ENVIRONMENT:  E cycles environment (urban_v1 / open_field_v1, isolated dirs)
              N wind cycle   Y noise cycle (PAPER = the paper's +/-0.1 model)
SIM:  P pause  T reset drone  [ ] speed 0.25..2x  Esc quit
CAM:  1 free  2 follow  3 chase  4 top  5 overview  C cycle

HUD shows: live KEY STATE [W][A][S][D]..., desired attitude, motor thrusts
M1-M4, position/velocity, tracking error, wind/noise, FPS, collisions.
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

BASE = Path(__file__).parent
ENVS = BASE / "envs"
LOGDIR = BASE / "logs"
ENV_LIST = ["urban_v1", "open_field_v1"]

MASS = cfg.DRONE["mass_kg"]
CTRL_EVERY = cfg.CONTROLLER["physics_hz"] // cfg.CONTROLLER["control_hz"]
POLL_EVERY = 100
HUD_EVERY = 40
LOG_EVERY = 200
G = 9.81

BENIGN = {"floor", "pad_base"}
DRONE_PARTS = ("prop", "mount", "arm", "shell", "plate", "cam", "lens", "led", "frame")
SPAWN = np.array([0.0, -6.0, 1.2])


def _euler(q) -> tuple[float, float, float]:
    return tuple(float(x) for x in quat.to_euler(q))


class DroneSim:
    def __init__(self, seed: int = 0, env: str = "urban_v1"):
        import mujoco

        self.mj = mujoco
        self.env = env if env in ENV_LIST else ENV_LIST[0]
        self.model = mujoco.MjModel.from_xml_path(str(ENVS / self.env / "world.xml"))
        self.data = mujoco.MjData(self.model)
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.dt = float(self.model.opt.timestep)
        self.rate = 1.0 / self.dt
        self.rng = np.random.default_rng(seed)

        self.bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=True)
        self.alt = AltitudeHold(max_thrust_factor=2.2)

        self.mode = "MANUAL"
        self.mission = "IDLE"
        self.paused = False
        self.track_type = None
        self.track_t = 0.0
        self.wp = 0
        self._seg = 0.0
        self.paper_test = None
        self.paper_t = 0.0
        self.wind = "OFF"
        self.noise = "PAPER"
        self.speed = 1.0
        self.refs = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "z": SPAWN[2]}
        self.keys: set = set()
        self.collisions: list[dict] = []
        self.telemetry: dict = {}
        self.log_rows: list[list] = []
        self.track_log: list = []
        self.xtrack = 0.0
        self.motors = np.zeros(4)
        self._held = {"R": np.eye(3), "fz": MASS * G, "tau": np.zeros(3), "wind": np.zeros(3)}
        self._thrust_override = None
        self._paper_qref = None

        self._qadr = self.model.jnt_qposadr[self.model.body_jntadr[self.bid]]
        self._vadr = self.model.jnt_dofadr[self.model.body_jntadr[self.bid]]
        self._mk = self._markers()
        self._traffic = self._collect_traffic()
        self.reset()

    def _markers(self) -> dict:
        g = lambda n: self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_GEOM, n)
        return {"dots": [g(f"mk_d{i}") for i in range(96)],
                "wps": [g(f"mk_w{i}") for i in range(24)], "target": g("mk_target")}

    def _collect_traffic(self) -> list:
        out = []
        for n in ("car_m1", "car_m2", "truck_m1"):
            b = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, n)
            a = self.model.jnt_dofadr[self.model.body_jntadr[b]]
            out.append((n, b, float(self.data.qpos[a + 1])))
        return out

    def _hide_markers(self) -> None:
        for g in self._mk["dots"] + self._mk["wps"] + [self._mk["target"]]:
            self.model.geom_pos[g] = [0, 0, -10]

    def show_path(self, pts, wps=None) -> None:
        self._hide_markers()
        for i in range(min(len(pts), 96)):
            self.model.geom_pos[self._mk["dots"][i]] = pts[i]
        if wps is not None:
            for i, w in enumerate(wps[:24]):
                self.model.geom_pos[self._mk["wps"][i]] = w

    def _target_marker(self, p) -> None:
        self.model.geom_pos[self._mk["target"]] = p

    def set_drone(self, pos, quat4=(1, 0, 0, 0)) -> None:
        self.data.qpos[self._qadr:self._qadr + 7] = [*pos, *quat4]
        self.data.qvel[self._vadr:self._vadr + 6] = 0

    def reset(self) -> None:
        self.set_drone(SPAWN)
        for _, b, y0 in self._traffic:
            self.data.qpos[self.model.jnt_dofadr[self.model.body_jntadr[b]] + 1] = y0
        self.refs.update(roll=0.0, pitch=0.0, yaw=0.0, z=SPAWN[2])
        self.alt.reset()
        self.mode, self.mission = "MANUAL", "IDLE"
        self.track_type = self.paper_test = None
        self._thrust_override = self._paper_qref = None
        self.track_log.clear()
        self._hide_markers()

    def read_state(self, noisy: bool = False) -> dict:
        q = np.array(self.data.qpos[self._qadr + 3:self._qadr + 7])
        pos = np.array(self.data.qpos[self._qadr:self._qadr + 3])
        vel = np.array(self.data.qvel[self._vadr:self._vadr + 3])
        omg = np.array(self.data.qvel[self._vadr + 3:self._vadr + 6])
        a = cfg.NOISE_LEVELS[self.noise]
        if noisy and a > 0:
            q = quat.normalize(q + self.rng.uniform(-a, a, 4))
            omg = omg + self.rng.uniform(-a, a, 3)
        return {"q": q, "pos": pos, "vel": vel, "omg": omg,
                "rpy": _euler(quat.conj(q)), "rpy_sim": _euler(q)}

    # MANUAL: keys -> attitude/altitude refs (heading-relative by construction)
    def pilot_input(self, keys: set) -> None:
        gz = cfg.GUIDANCE
        tgt_p = 0.4 * ("w" in keys) - 0.4 * ("s" in keys)
        tgt_r = 0.4 * ("d" in keys) - 0.4 * ("a" in keys)
        if "i" in keys: tgt_p += 0.25
        if "k" in keys: tgt_p -= 0.25
        if "j" in keys: tgt_r -= 0.25
        if "l" in keys: tgt_r += 0.25
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])
        self.refs["roll"] += 0.35 * (tgt_r - self.refs["roll"])
        self.refs["pitch"] = float(np.clip(self.refs["pitch"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        self.refs["roll"] = float(np.clip(self.refs["roll"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        if "q" in keys:
            self.refs["yaw"] += gz["yaw_rate"] / self.rate * POLL_EVERY
        if "e" in keys:
            self.refs["yaw"] -= gz["yaw_rate"] / self.rate * POLL_EVERY
        up = ("r" in keys) or ("up" in keys)
        dn = ("f" in keys) or ("down" in keys)
        self.refs["z"] = float(np.clip(
            self.refs["z"] + (up - dn) * gz["climb"] / self.rate * POLL_EVERY,
            cfg.SAFETY["min_alt"], cfg.SAFETY["max_alt"]))

    def _path_points(self, kind):
        if kind == "mission":
            wps = [np.array(w) for w in cfg.PATHS["mission"]]
            pts = []
            for a, b in zip(wps, wps[1:]):
                n = max(2, int(np.linalg.norm(b - a) * 3))
                pts += [a + (b - a) * i / n for i in range(n)]
            return np.array(pts + [wps[-1]]), wps
        if kind == "circle":
            c, r = cfg.PATHS["circle"]["center"], cfg.PATHS["circle"]["radius"]
            ts = np.linspace(0, 2 * np.pi, 96)
            return np.array([[c[0] + r * np.sin(t), c[1] + r * np.cos(t), c[2]] for t in ts]), None
        if kind == "figure8":
            p = cfg.PATHS["figure8"]
            ts = np.linspace(0, 2 * np.pi, 96)
            return np.array([[p["center"][0] + p["a"] * np.sin(t),
                              p["center"][1] + p["b"] * np.sin(t) * np.cos(t) * 2,
                              p["center"][2]] for t in ts]), None
        if kind == "straight":
            p = cfg.PATHS["straight"]
            a, b = np.array(p["start"]), np.array(p["end"])
            n = 60
            return np.array([a + (b - a) * i / n for i in range(n + 1)]), None
        raise ValueError(kind)

    def _track_ref(self, t):
        if self.track_type == "mission":
            wps = [np.array(w) for w in cfg.PATHS["mission"]]
            i = min(self.wp, len(wps) - 2)
            a, b = wps[i], wps[i + 1]
            seg = max(np.linalg.norm(b - a), 1e-6)
            p = a + (b - a) * (min(self._seg, seg) / seg)
            return p, float(np.arctan2(b[0] - a[0], b[1] - a[1])), b
        if self.track_type == "circle":
            c, r, w = (cfg.PATHS["circle"][k] for k in ("center", "radius", "omega"))
            th = w * t
            p = np.array([c[0] + r * np.sin(th), c[1] + r * np.cos(th), c[2]])
            return p, float(th + np.pi), p
        if self.track_type == "figure8":
            p = cfg.PATHS["figure8"]
            th = p["omega"] * t
            pos = np.array([p["center"][0] + p["a"] * np.sin(th),
                            p["center"][1] + p["b"] * np.sin(th) * np.cos(th) * 2, p["center"][2]])
            d = np.array([p["a"] * np.cos(th), p["b"] * 2 * (np.cos(th) ** 2 - np.sin(th) ** 2), 0])
            return pos, float(np.arctan2(d[0], d[1])), pos
        if self.track_type == "straight":
            p = cfg.PATHS["straight"]
            a, b = np.array(p["start"]), np.array(p["end"])
            L = max(np.linalg.norm(b - a), 1e-6)
            pr = a + (b - a) * (min(p["speed"] * t, L) / L)
            return pr, float(np.arctan2(b[0] - a[0], b[1] - a[1])), b
        raise ValueError(self.track_type)

    def start_track(self, kind: str) -> None:
        self.track_type, self.track_t, self.wp, self._seg = kind, 0.0, 0, 0.0
        self.mission, self.paper_test, self._paper_qref = "TAKEOFF", None, None
        self.refs["z"] = max(self.refs["z"], 2.5)
        pts, wps = self._path_points(kind)
        self.show_path(pts, wps)

    def start_paper_test(self, name: str) -> None:
        self.paper_test, self.paper_t, self.mission, self.track_type = name, 0.0, "PAPER", None
        self._hide_markers()
        alt0 = cfg.PAPER_TESTS[name]["start_alt"] if name == "flip" else max(2.0, self.read_state()["pos"][2])
        self.refs.update(roll=0.0, pitch=0.0, z=alt0)

    def guidance(self) -> None:
        s = self.read_state()
        gz = cfg.GUIDANCE
        if self.mission in ("IDLE", "DONE", "HOLD"):
            self.refs.update(roll=0.0, pitch=0.0)
            return
        if self.mission == "TAKEOFF":
            self.refs["z"] = min(3.0, self.refs["z"] + 0.8 / self.rate * POLL_EVERY)
            self.refs.update(roll=0.0, pitch=0.0)
            if s["pos"][2] > 2.4:
                self.mission = "TRACK"
            return
        if self.mission == "LAND":
            self.refs["z"] = max(0.15, self.refs["z"] - 0.5 / self.rate * POLL_EVERY)
            self.refs.update(roll=0.0, pitch=0.0)
            if s["pos"][2] < 0.22 and np.linalg.norm(s["vel"]) < 0.5:
                self.mission = "DONE"
                self._report()
            return
        if self.mission == "PAPER":
            spec = cfg.PAPER_TESTS[self.paper_test]
            t = self.paper_t
            self.paper_t += POLL_EVERY / self.rate
            self._paper_qref = reference_quat(
                self.paper_test, min(t, spec["duration"]),
                flip_ramp=FLIP_RAMP_6DOF if self.paper_test == "flip" else None)
            if self.paper_test == "flip":
                tilt = float(np.clip(quat.to_dcm(s["q"])[2, 2], -1, 1))
                self._thrust_override = (spec["idle_fraction"] * MASS * G) if tilt < 0.3 else None
            if t >= spec["duration"]:
                self._thrust_override = self._paper_qref = None
                self.mission = "HOLD"
            return
        self.track_t += POLL_EVERY / self.rate
        ref_p, ref_yaw, _ = self._track_ref(self.track_t)
        self._target_marker(ref_p)
        self.xtrack = float(np.linalg.norm(s["pos"] - ref_p))
        self.track_log.append((self._t_now, s["pos"].copy(), ref_p.copy()))
        if self.track_type == "mission":
            wps = [np.array(w) for w in cfg.PATHS["mission"]]
            b = wps[min(self.wp + 1, len(wps) - 1)]
            seg = np.linalg.norm(b - wps[self.wp])
            self._seg = min(self._seg + gz["v_max"] / self.rate * POLL_EVERY, seg)
            if self._seg >= seg - 0.05:
                self.wp += 1
                self._seg = 0.0
                if self.wp >= len(wps) - 1:
                    self.mission = "LAND"
                    return
                ref_p, ref_yaw, _ = self._track_ref(self.track_t)
        err = ref_p - s["pos"]
        v_des = gz["pos_k"] * err
        v_des[2] = np.clip(v_des[2], -gz["vz_clamp"], gz["vz_clamp"])
        n = np.linalg.norm(v_des[:2])
        if n > gz["v_max"]:
            v_des[:2] *= gz["v_max"] / n
        a = gz["vel_k"] * (v_des - s["vel"])
        a[:2] = np.clip(a[:2], -gz["acc_max"], gz["acc_max"])
        psi = self.refs["yaw"]
        c, sn = np.cos(psi), np.sin(psi)
        self.refs["pitch"] = float(np.clip((a[0] * c + a[1] * sn) / G, -0.4, 0.4))
        self.refs["roll"] = float(np.clip((a[0] * sn - a[1] * c) / G, -0.4, 0.4))
        if np.linalg.norm(err[:2]) > 1.0:
            self.refs["yaw"] = ref_yaw
        self.refs["z"] = float(ref_p[2])

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
        thrust = self.alt.thrust(self.refs["z"], s["pos"][2], s["vel"][2], tilt,
                                 MASS, PAPER_VEHICLE.thrust_max_total, CTRL_EVERY / self.rate)
        if self._thrust_override is not None:
            thrust = self._thrust_override
        thrusts, desat = self.bridge.mix(thrust, tau)
        self.motors = np.array(thrusts)
        fz, tau_a = thrusts_to_wrench_zup(thrusts, PAPER_VEHICLE)
        self._held = {"R": R, "fz": fz, "tau": tau_a, "wind": self._wind()}
        self.telemetry = {"thrust": thrust, "desat": desat,
                          "err_deg": np.degrees(2 * np.arccos(np.clip(abs(float(np.dot(q_ref, q_m))), 0, 1)))}

    def _wind(self) -> np.ndarray:
        b, g = cfg.WIND_LEVELS[self.wind]
        if b == 0:
            return np.zeros(3)
        t = self._t_now
        return MASS * np.array([b * (0.8 + g * (np.sin(t * 0.7) + 0.5 * np.sin(t * 1.7 + 1))),
                                b * (0.3 + g * (np.cos(t * 0.5) + 0.5 * np.sin(t * 1.3 + 3))), 0])

    def step(self) -> None:
        v = cfg.TRAFFIC["speed_m_s"]
        for n, b, y0 in self._traffic:
            a = self.model.jnt_dofadr[self.model.body_jntadr[b]]
            self.data.qpos[a + 1] = -35 + ((y0 + v * self._t_now) % cfg.TRAFFIC["lane_len"])
            self.data.qvel[a + 1] = v if n != "car_m2" else -v
        h = self._held
        self.data.xfrc_applied[self.bid, 0:3] = h["R"] @ np.array([0, 0, h["fz"]]) + h["wind"]
        self.data.xfrc_applied[self.bid, 3:6] = h["R"] @ h["tau"]
        self.mj.mj_step(self.model, self.data)

    @property
    def _t_now(self) -> float:
        return self.data.time

    def check_collisions(self, t: float) -> list[str]:
        mjn = self.mj.mj_id2name
        G_ = self.mj.mjtObj.mjOBJ_GEOM
        hits: set[str] = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            for me, o in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                n1 = mjn(self.model, G_, me)
                if n1 is None or n1 in BENIGN:
                    continue
                n2 = mjn(self.model, G_, o)
                if n1.startswith(DRONE_PARTS):
                    if n2 is None or n2 in BENIGN or n2.startswith(DRONE_PARTS):
                        continue
                    hits.add(n2)
                elif n2 is not None and n2.startswith(DRONE_PARTS):
                    hits.add(n1)
        for h in hits:
            self.collisions.append({"t": round(t, 2), "object": h})
        return sorted(hits)

    def _safety(self, s) -> str:
        if s["pos"][2] > cfg.SAFETY["max_alt"]:
            return "SAFETY: altitude"
        if np.linalg.norm(s["vel"]) > cfg.SAFETY["max_speed"]:
            return "SAFETY: speed"
        if np.degrees(max(abs(s["rpy_sim"][0]), abs(s["rpy_sim"][1]))) > cfg.SAFETY["max_tilt_deg"]:
            return "SAFETY: tilt"
        return ""

    def _log(self, s) -> None:
        self.log_rows.append([
            round(self._t_now, 3), self.mode, self.mission, self.track_type or self.paper_test or "",
            self.wp, round(self.xtrack, 3), *np.round(s["pos"], 3), *np.round(s["vel"], 3),
            *np.round(np.degrees(s["rpy_sim"]), 1), *np.round(s["omg"], 3),
            *np.round(self.motors, 3), round(self.telemetry.get("thrust", 0), 3),
            round(self.telemetry.get("err_deg", 0), 2), self.wind, self.noise, len(self.collisions)])

    def _report(self) -> None:
        if not self.track_log:
            return
        p = np.array([a for _, a, _ in self.track_log])
        r = np.array([b for _, _, b in self.track_log])
        x = np.linalg.norm(p - r, axis=1)
        al = np.abs(p[:, 2] - r[:, 2])
        la = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
        lr = float(np.sum(np.linalg.norm(np.diff(r, axis=0), axis=1)))
        print(f"\n== MISSION METRICS ==\n  duration {self.track_t:6.1f}s   xtrack mean {x.mean():.2f}"
              f" max {x.max():.2f} rms {np.sqrt((x**2).mean()):.2f} m\n  alt err mean {al.mean():.2f} m"
              f"   final {x[-1]:.2f} m\n  path actual {la:.1f} / planned {lr:.1f} m"
              f"  ({100*lr/max(la,1e-6):.0f}%)   collisions {len(self.collisions)}\n")
        LOGDIR.mkdir(exist_ok=True)
        st = time.strftime("%H%M%S")
        with open(LOGDIR / f"mission_{st}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "mode", "mission", "track", "wp", "xtrack", "x", "y", "z",
                        "vx", "vy", "vz", "roll", "pitch", "yaw", "p", "q", "r",
                        "m1", "m2", "m3", "m4", "thrust", "att_err", "wind", "noise", "col"])
            w.writerows(self.log_rows)
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
            ax[0].plot(r[:, 0], r[:, 1], "k--", label="ref")
            ax[0].plot(p[:, 0], p[:, 1], "b-", label="actual")
            ax[0].set_aspect("equal")
            ax[0].legend()
            ax[0].set_title("path XY")
            ax[1].plot(x)
            ax[1].set_title("xtrack [m]")
            ax[2].plot(p[:, 2])
            ax[2].set_title("alt [m]")
            fig.tight_layout()
            fig.savefig(LOGDIR / f"mission_{st}.png", dpi=110)
            plt.close(fig)
            print(f"  logs -> mission_{st}.csv/.png\n")
        except Exception as e:
            print(f"  csv saved (plots skipped: {e})")

    def switch_mode(self) -> None:
        s = self.read_state()
        self.refs.update(roll=float(s["rpy_sim"][0]), pitch=float(s["rpy_sim"][1]),
                         yaw=float(s["rpy_sim"][2]), z=float(s["pos"][2]))
        self._thrust_override = self._paper_qref = None
        self.alt.reset()
        if self.mode == "MANUAL":
            self.mode, self.mission = "OUR CONTROL", "HOLD"
        else:
            self.mode, self.mission = "MANUAL", "IDLE"
            self._hide_markers()

    def land(self) -> None:
        self.refs["z"] = self.read_state()["pos"][2]
        self.mission, self._paper_qref = "LAND", None


def main() -> None:
    import mujoco.viewer

    sim = DroneSim()
    inp = InputManager()
    print(__doc__)
    print(f">>> env: {sim.env} | 3D window launching... Esc quits.\n")

    import keyboard

    keyboard.on_press_key("esc", lambda _: sim.keys.add("__quit__"))

    env_i = 0
    cam = "FREE"
    k, t_wall, fps_t = 0, time.time(), time.time()
    fps = 0
    warn, warn_t = "", -10.0

    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
    try:
        while viewer.is_running() and "__quit__" not in sim.keys:
            held, taps = inp.poll()

            for tap in taps:
                if tap == "space":
                    s = sim.read_state()
                    sim.refs.update(roll=0.0, pitch=0.0, z=float(s["pos"][2]))
                elif tap == "m" and sim.mode != "MANUAL":
                    sim.switch_mode()
                elif tap == "o" and sim.mode != "OUR CONTROL":
                    sim.switch_mode()
                elif tap == "p":
                    sim.paused = not sim.paused
                elif tap == "t":
                    sim.reset()
                elif tap == "e":
                    env_i = (env_i + 1) % len(ENV_LIST)
                    viewer.close()
                    sim = DroneSim(env=ENV_LIST[env_i])
                    inp.reset()
                    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
                    k = 0
                    t_wall = time.time()
                    print(f"\n== ENV -> {sim.env} ==\n")
                elif sim.mode == "OUR CONTROL":
                    if tap == "g":
                        sim.start_track("mission")
                    elif tap == "u":
                        sim.start_track("circle")
                    elif tap == "8":
                        sim.start_track("figure8")
                    elif tap == "v":
                        sim.start_track("straight")
                    elif tap == "6":
                        sim.start_paper_test("step")
                    elif tap == "7":
                        sim.start_paper_test("sine")
                    elif tap == "9":
                        sim.start_paper_test("flip")
                    elif tap == "h":
                        sim.mission, sim.track_type, sim._paper_qref = "HOLD", None, None
                        sim._hide_markers()
                    elif tap == "l":
                        sim.land()
                    elif tap == "k":
                        sim.mission, sim.track_type = "HOLD", None
                        sim._hide_markers()
                elif tap == "n":
                    sim.wind = {"OFF": "LOW", "LOW": "MEDIUM", "MEDIUM": "HIGH", "HIGH": "OFF"}[sim.wind]
                elif tap == "y":
                    sim.noise = {"PERFECT": "LOW", "LOW": "PAPER", "PAPER": "HIGH", "HIGH": "PERFECT"}[sim.noise]
                elif tap == "[":
                    sim.speed = max(0.25, sim.speed / 2)
                elif tap == "]":
                    sim.speed = min(2.0, sim.speed * 2)
                elif tap == "c":
                    cam = {"FREE": "FOLLOW", "FOLLOW": "CHASE", "CHASE": "TOP",
                           "TOP": "OVERVIEW", "OVERVIEW": "FREE"}[cam]
                for num, cm in (("1", "FREE"), ("2", "FOLLOW"), ("3", "CHASE"),
                                ("4", "TOP"), ("5", "OVERVIEW")):
                    if tap == num:
                        cam = cm

            if not sim.paused:
                if k % POLL_EVERY == 0:
                    if sim.mode == "MANUAL":
                        sim.pilot_input(held)
                    else:
                        sim.guidance()
                if k % CTRL_EVERY == 0:
                    sim.control_tick()
                sim.step()
                hits = sim.check_collisions(sim._t_now)
                if hits:
                    warn, warn_t = f"COLLISION: {hits[0]}", sim._t_now
                if k % LOG_EVERY == 0:
                    sim._log(sim.read_state())

            if k % 100 == 0:
                q = sim.data.qpos[sim._qadr:sim._qadr + 3]
                if cam == "FOLLOW":
                    with viewer.lock():
                        viewer.cam.lookat[:] = q
                        viewer.cam.distance = 4.0
                elif cam == "CHASE":
                    yaw = sim.read_state()["rpy_sim"][2]
                    with viewer.lock():
                        viewer.cam.lookat[:] = q
                        viewer.cam.distance = 5.0
                        viewer.cam.azimuth = -np.degrees(yaw) + 180
                        viewer.cam.elevation = -15
                elif cam == "TOP":
                    with viewer.lock():
                        viewer.cam.lookat, viewer.cam.distance = [0, 6, 0], 45.0
                        viewer.cam.elevation, viewer.cam.azimuth = -90, 90
                elif cam == "OVERVIEW":
                    with viewer.lock():
                        viewer.cam.lookat, viewer.cam.distance = [0, 10, 0], 60.0
                        viewer.cam.elevation, viewer.cam.azimuth = -45, 130
                viewer.sync()

            if k % HUD_EVERY == 0:
                s = sim.read_state()
                tel = sim.telemetry
                now = time.time()
                if now - fps_t > 0.5:
                    fps = int(0.5 * HUD_EVERY / max(1e-9, now - fps_t) * 20)
                    fps_t = now
                key_hud = " ".join(f"[{kk.upper()}]" if kk in held else kk.upper()
                                   for kk in ("w", "a", "s", "d", "q", "e", "r", "f"))
                trk = sim.track_type or sim.paper_test or "-"
                rt = "SLOW" if now - t_wall > sim._t_now * sim.speed * 1.5 + 2 else "RT"
                saf = sim._safety(s)
                w = warn if sim._t_now - warn_t < 2 else saf
                print(f"[{sim.mode:<11}|{sim.mission:<7}|{trk:<8}|{cam:<9}|{sim.env:<13}] "
                      f"KEYS {key_hud} | refs r{np.degrees(sim.refs['roll']):+5.1f} "
                      f"p{np.degrees(sim.refs['pitch']):+5.1f} z{sim.refs['z']:4.1f} | "
                      f"M {' '.join(f'{m:4.2f}' for m in sim.motors)} | "
                      f"xyz=({s['pos'][0]:5.1f},{s['pos'][1]:5.1f},{s['pos'][2]:4.1f}) "
                      f"v={np.linalg.norm(s['vel']):4.1f} attE={tel.get('err_deg', 0):5.1f} "
                      f"xtr={sim.xtrack:4.2f} W:{sim.wind[:3]} N:{sim.noise[:4]} "
                      f"x{sim.speed:g} {fps}f{rt}{' PAUSE' if sim.paused else ''}"
                      + (f" !! {w}" if w else ""))

            if k % 1000 == 0 and k > 0 and not sim.paused:
                lag = sim._t_now * sim.speed - (time.time() - t_wall)
                if lag > 0:
                    time.sleep(min(lag, 0.05))
            k += 1
    finally:
        viewer.close()
        inp.unhook()
        if sim.track_log:
            sim._report()
        print("bye")


if __name__ == "__main__":
    main()
