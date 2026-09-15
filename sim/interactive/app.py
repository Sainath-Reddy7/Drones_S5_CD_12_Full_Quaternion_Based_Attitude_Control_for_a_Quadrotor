"""INTERACTIVE MUJOCO DRONE SIMULATOR -- pro edition.

One law always flies: the paper's UNMODIFIED quaternion controller (Pq=20,
Pw=4, +/-4 N*m) + priority mixer at 1 kHz. Modes decide only who commands
the reference. Nothing fakes movement: every position change comes from
physics reacting to rotor wrenches; every metric comes from logged state.

MODES
  MANUAL       you fly (keys -> attitude/altitude reference)
  OUR CONTROL  the project's autopilot:
     G  waypoint mission (city tour)        U  circle path
     I  figure-8 path                       V  straight-line path
     X  PAPER STEP test (base paper, live)  Z  PAPER SINE test
     B  PAPER FLIP test (auto-climbs first) H  hold position
     L  land                                K  abort mission
     O  switch to MANUAL, M to switch back

ENVIRONMENT   N cycles wind OFF/LOW/MEDIUM/HIGH | Y cycles sensor noise
              PERFECT/LOW/PAPER/HIGH (paper = the paper's own +/-0.1 model)
SIM           P pause/resume | T reset drone to pad | Esc quit
CAMERAS       1 free | 2 follow | 3 chase | 4 top | 5 overview | C cycle

Everything is logged (50 Hz CSV) and post-mission plots + metrics are
written to sim/interactive/logs/ (path XY reference-vs-actual, tracking
error vs t, altitude vs t). Collisions with named objects are shown in the
HUD and logged. Wind and noise physically/sensorially affect the loop.
Safety limits (altitude/speed/tilt) warn in the HUD.

Launch:  python -m sim.interactive.app      (deps: mujoco, keyboard)
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

XML = Path(__file__).with_name("world_city.xml")
LOGDIR = Path(__file__).parent / "logs"

MASS = cfg.DRONE["mass_kg"]
CTRL_EVERY = cfg.CONTROLLER["physics_hz"] // cfg.CONTROLLER["control_hz"]  # 10
POLL_EVERY = 100   # 100 Hz input/guidance
HUD_EVERY = 40     # ~25 Hz HUD
LOG_EVERY = 200    # 50 Hz telemetry
G = 9.81

BENIGN = {"floor", "pad_base"}
DRONE_PARTS = ("prop", "mount", "arm", "shell", "plate", "cam", "lens", "led", "frame")
SPAWN = np.array([0.0, -6.0, 1.2])


def _euler_refs(q) -> tuple[float, float, float]:
    return tuple(float(x) for x in quat.to_euler(q))


class DroneSim:
    """Core simulation -- viewer/keyboard-free so the tests can drive it."""

    def __init__(self, seed: int = 0):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(XML))
        self.data = mujoco.MjData(self.model)
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.dt = float(self.model.opt.timestep)
        self.rate = 1.0 / self.dt
        self.rng = np.random.default_rng(seed)

        self.bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=True)
        self.alt = AltitudeHold(max_thrust_factor=2.2)

        self.mode = "MANUAL"            # MANUAL | OUR CONTROL
        self.mission = "IDLE"           # IDLE/HOLD/TAKEOFF/TRACK/PAPER/LAND/DONE
        self.paused = False
        self.track_type = None          # mission/circle/figure8/straight
        self.track_t = 0.0
        self.wp = 0
        self._seg_progress = 0.0
        self.paper_test = None
        self.paper_t = 0.0
        self.wind = "OFF"
        self.noise = "PAPER"
        self.refs = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "z": SPAWN[2]}
        self.keys: set = set()
        self.collisions: list[dict] = []
        self.telemetry: dict = {}
        self.log_rows: list[list] = []
        self.track_log: list[tuple] = []   # (t, pos, ref_pos) for metrics
        self.xtrack = 0.0
        self.warn = ""
        self._held = {"R": np.eye(3), "fz": MASS * G, "tau": np.zeros(3)}
        self._thrust_override = None
        self._paper_qref = None

        # qpos/qvel base addresses of the drone's free joint (the moving
        # traffic bodies share the state vector -> never write qpos[:7])
        self._qadr = self.model.jnt_qposadr[self.model.body_jntadr[self.bid]]
        self._vadr = self.model.jnt_dofadr[self.model.body_jntadr[self.bid]]

        self._mk = self._collect_markers()
        self._traffic_ids = self._collect_traffic()
        self.reset()

    # ------------------------------------------------------------- markers
    def _collect_markers(self) -> dict:
        g = lambda n: self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_GEOM, n)
        dots = [g(f"mk_d{i}") for i in range(96)]
        wps = [g(f"mk_w{i}") for i in range(24)]
        return {"dots": dots, "wps": wps, "target": g("mk_target"), "shown": 0}

    def _collect_traffic(self) -> list:
        ids = []
        for n in ("car_m1", "car_m2", "truck_m1"):
            bid = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, n)
            qadr = self.model.jnt_dofadr[self.model.body_jntadr[bid]]
            ids.append((n, bid, float(self.data.qpos[qadr + 1])))
        return ids  # (name, body_id, y0)

    def _hide_markers(self) -> None:
        for gid in self._mk["dots"] + self._mk["wps"] + [self._mk["target"]]:
            self.model.geom_pos[gid] = [0, 0, -10]
        self._mk["shown"] = 0

    def show_path(self, pts: np.ndarray, waypoints: np.ndarray | None = None) -> None:
        """Move the marker pool to visualize the reference path (visual only)."""
        self._hide_markers()
        n = min(len(pts), len(self._mk["dots"]))
        for i in range(n):
            self.model.geom_pos[self._mk["dots"][i]] = pts[i]
        if waypoints is not None:
            for i, w in enumerate(waypoints[: len(self._mk["wps"])]):
                self.model.geom_pos[self._mk["wps"][i]] = w
        self._mk["shown"] = n

    def move_target_marker(self, p) -> None:
        self.model.geom_pos[self._mk["target"]] = p

    # ------------------------------------------------------------ lifecycle
    def set_drone(self, pos, quat=(1, 0, 0, 0), vel=(0, 0, 0)) -> None:
        self.data.qpos[self._qadr:self._qadr + 7] = [*pos, *quat]
        self.data.qvel[self._vadr:self._vadr + 6] = [*vel, 0, 0, 0]

    def reset(self) -> None:
        self.set_drone(SPAWN)
        self.data.qvel[self._vadr:self._vadr + 6] = 0
        for name, bid, y0 in self._traffic_ids:
            qadr = self.model.jnt_dofadr[self.model.body_jntadr[bid]]
            self.data.qpos[qadr + 1] = y0
        self.refs.update(roll=0.0, pitch=0.0, yaw=0.0, z=SPAWN[2])
        self.alt.reset()
        self.mode, self.mission = "MANUAL", "IDLE"
        self.track_type = None
        self.paper_test = None
        self._thrust_override = None
        self._paper_qref = None
        self._hide_markers()

    # ---------------------------------------------------------------- state
    def read_state(self, noisy: bool = False) -> dict:
        q = np.array(self.data.qpos[self._qadr + 3:self._qadr + 7])
        pos = np.array(self.data.qpos[self._qadr:self._qadr + 3])
        vel = np.array(self.data.qvel[self._vadr:self._vadr + 3])
        omg = np.array(self.data.qvel[self._vadr + 3:self._vadr + 6])
        amp = cfg.NOISE_LEVELS[self.noise]
        if noisy and amp > 0:
            q = quat.normalize(q + self.rng.uniform(-amp, amp, 4))
            omg = omg + self.rng.uniform(-amp, amp, 3)
        return {"q": q, "pos": pos, "vel": vel, "omg": omg,
                "rpy": _euler_refs(quat.conj(q)),     # paper frame
                "rpy_sim": _euler_refs(q)}            # sim frame (the viewer's)

    # ------------------------------------------------------------- MANUAL
    def pilot_input(self, keys: set) -> None:
        gz = cfg.GUIDANCE
        tgt_p = 0.4 * ("w" in keys) - 0.4 * ("s" in keys)
        tgt_r = 0.4 * ("d" in keys) - 0.4 * ("a" in keys)
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])
        self.refs["roll"] += 0.35 * (tgt_r - self.refs["roll"])
        self.refs["roll"] = float(np.clip(self.refs["roll"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        self.refs["pitch"] = float(np.clip(self.refs["pitch"], -gz["max_tilt_rad"], gz["max_tilt_rad"]))
        if "q" in keys:
            self.refs["yaw"] += gz["yaw_rate"] / self.rate * POLL_EVERY
        if "e" in keys:
            self.refs["yaw"] -= gz["yaw_rate"] / self.rate * POLL_EVERY
        up = ("up" in keys) or ("r" in keys)
        dn = ("down" in keys) or ("f" in keys)
        self.refs["z"] = float(np.clip(
            self.refs["z"] + (up - dn) * gz["climb"] / self.rate * POLL_EVERY,
            cfg.SAFETY["min_alt"], cfg.SAFETY["max_alt"]))

    # ------------------------------------------------------ trajectory refs
    def _path_points(self, kind: str):
        """Dense reference-path points for visualization."""
        if kind == "mission":
            wps = [np.array(w) for w in cfg.PATHS["mission"]]
            pts = []
            for a, b in zip(wps, wps[1:]):
                n = max(2, int(np.linalg.norm(b - a) * 3))
                pts += [a + (b - a) * i / n for i in range(n)]
            pts.append(wps[-1])
            return np.array(pts), wps
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

    def _track_ref(self, t: float):
        """Reference position/yaw along the active trajectory at time t."""
        if self.track_type == "mission":
            wps = [np.array(w) for w in cfg.PATHS["mission"]]
            i = min(self.wp, len(wps) - 2)
            a, b = wps[i], wps[i + 1]
            seg = np.linalg.norm(b - a)
            s = min(self._seg_progress, seg)
            p = a + (b - a) * (s / max(seg, 1e-6))
            return p, float(np.arctan2(b[0] - a[0], b[1] - a[1])), b
        if self.track_type == "circle":
            c, r, w = cfg.PATHS["circle"]["center"], cfg.PATHS["circle"]["radius"], cfg.PATHS["circle"]["omega"]
            th = w * t
            p = np.array([c[0] + r * np.sin(th), c[1] + r * np.cos(th), c[2]])
            return p, float(th + np.pi), p
        if self.track_type == "figure8":
            p = cfg.PATHS["figure8"]
            th = p["omega"] * t
            pos = np.array([p["center"][0] + p["a"] * np.sin(th),
                            p["center"][1] + p["b"] * np.sin(th) * np.cos(th) * 2,
                            p["center"][2]])
            d = np.array([p["a"] * np.cos(th),
                          p["b"] * 2 * (np.cos(th) ** 2 - np.sin(th) ** 2), 0])
            return pos, float(np.arctan2(d[0], d[1])), pos
        if self.track_type == "straight":
            p = cfg.PATHS["straight"]
            a, b = np.array(p["start"]), np.array(p["end"])
            L = np.linalg.norm(b - a)
            s = min(p["speed"] * t, L)
            pr = a + (b - a) * (s / L)
            return pr, float(np.arctan2(b[0] - a[0], b[1] - a[1])), b
        raise ValueError(self.track_type)

    # ---------------------------------------------------------- OUR CONTROL
    def start_track(self, kind: str) -> None:
        self.track_type = kind
        self.track_t = 0.0
        self.wp = 0
        self._seg_progress = 0.0
        self.mission = "TAKEOFF"
        self.paper_test = None
        self.refs["z"] = max(self.refs["z"], 2.5)
        pts, wps = self._path_points(kind)
        self.show_path(pts, wps)
        print(f"\n== OUR CONTROL: {kind.upper()} path tracking armed ==\n")

    def start_paper_test(self, name: str) -> None:
        self.paper_test = name
        self.paper_t = 0.0
        self.mission = "PAPER"
        self.track_type = None
        self._hide_markers()
        alt0 = cfg.PAPER_TESTS[name]["start_alt"] if name == "flip" else max(2.0, self.read_state()["pos"][2])
        self.refs.update(roll=0.0, pitch=0.0, z=alt0)
        print(f"\n== OUR CONTROL: base-paper {name.upper()} test "
              f"(reference generator + frozen P^2 law, live) ==\n")

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
                self._mission_report()
            return
        if self.mission == "PAPER":
            self._paper_guidance(s)
            return
        # TRACK ------------------------------------------------------------
        self.track_t += POLL_EVERY / self.rate
        ref_p, ref_yaw, goal = self._track_ref(self.track_t)
        self.move_target_marker(ref_p)
        self.xtrack = float(np.linalg.norm(s["pos"] - ref_p))
        self.track_log.append((self._t_now, s["pos"].copy(), ref_p.copy()))

        if self.track_type == "mission":
            wps = [np.array(w) for w in cfg.PATHS["mission"]]
            b = wps[min(self.wp + 1, len(wps) - 1)]
            seg_len = np.linalg.norm(b - wps[self.wp])
            self._seg_progress = min(self._seg_progress + gz["v_max"] / self.rate * POLL_EVERY, seg_len)
            if self._seg_progress >= seg_len - 0.05:
                self.wp += 1
                self._seg_progress = 0.0
                if self.wp >= len(wps) - 1:
                    self.mission = "LAND"
                    return
                ref_p, ref_yaw, goal = self._track_ref(self.track_t)

        err = ref_p - s["pos"]
        v_des = gz["pos_k"] * err
        v_des[2] = np.clip(v_des[2], -gz["vz_clamp"], gz["vz_clamp"])
        n = np.linalg.norm(v_des[:2])
        if n > gz["v_max"]:
            v_des[:2] *= gz["v_max"] / n
        a_cmd = gz["vel_k"] * (v_des - s["vel"])
        a_cmd[:2] = np.clip(a_cmd[:2], -gz["acc_max"], gz["acc_max"])
        # SIM-frame tilt from world accel at current yaw (derived from
        # R = Rz(psi)Ry(theta)Rx(phi): thrust_xy/mg = (th c+ph s, th s-ph c))
        psi = self.refs["yaw"]
        c, ps = np.cos(psi), np.sin(psi)
        self.refs["pitch"] = float(np.clip((a_cmd[0] * c + a_cmd[1] * ps) / G, -0.4, 0.4))
        self.refs["roll"] = float(np.clip((a_cmd[0] * ps - a_cmd[1] * c) / G, -0.4, 0.4))
        if np.linalg.norm(err[:2]) > 1.0:
            self.refs["yaw"] = ref_yaw
        self.refs["z"] = float(ref_p[2])

    def _paper_guidance(self, s: dict) -> None:
        """THE BASE-PAPER PROTOCOL, live: the paper's own reference
        generators (step/sine/flip) drive the frozen law in the simulator."""
        spec = cfg.PAPER_TESTS[self.paper_test]
        t = self.paper_t
        self.paper_t += POLL_EVERY / self.rate
        ramp = FLIP_RAMP_6DOF if self.paper_test == "flip" else None
        self._paper_qref = reference_quat(
            self.paper_test, min(t, spec["duration"]), flip_ramp=ramp)  # paper convention

        if self.paper_test == "flip":
            tilt = float(np.clip(quat.to_dcm(s["q"])[2, 2], -1.0, 1.0))
            self._thrust_override = (spec["idle_fraction"] * MASS * G) if tilt < 0.3 else None
        if t >= spec["duration"]:
            self._thrust_override = None
            self._paper_qref = None
            self.mission = "HOLD"
            self._paper_report()

    # ------------------------------------------------------------ controller
    def control_tick(self) -> None:
        s = self.read_state(noisy=True)  # sensor model per noise level
        if self.mission == "PAPER" and self._paper_qref is not None:
            q_ref = self._paper_qref                     # paper convention as-is
        else:
            # refs are SIM-convention (what the viewer shows); the controller
            # speaks paper -- the conjugate bridge, same as every adapter
            q_ref = quat.conj(quat.from_euler(self.refs["roll"], self.refs["pitch"], self.refs["yaw"]))
        q_m = quat.conj(s["q"])
        tau, _ = self.bridge.torque(q_ref, q_m, s["omg"])
        R = quat.to_dcm(s["q"])
        tilt = float(np.clip(R[2, 2], -1.0, 1.0))
        thrust = self.alt.thrust(
            self.refs["z"], s["pos"][2], s["vel"][2], tilt, MASS,
            PAPER_VEHICLE.thrust_max_total, CTRL_EVERY / self.rate,
        )
        if self._thrust_override is not None:
            thrust = self._thrust_override
        thrusts, desat = self.bridge.mix(thrust, tau)
        fz, tau_a = thrusts_to_wrench_zup(thrusts, PAPER_VEHICLE)

        self._held = {"R": R, "fz": fz, "tau": tau_a, "wind": self._wind_force()}
        self.telemetry = {
            "tau": tau, "thrust": thrust, "desat": desat,
            "err_deg": np.degrees(2 * np.arccos(np.clip(abs(float(np.dot(q_ref, q_m))), 0, 1))),
        }

    def _wind_force(self) -> np.ndarray:
        base, gust = cfg.WIND_LEVELS[self.wind]
        if base == 0:
            return np.zeros(3)
        t = self._t_now
        g1 = np.sin(t * 0.7) + 0.5 * np.sin(t * 1.7 + 1.0)
        g2 = np.cos(t * 0.5) + 0.5 * np.sin(t * 1.3 + 3.0)
        return MASS * np.array([base * (0.8 + gust * g1), base * (0.3 + gust * g2), 0.0])

    # -------------------------------------------------------------- physics
    def step(self) -> None:
        self._traffic_step()
        h = self._held
        self.data.xfrc_applied[self.bid, 0:3] = h["R"] @ np.array([0.0, 0.0, h["fz"]]) + h.get("wind", np.zeros(3))
        self.data.xfrc_applied[self.bid, 3:6] = h["R"] @ h["tau"]
        self.mj.mj_step(self.model, self.data)

    def _traffic_step(self) -> None:
        v = cfg.TRAFFIC["speed_m_s"]
        for name, bid, y0 in self._traffic_ids:
            qadr = self.model.jnt_dofadr[self.model.body_jntadr[bid]]
            y = -35 + ((y0 + v * self._t_now) % cfg.TRAFFIC["lane_len"])
            self.data.qpos[qadr + 1] = y
            self.data.qvel[qadr + 1] = v if name != "car_m2" else -v

    @property
    def _t_now(self) -> float:
        return self.data.time

    def check_collisions(self, t: float) -> list[str]:
        mjn = self.mj.mj_id2name
        geom = self.mj.mjtObj.mjOBJ_GEOM
        hits: set[str] = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            for me, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                name = mjn(self.model, geom, me)
                if name is None or name in BENIGN:
                    continue
                if name.startswith(DRONE_PARTS):
                    on = mjn(self.model, geom, other)
                    if on is None or on in BENIGN or on.startswith(DRONE_PARTS):
                        continue
                    hits.add(on)
                else:
                    on = mjn(self.model, geom, other)
                    if on is not None and on.startswith(DRONE_PARTS):
                        hits.add(name)
        for h in hits:
            self.collisions.append({"t": round(t, 2), "object": h})
        return sorted(hits)

    def _safety_check(self, s: dict) -> str:
        sp = np.linalg.norm(s["vel"])
        tilt_deg = np.degrees(max(abs(s["rpy"][0]), abs(s["rpy"][1])))
        if s["pos"][2] > cfg.SAFETY["max_alt"]:
            return "SAFETY: max altitude"
        if sp > cfg.SAFETY["max_speed"]:
            return f"SAFETY: speed {sp:.1f} m/s"
        if tilt_deg > cfg.SAFETY["max_tilt_deg"]:
            return f"SAFETY: tilt {tilt_deg:.0f} deg"
        return ""

    # --------------------------------------------------------------- logging
    def _log_row(self, s: dict) -> None:
        tel = self.telemetry
        self.log_rows.append([
            round(self._t_now, 3), self.mode, self.mission, self.track_type or self.paper_test or "",
            self.wp, round(self.xtrack, 3),
            *np.round(s["pos"], 3), *np.round(s["vel"], 3),
            *np.round(np.degrees(s["rpy_sim"]), 1), *np.round(s["omg"], 3),
            round(tel.get("thrust", 0.0), 3), round(tel.get("err_deg", 0.0), 2),
            self.wind, self.noise, len(self.collisions),
        ])

    def _mission_report(self) -> None:
        if not self.track_log:
            return
        arr_p = np.array([p for _, p, _ in self.track_log])
        arr_r = np.array([r for _, _, r in self.track_log])
        xtr = np.linalg.norm(arr_p - arr_r, axis=1)
        actual_len = float(np.sum(np.linalg.norm(np.diff(arr_p, axis=0), axis=1)))
        planned_len = float(np.sum(np.linalg.norm(np.diff(arr_r, axis=0), axis=1)))
        print(
            f"\n== MISSION METRICS ==\n"
            f"  duration        {self.track_t:6.1f} s\n"
            f"  mean xtrack     {xtr.mean():6.2f} m   max {xtr.max():.2f} m\n"
            f"  final error     {xtr[-1]:6.2f} m\n"
            f"  path length     actual {actual_len:.1f} m / planned {planned_len:.1f} m"
            f"  (efficiency {100 * planned_len / max(actual_len, 1e-6):.0f}%)\n"
            f"  collisions      {len(self.collisions)}\n")
        self._write_logs(arr_p, arr_r, xtr)

    def _paper_report(self) -> None:
        print(f"\n== PAPER {self.paper_test.upper()} TEST DONE ==  "
              f"final att. err {self.telemetry.get('err_deg', 0):.1f} deg "
              f"(noise={self.noise}, controller frozen)\n")

    def _write_logs(self, arr_p, arr_r, xtr) -> None:
        LOGDIR.mkdir(exist_ok=True)
        stamp = time.strftime("%H%M%S")
        csv_path = LOGDIR / f"mission_{stamp}.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "mode", "mission", "track", "wp", "xtrack",
                        "x", "y", "z", "vx", "vy", "vz", "roll_deg", "pitch_deg",
                        "yaw_deg", "p", "q", "r", "thrust", "att_err_deg",
                        "wind", "noise", "collisions"])
            w.writerows(self.log_rows)
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
            ax[0].plot(arr_r[:, 0], arr_r[:, 1], "k--", label="reference")
            ax[0].plot(arr_p[:, 0], arr_p[:, 1], "b-", label="actual")
            ax[0].set_aspect("equal"); ax[0].legend(); ax[0].set_title("path (XY)")
            ax[1].plot(xtr); ax[1].set_title("cross-track error [m]")
            ax[2].plot(arr_p[:, 2]); ax[2].set_title("altitude [m]")
            fig.tight_layout()
            png = LOGDIR / f"mission_{stamp}.png"
            fig.savefig(png, dpi=110)
            plt.close(fig)
            print(f"  logs -> {csv_path.name} + {png.name}\n")
        except Exception as e:  # plots optional
            print(f"  csv -> {csv_path.name} (plots skipped: {e})")

    # -------------------------------------------------------------- switch
    def switch_mode(self) -> None:
        s = self.read_state()
        self.refs["roll"], self.refs["pitch"] = float(s["rpy_sim"][0]), float(s["rpy_sim"][1])
        self.refs["yaw"] = float(s["rpy_sim"][2])
        self.refs["z"] = float(s["pos"][2])
        self._thrust_override = None
        self._paper_qref = None
        self.alt.reset()
        if self.mode == "MANUAL":
            self.mode, self.mission = "OUR CONTROL", "HOLD"
        else:
            self.mode, self.mission = "MANUAL", "IDLE"
            self._hide_markers()
        print(f"\n== MODE -> {self.mode} (safe handoff from current state) ==\n")

    def mission_cmd(self, act: str) -> None:
        s = self.read_state()
        self.refs["z"] = float(s["pos"][2])
        if act == "GO":
            self.start_track("mission")
        elif act == "HOLD":
            self.mission, self.track_type = "HOLD", None
            self._paper_qref = None
            self._hide_markers()
        elif act == "LAND":
            self.mission = "LAND"
        elif act == "ABORT":
            self.mission, self.track_type = "HOLD", None
            self._paper_qref = None
            self._hide_markers()
            print("== mission aborted ==\n")

    # ----------------------------------------------------------------- run
    def run(self) -> None:
        import mujoco.viewer
        import keyboard

        print(__doc__)
        print(">>> 3D window launching... Esc quits.\n")
        keyboard.on_press_key("esc", lambda _: self.keys.add("__quit__"))

        k = 0
        t_wall = time.time()
        cam_mode = "FREE"
        warn_age = -10.0
        slow_warned = False
        fps, t_fps = 0, t_wall

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and "__quit__" not in self.keys:
                if k % POLL_EVERY == 0:
                    ks = set()
                    for key in ("w", "a", "s", "d", "q", "e", "r", "f", "up", "down"):
                        if keyboard.is_pressed(key):
                            ks.add(key)
                    if keyboard.is_pressed("space"):
                        self.refs.update(roll=0.0, pitch=0.0)
                    one_shot = [
                        ("m", lambda: self.mode == "MANUAL" and self.switch_mode()),
                        ("o", lambda: self.mode == "OUR CONTROL" and self.switch_mode()),
                        ("p", lambda: setattr(self, "paused", not self.paused)),
                        ("t", self.reset),
                        ("g", lambda: self.mode == "OUR CONTROL" and self.mission_cmd("GO")),
                        ("h", lambda: self.mode == "OUR CONTROL" and self.mission_cmd("HOLD")),
                        ("l", lambda: self.mode == "OUR CONTROL" and self.mission_cmd("LAND")),
                        ("k", lambda: self.mode == "OUR CONTROL" and self.mission_cmd("ABORT")),
                        ("u", lambda: self.mode == "OUR CONTROL" and self.start_track("circle")),
                        ("i", lambda: self.mode == "OUR CONTROL" and self.start_track("figure8")),
                        ("v", lambda: self.mode == "OUR CONTROL" and self.start_track("straight")),
                        ("x", lambda: self.mode == "OUR CONTROL" and self.start_paper_test("step")),
                        ("z", lambda: self.mode == "OUR CONTROL" and self.start_paper_test("sine")),
                        ("b", lambda: self.mode == "OUR CONTROL" and self.start_paper_test("flip")),
                        ("n", lambda: setattr(self, "wind",
                            {"OFF": "LOW", "LOW": "MEDIUM", "MEDIUM": "HIGH", "HIGH": "OFF"}[self.wind])),
                        ("y", lambda: setattr(self, "noise",
                            {"PERFECT": "LOW", "LOW": "PAPER", "PAPER": "HIGH", "HIGH": "PERFECT"}[self.noise])),
                    ]
                    for key, fn in one_shot:
                        if keyboard.is_pressed(key):
                            try:
                                fn()
                            except Exception as e:
                                print("cmd error:", e)
                            time.sleep(0.15)
                    if keyboard.is_pressed("c"):
                        cam_mode = {"FREE": "FOLLOW", "FOLLOW": "CHASE", "CHASE": "TOP",
                                    "TOP": "OVERVIEW", "OVERVIEW": "FREE"}[cam_mode]
                        time.sleep(0.15)
                    for num, cm in (("1", "FREE"), ("2", "FOLLOW"), ("3", "CHASE"),
                                    ("4", "TOP"), ("5", "OVERVIEW")):
                        if keyboard.is_pressed(num):
                            cam_mode = cm

                    if not self.paused:
                        if self.mode == "MANUAL":
                            self.pilot_input(ks)
                        else:
                            self.guidance()

                if not self.paused:
                    if k % CTRL_EVERY == 0:
                        self.control_tick()
                    self.step()
                    hits = self.check_collisions(self._t_now)
                    if hits:
                        self.warn = f"COLLISION: {hits[0]}"
                        warn_age = self._t_now

                if k % LOG_EVERY == 0:
                    self._log_row(self.read_state())

                # render + cameras
                if k % 100 == 0:
                    if cam_mode == "FOLLOW":
                        with viewer.lock():
                            viewer.cam.lookat[:] = self.data.qpos[0:3]
                            viewer.cam.distance = 4.0
                    elif cam_mode == "CHASE":
                        yaw = self.read_state()["rpy"][2]
                        with viewer.lock():
                            viewer.cam.lookat[:] = self.data.qpos[0:3]
                            viewer.cam.distance = 5.0
                            viewer.cam.azimuth = -np.degrees(yaw) + 180
                            viewer.cam.elevation = -15
                    elif cam_mode == "TOP":
                        with viewer.lock():
                            viewer.cam.lookat[:] = [0, 6, 0]
                            viewer.cam.distance = 45.0
                            viewer.cam.elevation = -90
                            viewer.cam.azimuth = 90
                    elif cam_mode == "OVERVIEW":
                        with viewer.lock():
                            viewer.cam.lookat[:] = [0, 10, 0]
                            viewer.cam.distance = 60.0
                            viewer.cam.elevation = -45
                            viewer.cam.azimuth = 130
                    viewer.sync()

                # HUD
                if k % HUD_EVERY == 0:
                    s = self.read_state()
                    tel = self.telemetry
                    safety = self._safety_check(s)
                    now = time.time()
                    if now - t_fps > 0.5:
                        fps = int((k / max(1e-9, now - t_wall)) % 10000)
                        t_fps = now
                    rt = "SLOW" if now - t_wall > self._t_now * 1.5 + 2 else "RT"
                    track = self.track_type or self.paper_test or "-"
                    w = self.warn if self._t_now - warn_age < 2 else (safety or "")
                    print(
                        f"[{self.mode:<11}|{self.mission:<7}|{track:<8}] "
                        f"xyz=({s['pos'][0]:5.1f},{s['pos'][1]:5.1f},{s['pos'][2]:4.1f}) "
                        f"rpy=({np.degrees(s['rpy_sim'][0]):+5.1f},{np.degrees(s['rpy_sim'][1]):+5.1f},{np.degrees(s['rpy_sim'][2]):+6.1f}) "
                        f"v={np.linalg.norm(s['vel']):4.1f} thr={tel.get('thrust', 0):4.2f} "
                        f"xtr={self.xtrack:4.2f} attE={tel.get('err_deg', 0):5.1f} "
                        f"wind={self.wind:<6} noise={self.noise:<7} {rt}"
                        f"{' PAUSED' if self.paused else ''}"
                        + (f"  !! {w}" if w else ""))

                # pacing
                if k % 1000 == 0 and k > 0 and not self.paused:
                    delta = self._t_now - (time.time() - t_wall)
                    if delta > 0:
                        time.sleep(min(delta, 0.05))
                    elif delta < -3 and not slow_warned:
                        print("  (below real-time; sim continues slower)")
                        slow_warned = True
                k += 1

        if self.track_log:
            self._mission_report()
        elif self.log_rows:
            LOGDIR.mkdir(exist_ok=True)
            p = LOGDIR / f"session_{time.strftime('%H%M%S')}.csv"
            with open(p, "w", newline="") as f:
                csv.writer(f).writerows(self.log_rows)
            print(f"\nlog -> {p} | collisions: {len(self.collisions)}")
        print("bye")


def main() -> None:
    DroneSim().run()


if __name__ == "__main__":
    main()
