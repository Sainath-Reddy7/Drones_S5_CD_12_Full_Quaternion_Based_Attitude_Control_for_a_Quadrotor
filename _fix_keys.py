"""Fix key collisions + add trail markers. Run from repo root."""
from pathlib import Path

p = Path("sim/interactive/app.py")
s = p.read_text(encoding="utf-8")

# 1. NEW DOCSTRING
old_doc = s[:s.index('"""', 3) + 3]
new_doc = '''"""INTERACTIVE MUJOCO DRONE SIMULATOR -- clean keys, colored paths.

FLIGHT KEYS (held - continuous control, NEVER used for anything else):
    W/S = forward/back    A/D = left/right    Q/E = yaw left/right
    R/F = climb/descend   SPACE = hover (capture position)

MODE COMMANDS (one-shot taps - no overlap with flight keys):
    M     = toggle MANUAL <-> OUR CONTROL
    G     = waypoint mission        U = circle path
    J     = figure-8 path           L = straight line
    1/2/3 = paper STEP/SINE/FLIP (base paper, live)
    H     = hold position           N = land
    X     = abort mission           P = pause/resume
    T     = reset drone

ENVIRONMENT & VISUALS (one-shot - on keys you won't hit while flying):
    TAB   = cycle environment (urban <-> open field)
    C     = cycle camera (free/follow/chase/top/overview)
    [ / ] = simulation speed 0.25x .. 2x
    Y     = sensor-noise cycle     B = wind cycle
    ESC   = quit

PATH COLORS (in the 3D view):
    YELLOW dots = reference path (where the drone SHOULD go)
    BLUE dots   = actual trail (where the drone HAS been)
    GREEN dots  = waypoints       RED ball = current target

HUD shows: live KEY STATE, desired attitude, motor thrusts M1-M4,
position/velocity, tracking error, wind/noise, FPS, collisions.
"""'''
s = s.replace(old_doc, new_doc)

# 2. Remove I/K/J/L trims from pilot_input (collision with one-shots)
old = '''        tgt_p = 0.4 * ("w" in keys) - 0.4 * ("s" in keys)
        tgt_r = 0.4 * ("d" in keys) - 0.4 * ("a" in keys)
        if "i" in keys: tgt_p += 0.25
        if "k" in keys: tgt_p -= 0.25
        if "j" in keys: tgt_r -= 0.25
        if "l" in keys: tgt_r += 0.25
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])'''
new = '''        tgt_p = 0.4 * ("w" in keys) - 0.4 * ("s" in keys)
        tgt_r = 0.4 * ("d" in keys) - 0.4 * ("a" in keys)
        self.refs["pitch"] += 0.35 * (tgt_p - self.refs["pitch"])'''
assert old in s, "pilot_input trim block"
s = s.replace(old, new)

# 3. Trail marker pool
old = '''        return {"dots": [g(f"mk_d{i}") for i in range(96)],
                "wps": [g(f"mk_w{i}") for i in range(24)],
                "target": g("mk_target")}'''
new = '''        return {"dots": [g(f"mk_d{i}") for i in range(96)],
                "wps": [g(f"mk_w{i}") for i in range(24)],
                "target": g("mk_target"),
                "trail": [g(f"mk_t{i}") for i in range(48)]}'''
assert old in s, "marker dict"
s = s.replace(old, new)

old = '''    def _hide_markers(self) -> None:
        for g in (self._mk["dots"] + self._mk["wps"] + [self._mk["target"]]):
            self.model.geom_pos[g] = [0, 0, -10]'''
if old not in s:
    old = old.replace("(", "(").replace(")", ")")
new = '''    def _hide_markers(self) -> None:
        for g in (self._mk["dots"] + self._mk["wps"] + [self._mk["target"]]
                  + self._mk["trail"]):
            self.model.geom_pos[g] = [0, 0, -10]
        self._trail_i = 0'''
assert old in s, "hide markers"
s = s.replace(old, new)

# 4. Trail drop method
old = '''    def _target_marker(self, p) -> None:
        self.model.geom_pos[self._mk["target"]] = p'''
new = '''    def _target_marker(self, p) -> None:
        self.model.geom_pos[self._mk["target"]] = p

    def _drop_trail_marker(self, pos) -> None:
        """Drop a blue trail dot at the drone's actual position (ring buffer)."""
        if not hasattr(self, "_trail_i"):
            self._trail_i = 0
        self.model.geom_pos[self._mk["trail"][self._trail_i]] = pos
        self._trail_i = (self._trail_i + 1) % len(self._mk["trail"])'''
assert old in s, "target marker"
s = s.replace(old, new)

# 5. Drop trail markers during sim (every LOG_EVERY steps)
old = '''                if k % LOG_EVERY == 0:
                    sim._log(sim.read_state())'''
new = '''                if k % LOG_EVERY == 0:
                    sim._log(sim.read_state())
                    sim._drop_trail_marker(sim.read_state()["pos"])'''
assert old in s, "log block"
s = s.replace(old, new)

# 6. Fix the key handler: clean non-colliding layout
old_handler = s[s.index("            for tap in taps:"):s.index("            if not sim.paused:")]
new_handler = '''            for tap in taps:
                # FLIGHT (hold-only keys, no one-shot conflicts)
                if tap == "space":
                    st = sim.read_state()
                    sim.refs.update(roll=0.0, pitch=0.0, z=float(st["pos"][2]))

                # MODE COMMANDS (non-flight keys)
                elif tap == "m":
                    sim.switch_mode()
                elif tap == "p":
                    sim.paused = not sim.paused
                elif tap == "t":
                    sim.reset()
                elif sim.mode == "OUR CONTROL":
                    if tap == "g":
                        sim.start_track("mission")
                    elif tap == "u":
                        sim.start_track("circle")
                    elif tap == "j":
                        sim.start_track("figure8")
                    elif tap == "l":
                        sim.start_track("straight")
                    elif tap == "1":
                        sim.start_paper_test("step")
                    elif tap == "2":
                        sim.start_paper_test("sine")
                    elif tap == "3":
                        sim.start_paper_test("flip")
                    elif tap == "h":
                        sim.mission, sim.track_type, sim._paper_qref = "HOLD", None, None
                        sim._hide_markers()
                    elif tap == "n":
                        sim.land()
                    elif tap == "x":
                        sim.mission, sim.track_type = "HOLD", None
                        sim._hide_markers()

                # ENVIRONMENT & VISUALS (keys you never touch while flying)
                elif tap == "tab":
                    env_i = (env_i + 1) % len(ENV_LIST)
                    viewer.close()
                    sim = DroneSim(env=ENV_LIST[env_i])
                    inp.reset()
                    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
                    k = 0
                    t_wall = time.time()
                    print(f"\\n== ENV -> {sim.env} ==\\n")
                elif tap == "c":
                    cam = {"FREE": "FOLLOW", "FOLLOW": "CHASE", "CHASE": "TOP",
                           "TOP": "OVERVIEW", "OVERVIEW": "FREE"}[cam]
                elif tap == "y":
                    sim.noise = {"PERFECT": "LOW", "LOW": "PAPER",
                                 "PAPER": "HIGH", "HIGH": "PERFECT"}[sim.noise]
                elif tap == "b":
                    sim.wind = {"OFF": "LOW", "LOW": "MEDIUM",
                                "MEDIUM": "HIGH", "HIGH": "OFF"}[sim.wind]
                elif tap == "[":
                    sim.speed = max(0.25, sim.speed / 2)
                elif tap == "]":
                    sim.speed = min(2.0, sim.speed * 2)

'''
s = s.replace(old_handler, new_handler)

p.write_text(s, encoding="utf-8", newline="\n")
print("app.py: key layout fixed + trail markers wired")

# 7. Add trail markers to world_build
p2 = Path("sim/interactive/world_build.py")
s2 = p2.read_text(encoding="utf-8")
if "mk_t0" not in s2:
    # insert after mk_target line
    old_marker_line = "    <geom name=\"mk_target\" type=\"sphere\" size=\"0.22\" pos=\"0 0 -10\" rgba=\"1 0.2 0.2 1\" contype=\"0\" conaffinity=\"0\"/>"
    trail_block = old_marker_line + "\n"
    for i in range(48):
        trail_block += f"    <geom name=\"mk_t{i}\" type=\"sphere\" size=\"0.06\" pos=\"0 0 -10\" rgba=\"0.2 0.5 1 0.85\" contype=\"0\" conaffinity=\"0\"/>\n"
    s2 = s2.replace(old_marker_line + "\n", trail_block)
    p2.write_text(s2, encoding="utf-8", newline="\n")
    print("world_build.py: 48 trail markers added")
else:
    print("world_build.py: trail markers already present")
