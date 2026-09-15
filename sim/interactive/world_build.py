"""Builds the versioned environment XMLs under sim/interactive/envs/.

Each environment is a standalone directory with its own world file; loading
one never touches the others (spec: env-version isolation). Every world
MUST contain the named elements the app binds to:

  body "quadrotor" (the exact paper drone)
  bodies car_m1 / car_m2 / truck_m1          (kinematic traffic)
  marker pool mk_d0..95, mk_w0..23, mk_target (path visualization)

Run:  python -m sim.interactive.world_build   (regenerates all env XMLs)
"""
from __future__ import annotations

from pathlib import Path

from sim.interactive.vehicles import car_xml, truck_xml

ENVS = Path(__file__).parent / "envs"

HEADER = """<mujoco model="{name}">
  <option timestep="1e-4" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6" specular="0.12 0.12 0.12"/>
    <global offwidth="1920" offheight="1080"/>
    <map znear="0.01" zfar="120"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.5 0.6 0.78" rgb2="0.92 0.93 0.96" width="256" height="256"/>
    <texture name="groundtex" type="2d" builtin="checker" rgb1="{gc1}" rgb2="{gc2}" width="200" height="200"/>
    <material name="groundmat" texture="groundtex" texrepeat="14 14" specular="0.05"/>
  </asset>
  <worldbody>
    <light directional="true" castshadow="true" pos="6 6 14" dir="0 0 -1"/>
    <light directional="true" pos="-8 -6 10" dir="0.3 0.2 -1" diffuse="0.25 0.27 0.3"/>
    <geom name="floor" type="plane" size="60 60 0.1" material="groundmat" rgba="0.7 0.75 0.7 1" friction="1.0 0.5 0.5"/>
"""

DRONE = """
    <!-- the exact paper drone (mass/inertia = benchmark values) -->
    <body name="quadrotor" pos="0 -6 1.2">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="0.2" diaginertia="6.5e-4 6.5e-4 1.2e-3"/>
      <geom name="frame" type="box" size="0.17 0.17 0.015" group="3" friction="0.8 0.5 0.5"/>
      <geom name="shell" type="ellipsoid" size="0.085 0.085 0.028" pos="0 0 0.018" rgba="0.22 0.24 0.28 1" contype="0" conaffinity="0"/>
      <geom name="plate" type="cylinder" size="0.09 0.008" pos="0 0 -0.004" rgba="0.15 0.16 0.18 1" contype="0" conaffinity="0"/>
      <geom name="arm1" type="box" size="0.075 0.007 0.005" euler="0 0 45" pos="0.053033 0.053033 0" rgba="0.28 0.3 0.34 1" contype="0" conaffinity="0"/>
      <geom name="arm2" type="box" size="0.075 0.007 0.005" euler="0 0 -45" pos="-0.053033 -0.053033 0" rgba="0.28 0.3 0.34 1" contype="0" conaffinity="0"/>
      <geom name="arm3" type="box" size="0.075 0.007 0.005" euler="0 0 -45" pos="0.053033 -0.053033 0" rgba="0.28 0.3 0.34 1" contype="0" conaffinity="0"/>
      <geom name="arm4" type="box" size="0.075 0.007 0.005" euler="0 0 45" pos="-0.053033 0.053033 0" rgba="0.28 0.3 0.34 1" contype="0" conaffinity="0"/>
      <geom name="cam" type="box" size="0.014 0.01 0.008" pos="0.098 0 0.02" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
      <geom name="lens" type="cylinder" size="0.004 0.006" euler="0 90 0" pos="0.113 0 0.02" rgba="0.05 0.08 0.12 1" contype="0" conaffinity="0"/>
      <geom name="led" type="sphere" size="0.006" pos="-0.095 0 0.02" rgba="1 0.1 0.1 1" contype="0" conaffinity="0"/>
      <geom name="mount1" type="cylinder" size="0.011 0.012" pos="0.106066 0.106066 0.014" rgba="0.18 0.18 0.2 1" contype="0" conaffinity="0"/>
      <geom name="mount2" type="cylinder" size="0.011 0.012" pos="-0.106066 -0.106066 0.014" rgba="0.18 0.18 0.2 1" contype="0" conaffinity="0"/>
      <geom name="mount3" type="cylinder" size="0.011 0.012" pos="0.106066 -0.106066 0.014" rgba="0.18 0.18 0.2 1" contype="0" conaffinity="0"/>
      <geom name="mount4" type="cylinder" size="0.011 0.012" pos="-0.106066 0.106066 0.014" rgba="0.18 0.18 0.2 1" contype="0" conaffinity="0"/>
      <geom name="prop1" type="cylinder" size="0.045 0.0012" pos="0.106066 0.106066 0.03" rgba="0.9 0.2 0.2 0.32" contype="0" conaffinity="0"/>
      <geom name="prop2" type="cylinder" size="0.045 0.0012" pos="-0.106066 -0.106066 0.03" rgba="0.2 0.25 0.9 0.32" contype="0" conaffinity="0"/>
      <geom name="prop3" type="cylinder" size="0.045 0.0012" pos="0.106066 -0.106066 0.03" rgba="0.2 0.8 0.25 0.32" contype="0" conaffinity="0"/>
      <geom name="prop4" type="cylinder" size="0.045 0.0012" pos="-0.106066 0.106066 0.03" rgba="0.95 0.85 0.2 0.32" contype="0" conaffinity="0"/>
    </body>
"""

MARKERS = """
    <!-- path visualization pool (moved by the app; visual only) -->
"""
for i in range(96):
    MARKERS += f'    <geom name="mk_d{i}" type="sphere" size="0.07" pos="0 0 -10" rgba="1 0.85 0.2 0.9" contype="0" conaffinity="0"/>\n'
for i in range(24):
    MARKERS += f'    <geom name="mk_w{i}" type="sphere" size="0.16" pos="0 0 -10" rgba="0.2 0.9 0.3 0.95" contype="0" conaffinity="0"/>\n'
MARKERS += '    <geom name="mk_target" type="sphere" size="0.22" pos="0 0 -10" rgba="1 0.2 0.2 1" contype="0" conaffinity="0"/>\n'

FOOTER = "  </worldbody>\n</mujoco>\n"


def _g(name, gtype, size, pos, rgba, euler=None, collision=True, friction=True):
    e = f' euler="{euler}"' if euler else ""
    c = "" if collision else ' contype="0" conaffinity="0"'
    fr = ' friction="1 0.5 0.5"' if friction else ""
    return f'    <geom name="{name}" type="{gtype}" size="{size}" pos="{pos}" rgba="{rgba}"{e}{c}{fr}/>\n'


def urban_v1() -> str:
    s = HEADER.format(name="paper_drone_urban_v1", gc1="0.30 0.40 0.26", gc2="0.36 0.46 0.30")
    # roads + markings
    s += _g("road_ns", "box", "2.2 42 0.01", "0 0 0.011", "0.24 0.24 0.26 1", collision=False, friction=False)
    s += _g("road_ew", "box", "26 2.2 0.01", "0 -15 0.011", "0.24 0.24 0.26 1", collision=False, friction=False)
    for i, y in enumerate(range(-38, 40, 6)):
        s += _g(f"dash{i}", "box", "0.06 1.4 0.005", f"0 {y} 0.021", "0.9 0.9 0.85 1", collision=False, friction=False)
    s += _g("sidewalk_e", "box", "0.9 42 0.03", "3.1 0 0.03", "0.6 0.6 0.58 1")
    s += _g("sidewalk_w", "box", "0.9 42 0.03", "-3.1 0 0.03", "0.6 0.6 0.58 1")
    s += _g("sidewalk_n", "box", "26 0.9 0.03", "0 -12.1 0.03", "0.6 0.6 0.58 1")
    s += _g("crosswalk", "box", "1.8 0.25 0.006", "0 -12.6 0.022", "0.9 0.9 0.85 1", collision=False, friction=False)
    # helipad
    s += _g("pad_base", "cylinder", "1.5 0.02", "0 -6 0.02", "0.15 0.17 0.2 1")
    s += _g("pad_ring", "cylinder", "1.2 0.025", "0 -6 0.026", "0.95 0.8 0.1 1", collision=False, friction=False)
    s += _g("pad_inner", "cylinder", "0.9 0.028", "0 -6 0.028", "0.15 0.17 0.2 1", collision=False, friction=False)
    # buildings (collision) with roofs/details
    for name, sz, pos, col in (
        ("bldg_1", "2.6 2.4 6.0", "-9 14 6.0", "0.62 0.60 0.57"),
        ("bldg_2", "3.0 3.0 9.5", "9 20 9.5", "0.55 0.58 0.62"),
        ("bldg_3", "2.2 2.6 4.5", "-14 22 4.5", "0.68 0.62 0.55"),
        ("bldg_4", "2.8 2.2 7.5", "16 13 7.5", "0.58 0.55 0.6"),
        ("bldg_5", "2.4 2.4 5.5", "20 22 5.5", "0.66 0.6 0.52"),
        ("bldg_6", "2.0 2.0 3.2", "-19 8 3.2", "0.6 0.64 0.6"),
    ):
        s += _g(name, "box", sz, pos, f"{col} 1")
        s += _g(name + "_roof", "box",
                f"{float(sz.split()[0]) + 0.15} {float(sz.split()[1]) + 0.15} 0.25",
                f"{pos.split()[0]} {pos.split()[1]} {float(pos.split()[2]) + float(sz.split()[2]) + 0.05}",
                "0.4 0.4 0.42 1", collision=False, friction=False)
        # window strips (visual)
        s += _g(name + "_win", "box",
                f"{float(sz.split()[0]) * 1.01} {float(sz.split()[1]) * 0.4} {float(sz.split()[2]) * 0.7}",
                pos, "0.35 0.45 0.55 0.9", collision=False, friction=False)
    s += _g("bldg_4_ant", "cylinder", "0.05 1.4", "16 13 15.9", "0.38 0.38 0.4 1", collision=False, friction=False)
    # trees
    for i, (x, y, h, r) in enumerate((
            (5, -10, 1.4, 0.85), (-5, -4, 1.6, 0.9), (5, 6, 1.2, 0.75),
            (-5, 14, 1.8, 1.0), (5, 24, 1.3, 0.8), (-6, 26, 1.5, 0.9),
            (-10, -18, 1.7, 0.95), (10, -20, 1.1, 0.7))):
        s += _g(f"tree_{i}_trunk", "cylinder", f"0.09 {h}", f"{x} {y} {h}", "0.36 0.26 0.16 1")
        s += _g(f"tree_{i}_top", "sphere", str(r), f"{x} {y} {h + r * 0.85}", f"0.1{6-i%3} 0.4{2+i%3} 0.1{7+i%2} 1")
        s += _g(f"tree_{i}_top2", "sphere", str(r * 0.6), f"{x} {y} {h + r * 1.5}", f"0.1{7+i%2} 0.4{6-i%3} 0.1{8-i%3} 1", collision=False, friction=False)
    # bushes
    for i, (x, y) in enumerate(((4.2, -1.5), (-4.2, 9), (4.2, 18.5), (-4.2, -13))):
        s += _g(f"bush_{i}", "sphere", "0.35", f"{x} {y} 0.3", "0.2 0.42 0.2 1", collision=False, friction=False)
    # lamps
    for i, (x, y) in enumerate(((2.7, -30), (-2.7, -18), (2.7, -6), (-2.7, 6), (2.7, 18), (-2.7, 30))):
        s += _g(f"lamp_{i}_pole", "cylinder", "0.035 2.6", f"{x} {y} 2.6", "0.35 0.36 0.38 1")
        dx = -0.28 if x > 0 else 0.28
        s += _g(f"lamp_{i}_head", "box", "0.28 0.1 0.06", f"{x + dx} {y} 5.15", "0.9 0.85 0.5 1", collision=False, friction=False)
    # traffic light at the intersection
    s += _g("tlight_pole", "cylinder", "0.04 3.2", "2.7 -12.6 3.2", "0.3 0.3 0.32 1")
    for j, col in enumerate(((0.9, 0.2, 0.15), (0.95, 0.8, 0.2), (0.2, 0.8, 0.3))):
        s += _g(f"tlight_{j}", "sphere", "0.055", f"2.7 -12.35 {4.6 - j * 0.14}", f"{col[0]} {col[1]} {col[2]} 1", collision=False, friction=False)
    # static vehicles (procedural: wheels, cabin, windows, lights)
    s += car_xml("1", (1.1, -24, 0), 0, (0.75, 0.2, 0.18))
    s += car_xml("2", (-1.1, -2, 0), 180, (0.2, 0.35, 0.75))
    s += car_xml("3", (1.1, 12, 0), 0, (0.85, 0.7, 0.15))
    s += car_xml("4", (-14, -15.9, 0), 90, (0.3, 0.55, 0.3))
    s += truck_xml("1", (-1.1, -30, 0), 0)
    s += truck_xml("2", (12, -14.1, 0), 90, (0.4, 0.55, 0.75), (0.4, 0.55, 0.75))
    # moving traffic (kinematic; app drives these along the N-S road)
    s += car_xml("m1", (1.1, -34, 0), 0, (0.8, 0.3, 0.2))
    s += car_xml("m2", (-1.1, 20, 0), 180, (0.25, 0.45, 0.8))
    s += truck_xml("m1", (1.1, 8, 0), 0)
    s += DRONE + MARKERS + FOOTER
    return s


def open_field_v1() -> str:
    s = HEADER.format(name="paper_drone_open_field_v1", gc1="0.28 0.42 0.24", gc2="0.34 0.48 0.28")
    s += _g("pad_base", "cylinder", "1.5 0.02", "0 -6 0.02", "0.15 0.17 0.2 1")
    s += _g("pad_ring", "cylinder", "1.2 0.025", "0 -6 0.026", "0.95 0.8 0.1 1", collision=False, friction=False)
    s += _g("pad_inner", "cylinder", "0.9 0.028", "0 -6 0.028", "0.15 0.17 0.2 1", collision=False, friction=False)
    # landing zone marker at the far end
    s += _g("land_ring", "cylinder", "1.0 0.025", "0 24 0.026", "0.2 0.8 0.9 1", collision=False, friction=False)
    # scattered trees + bushes only
    for i, (x, y, h, r) in enumerate((
            (-8, 4, 1.6, 0.95), (9, 10, 1.4, 0.85), (-11, 16, 1.8, 1.0),
            (7, -14, 1.2, 0.75), (-7, -18, 1.5, 0.9), (13, 22, 1.3, 0.8))):
        s += _g(f"tree_{i}_trunk", "cylinder", f"0.09 {h}", f"{x} {y} {h}", "0.36 0.26 0.16 1")
        s += _g(f"tree_{i}_top", "sphere", str(r), f"{x} {y} {h + r * 0.85}", "0.16 0.44 0.19 1")
    for i, (x, y) in enumerate(((3, 2), (-3, 10), (4, 16), (-4, -12))):
        s += _g(f"bush_{i}", "sphere", "0.4", f"{x} {y} 0.32", "0.2 0.42 0.2 1", collision=False, friction=False)
    # parked vehicles at the edge
    s += car_xml("1", (0, -20, 0), 0, (0.7, 0.7, 0.72))
    s += truck_xml("1", (6, -20, 0), 0)
    # moving traffic (kept far from the field; app requires the bodies)
    s += car_xml("m1", (1.1, -45, 0), 0, (0.8, 0.3, 0.2))
    s += car_xml("m2", (-1.1, 45, 0), 180, (0.25, 0.45, 0.8))
    s += truck_xml("m1", (1.1, 48, 0), 0)
    s += DRONE + MARKERS + FOOTER
    return s


def build_all() -> None:
    (ENVS / "urban_v1").mkdir(parents=True, exist_ok=True)
    (ENVS / "open_field_v1").mkdir(parents=True, exist_ok=True)
    (ENVS / "urban_v1" / "world.xml").write_text(urban_v1(), encoding="utf-8", newline="\n")
    (ENVS / "open_field_v1" / "world.xml").write_text(open_field_v1(), encoding="utf-8", newline="\n")
    print(f"built: {ENVS/'urban_v1'/'world.xml'}\n      {ENVS/'open_field_v1'/'world.xml'}")


if __name__ == "__main__":
    build_all()
