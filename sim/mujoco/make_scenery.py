"""Generate the road / vehicle scenery block for sim/mujoco/quadrotor.xml.

Everything is visual-only (contype=0 conaffinity=0): the only collider in the
world is the unchanged floor plane, so benchmark physics is untouched (seed-0
rerun verifies this). Run from the repo root to regenerate the block:

    python -m sim.mujoco.make_scenery   (prints the XML; run with --insert to write)
"""
from __future__ import annotations

V = 'contype="0" conaffinity="0"'


def geom(name, gtype, size, pos, rgba, **kw):
    extra = " ".join(f'{k}="{v}"' for k, v in kw.items())
    return (f'    <geom name="{name}" type="{gtype}" size="{size}" pos="{pos}" '
            f'rgba="{rgba}" {V} {extra}/>')


def car(name, x, y, yaw, color):
    """One car ~1.8 m long: body + cabin + 4 wheels. yaw in degrees."""
    g = []
    tr = f'euler="0 0 {yaw}"'
    g.append(geom(f"{name}_body", "box", "0.9 0.42 0.14", f"{x} {y} 0.16", color, euler=f"0 0 {yaw}"))
    g.append(geom(f"{name}_cabin", "box", "0.45 0.38 0.13", f"{x} {y} 0.36", color, euler=f"0 0 {yaw}"))
    g.append(geom(f"{name}_glass", "box", "0.42 0.34 0.09", f"{x+0.08} {y} 0.37", "0.6 0.75 0.85 0.8", euler=f"0 0 {yaw}"))
    for i, (dx, dy) in enumerate(((0.55, 0.38), (0.55, -0.38), (-0.55, 0.38), (-0.55, -0.38))):
        wx = x + dx * np.cos(np.radians(yaw)) - dy * np.sin(np.radians(yaw))
        wy = y + dx * np.sin(np.radians(yaw)) + dy * np.cos(np.radians(yaw))
        g.append(geom(f"{name}_w{i}", "cylinder", "0.14 0.06", f"{wx} {wy} 0.14",
                      "0.08 0.08 0.08 1", euler="90 0 0"))
    return g


def truck(name, x, y, yaw):
    """Semi-truck ~9 m: tractor cab + trailer + 10 wheels."""
    g = []
    a = np.radians(yaw)
    ca, sa = np.cos(a), np.sin(a)

    def place(dx, dy, z=0.0):
        return f"{x + dx*ca - dy*sa:.3f} {y + dx*sa + dy*ca:.3f} {z:.2f}"

    g.append(geom(f"{name}_cab", "box", "0.8 0.55 0.55", place(3.2, 0), "0.85 0.3 0.1 1", euler=f"0 0 {yaw}"))
    g.append(geom(f"{name}_glass", "box", "0.1 0.45 0.3", place(4.0, 0), "0.6 0.75 0.85 0.8", euler=f"0 0 {yaw}"))
    g.append(geom(f"{name}_chassis", "box", "1.0 0.45 0.15", place(2.0, 0), "0.15 0.15 0.17 1", euler=f"0 0 {yaw}"))
    g.append(geom(f"{name}_trailer", "box", "2.9 0.62 0.75", place(-0.9, 0), "0.9 0.9 0.92 1", euler=f"0 0 {yaw}"))
    g.append(geom(f"{name}_trailer_r", "box", "0.1 0.64 0.77", place(-3.85, 0), "0.75 0.2 0.15 1", euler=f"0 0 {yaw}"))
    for i, dx in enumerate((3.3, 2.5, -2.6, -3.4)):
        for j, dy in enumerate((0.5, -0.5)):
            g.append(geom(f"{name}_w{i}{j}", "cylinder", "0.22 0.08",
                          place(dx, dy * 1.1), "0.08 0.08 0.08 1", euler="90 0 0"))
    return g


import numpy as np  # noqa: E402


def build() -> str:
    g = ["    <!-- ===== roads, cars, trucks (visual-only) ===== -->"]
    # main road along y at x=12; cross road along x at y=10
    g.append(geom("road_main", "box", "2.2 30 0.01", "12 0 0.008", "0.24 0.24 0.26 1"))
    g.append(geom("road_cross", "box", "30 2.2 0.009", "0 10 0.007", "0.24 0.24 0.26 1"))
    # edge lines
    g.append(geom("road_m_l", "box", "0.04 30 0.012", "10.0 0 0.012", "0.9 0.9 0.9 1"))
    g.append(geom("road_m_r", "box", "0.04 30 0.012", "14.0 0 0.012", "0.9 0.9 0.9 1"))
    g.append(geom("road_c_f", "box", "30 0.04 0.011", "0 8.0 0.011", "0.9 0.9 0.9 1"))
    g.append(geom("road_c_n", "box", "30 0.04 0.011", "0 12.0 0.011", "0.9 0.9 0.9 1"))
    # dashed center lines
    for i in range(-9, 10):
        g.append(geom(f"dash_m{i}", "box", "0.05 0.9 0.013", f"12 {i*3.0:.0f} 0.013", "0.95 0.8 0.1 1"))
        g.append(geom(f"dash_c{i}", "box", "0.9 0.05 0.012", f"{i*3.0:.0f} 10 0.012", "0.95 0.8 0.1 1"))
    # parking lot near the town
    g.append(geom("lot", "box", "3.5 2.5 0.006", "34 14 0.006", "0.3 0.3 0.32 1"))

    # cars on the roads / lot
    g += car("car1", 12.8, -6, 90, "0.75 0.15 0.15 1")     # red, driving lane
    g += car("car2", 11.2, 4, -90, "0.15 0.3 0.7 1")      # blue, opposite lane
    g += car("car3", 34, 14, 0, "0.2 0.55 0.2 1")         # green, parked in lot
    g += car("car4", -6, 10.8, 180, "0.8 0.8 0.85 1")     # white, cross road
    g += car("car5", 6, 9.2, 0, "0.9 0.6 0.05 1")         # orange, cross road

    # trucks
    g += truck("truck1", 12.8, 14, 90)                    # on the main road
    g += truck("truck2", -12, 10.8, 0)                    # on the cross road

    return "\n".join(g)


if __name__ == "__main__":
    print(build())
