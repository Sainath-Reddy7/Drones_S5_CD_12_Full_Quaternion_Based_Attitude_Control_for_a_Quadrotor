"""Procedural vehicle builders for the interactive worlds.

Cars: body + hood + cabin with tinted windows + 4 wheels + head/tail lights.
Trucks: chassis + cab + cargo container + 6 wheels. Collision = one box per
vehicle (wheels/trim are visual-only) -- cheap collision, recognizable
silhouettes. Shared by every environment version so vehicles look the same
everywhere.
"""
from __future__ import annotations

CAR_SCALE = dict(body=(0.42, 0.95, 0.16), cabin=(0.36, 0.42, 0.14),
                 wheel_r=0.09, wheel_w=0.06, ground=0.09)
TRUCK_SCALE = dict(container=(0.55, 1.55, 0.55), cab=(0.52, 0.5, 0.5),
                   wheel_r=0.13, wheel_w=0.08, ground=0.13)


def _geom(name, gtype, size, pos, rgba, euler=None, collision=True):
    e = f' euler="{euler}"' if euler else ""
    c = "" if collision else ' contype="0" conaffinity="0"'
    return (f'    <geom name="{name}" type="{gtype}" size="{size}" '
            f'pos="{pos}" rgba="{rgba}"{e}{c} friction="1 0.5 0.5"/>\n')


def car_xml(idx: str, pos: tuple[float, float, float], yaw_deg: float,
            color: tuple[float, float, float]) -> str:
    """One recognizable car: ~2 m long, 4 wheels, cabin + windows, lights.
    `pos` is the ground contact center; yaw rotates it onto the road."""
    x, y, _ = pos
    g = CAR_SCALE
    wx, wy = g["body"][0], g["body"][1]
    z = g["ground"] + g["body"][2]
    c = f"{color[0]} {color[1]} {color[2]}"
    s = [f'  <body name="car_{idx}" pos="{x} {y} 0">\n']
    s.append(f'    <freejoint name="car_{idx}_j"/>\n')
    s.append(f'    <geom name="car_{idx}" type="box" size="{wx} {wy} {g["body"][2]}" '
             f'pos="0 0 {z}" rgba="{c} 1" euler="0 0 {yaw_deg}" friction="1 0.5 0.5"/>\n')
    # hood (front) + trunk silhouette: slightly lower boxes fore/aft
    s.append(_geom(f"car_{idx}_hood", "box", f"{wx*0.9} {wy*0.22} {g['body'][2]*0.5}",
                   f"0 {wy + 0.10} {z - g['body'][2]*0.35}", f"{c} 1",
                   euler=f"0 0 {yaw_deg}", collision=False))
    s.append(_geom(f"car_{idx}_trunk", "box", f"{wx*0.9} {wy*0.18} {g['body'][2]*0.5}",
                   f"0 {-wy - 0.08} {z - g['body'][2]*0.35}", f"{c} 1",
                   euler=f"0 0 {yaw_deg}", collision=False))
    # cabin with tinted windows (two-tone)
    s.append(_geom(f"car_{idx}_cabin", "box",
                   f"{g['cabin'][0]} {g['cabin'][1]} {g['cabin'][2]}",
                   f"0 {-wy*0.15} {z + g['body'][2] + g['cabin'][2]}",
                   f"{c} 1", euler=f"0 0 {yaw_deg}", collision=False))
    s.append(_geom(f"car_{idx}_glass", "box",
                   f"{g['cabin'][0]*1.02} {g['cabin'][1]*0.8} {g['cabin'][2]*0.55}",
                   f"0 {-wy*0.15} {z + g['body'][2] + g['cabin'][2]*0.95}",
                   "0.15 0.2 0.28 0.85", euler=f"0 0 {yaw_deg}", collision=False))
    # 4 wheels (visual cylinders rotated to stand upright)
    for k, (ox, oy) in enumerate(((wx, wy * 0.55), (wx, -wy * 0.55),
                                  (-wx, wy * 0.55), (-wx, -wy * 0.55))):
        s.append(_geom(f"car_{idx}_w{k}", "cylinder",
                       f"{g['wheel_r']} {g['wheel_w']}",
                       f"{ox} {oy} {g['ground']}", "0.08 0.08 0.09 1",
                       euler="90 0 0", collision=False))
    # head/tail lights
    s.append(_geom(f"car_{idx}_hl", "box", f"{wx*0.5} 0.04 0.04",
                   f"0 {wy + 0.16} {z}", "1 0.95 0.6 1", collision=False))
    s.append(_geom(f"car_{idx}_tl", "box", f"{wx*0.5} 0.04 0.04",
                   f"0 {-wy - 0.16} {z}", "0.9 0.15 0.1 1", collision=False))
    s.append("  </body>\n")
    return "".join(s)


def truck_xml(idx: str, pos: tuple[float, float, float], yaw_deg: float,
              cab_color=(0.35, 0.45, 0.6), box_color=(0.85, 0.82, 0.78)) -> str:
    """Container truck: ~3.5 m long, 6 wheels, clearly bigger than a car."""
    x, y, _ = pos
    g = TRUCK_SCALE
    z = g["ground"]
    cab_c = f"{cab_color[0]} {cab_color[1]} {cab_color[2]}"
    box_c = f"{box_color[0]} {box_color[1]} {box_color[2]}"
    s = [f'  <body name="truck_{idx}" pos="{x} {y} 0">\n']
    s.append(f'    <freejoint name="truck_{idx}_j"/>\n')
    # collision: one box covering container + cab
    s.append(_geom(f"truck_{idx}", "box",
                   f"{g['container'][0]} {g['container'][1] + g['cab'][1] + 0.15} {g['container'][2]}",
                   f"0 {-g['cab'][1]*0.15} {z + g['container'][2]}",
                   f"{box_c} 1", euler=f"0 0 {yaw_deg}"))
    # cab (front) with window
    s.append(_geom(f"truck_{idx}_cab", "box",
                   f"{g['cab'][0]} {g['cab'][1]} {g['cab'][2]}",
                   f"0 {g['container'][1] + g['cab'][1] + 0.12} {z + g['cab'][2]}",
                   f"{cab_c} 1", euler=f"0 0 {yaw_deg}", collision=False))
    s.append(_geom(f"truck_{idx}_cabglass", "box",
                   f"{g['cab'][0]*0.9} {g['cab'][1]*0.3} {g['cab'][2]*0.55}",
                   f"0 {g['container'][1] + g['cab'][1] * 1.45} {z + g['cab'][2] * 1.25}",
                   "0.15 0.2 0.28 0.85", euler=f"0 0 {yaw_deg}", collision=False))
    # cargo container with ribs
    s.append(_geom(f"truck_{idx}_box", "box",
                   f"{g['container'][0]*0.98} {g['container'][1]} {g['container'][2]*0.96}",
                   f"0 {-g['cab'][1]*0.35} {z + g['container'][2]}",
                   f"{box_c} 1", euler=f"0 0 {yaw_deg}", collision=False))
    for k, r in enumerate((-0.9, 0.0, 0.9)):
        s.append(_geom(f"truck_{idx}_rib{k}", "box",
                       f"{g['container'][0]} 0.015 0.12",
                       f"0 {r - g['cab'][1]*0.35} {z + g['container'][2]}",
                       f"{cab_c} 1", euler=f"0 0 {yaw_deg}", collision=False))
    # 6 wheels
    ys = (g["container"][1] * 0.75, 0.0, -g["container"][1] * 0.8)
    for k, oy in enumerate(ys):
        for sgn, side in ((1, "r"), (-1, "l")):
            s.append(_geom(f"truck_{idx}_w{k}{side}", "cylinder",
                           f"{g['wheel_r']} {g['wheel_w']}",
                           f"{sgn * (g['container'][0] + 0.02)} {oy} {g['ground']}",
                           "0.08 0.08 0.09 1", euler="90 0 0", collision=False))
    s.append(_geom(f"truck_{idx}_hl", "box", "0.08 0.04 0.05",
                   f"0 {g['container'][1] + g['cab'][1] * 2 + 0.05} {z + g['cab'][2] * 0.7}",
                   "1 0.95 0.6 1", collision=False))
    s.append("  </body>\n")
    return "".join(s)
