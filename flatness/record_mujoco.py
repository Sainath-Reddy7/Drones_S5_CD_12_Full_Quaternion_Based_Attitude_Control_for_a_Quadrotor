"""In-engine MuJoCo recording of the flatness path-planning circle.

Runs the exact run_mujoco.run() control loop (same code path, so the
recording IS the benchmark run) inside flatness/circle_scene.xml — the
paper vehicle plus visual-only markers tracing the paper's Section V
reference circle (radius 1 m at altitude 1 m, start marker at (-1, 0)) —
and captures offscreen MuJoCo renders with a slowly orbiting chase camera,
assembled into an animated GIF with live time / radial / altitude error.

Usage:
    python -m flatness.record_mujoco [--cycles 2] [--noise 0.0]
Output:
    results/recordings/mujoco_flatness_RENDER.gif
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from flatness.run_mujoco import run

SCENE = Path(__file__).with_name("circle_scene.xml")
OUT = Path(__file__).resolve().parent.parent / "results" / "recordings"
W, H = 720, 540
EVERY_S = 0.2       # capture cadence [s of simulated time]
FPS = 12
R_REF = 1.0         # circle radius [m]
Z_REF = 1.0         # circle altitude [m]


def record(cycles: float = 2.0, noise: float = 0.0, speed: float = 0.5) -> dict:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = mujoco.MjvCamera()
    frames: list[np.ndarray] = []
    stats: list[tuple[float, float, float]] = []
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    last_cap = [-1.0]

    def frame_cb(t: float, data) -> None:
        if t - last_cap[0] < EVERY_S:
            return
        last_cap[0] = t
        p = data.qpos[0:3]
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [p[0] * 0.7, p[1] * 0.7, 0.55]  # frame drone + circle
        cam.distance = 3.4
        cam.azimuth = -62.0 + 8.0 * t                     # slow orbit
        cam.elevation = -16.0
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render())
        radial = float(np.linalg.norm(p[:2]))
        stats.append((float(t), radial, float(p[2])))

    summary = run(cycles=cycles, noise=noise, speed=speed,
                  frame_cb=frame_cb, xml_path=SCENE,
                  out_root=OUT.parent / "flatness_render")
    renderer.close()

    fig = plt.figure(figsize=(7.6, 6.0))
    ax = fig.add_subplot(111)
    ax.set_axis_off()
    im = ax.imshow(np.zeros((H, W, 3), np.uint8))
    title = ax.set_title("", fontsize=11)

    def frame(i: int):
        im.set_array(frames[i])
        t, radial, z = stats[i]
        title.set_text(
            "Choutri & Lagha 2017 — flatness + LQR circle, MuJoCo contact physics\n"
            f"t = {t:5.1f} s    radial = {radial:5.3f} m (ref {R_REF:.3f})"
            f"    z = {z:5.3f} m (ref {Z_REF:.3f})")
        return [im]

    OUT.mkdir(parents=True, exist_ok=True)
    anim = FuncAnimation(fig, frame, frames=len(frames), blit=False)
    gif = OUT / "mujoco_flatness_RENDER.gif"
    anim.save(str(gif), writer=PillowWriter(fps=FPS), dpi=100)
    plt.close(fig)
    print(f"wrote {gif} ({gif.stat().st_size // 1024} KB, {len(frames)} frames)")
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cycles", type=float, default=2.0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=0.5)
    args = ap.parse_args()
    print(record(cycles=args.cycles, noise=args.noise, speed=args.speed))
