"""In-engine MuJoCo recordings of the base paper's three attitude tests.

Runs the exact sim.mujoco.run benchmark (same code path — the recording IS
the benchmark run) with an offscreen renderer and the GUI's proven chase
camera (the attitude-only scenarios drift by design, so the camera tracks
the drone). Title overlay: scenario, time, and the quaternion tracking
error the benchmark logs.

Usage:
    python -m sim.mujoco.record            # step + sine + flip
    python -m sim.mujoco.record --scenario flip
Output:
    results/recordings/mujoco_{scenario}_RENDER.gif
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from sim.mujoco.run import run

OUT = Path(__file__).resolve().parent.parent.parent / "results" / "recordings"
W, H = 720, 540
FPS = 12
HOME = (0.0, 0.0, 2.0)   # puppet stage point: attitude tests drift kilometers
# by design, and MuJoCo's offscreen path loses bodies far from the world
# origin — so the render puppet flies at a fixed home (orientation and body
# rates are live) with the pad/trees as backdrop and a slow-orbit camera.


def make_render_pair(xml_path: Path):
    """(render_model, render_data) puppet copy of the scene for offscreen
    rendering. MuJoCo 3.x's classic Renderer skips geoms with contype=0
    conaffinity=0 (the repo's 'visual-only' attribute) and MjvOption hides
    group 3+ — the drone's entire visual upgrade is both, so the vehicle
    never rasterizes offscreen. The puppet copy flips those flags; it is
    never stepped, so the physics model is untouched."""
    import mujoco

    xml = Path(xml_path).read_text(encoding="utf-8")
    xml = xml.replace('contype="0" conaffinity="0"', 'contype="1" conaffinity="1"')
    model = mujoco.MjModel.from_xml_string(xml)
    opt = mujoco.MjvOption()
    for g in range(6):
        opt.geomgroup[g] = 1
    return model, mujoco.MjData(model), opt


def record(scenario: str, every_s: float = 0.12, seed: int = 0) -> dict:
    import mujoco

    r_model, r_data, opt = make_render_pair(Path(__file__).with_name("quadrotor.xml"))
    renderer = mujoco.Renderer(r_model, height=H, width=W)
    cam = mujoco.MjvCamera()
    frames: list[np.ndarray] = []
    stats: list[tuple[float, float]] = []
    last = [-1.0]

    def frame_cb(t: float, data, info) -> None:
        if t - last[0] < every_s:
            return
        last[0] = t
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = HOME
        cam.distance = 1.4
        cam.azimuth = 25.0 + 6.0 * t          # slow orbit: all attitude angles
        cam.elevation = -18.0
        r_data.qpos[0:3] = HOME               # puppet: live attitude, fixed stage
        r_data.qpos[3:7] = data.qpos[3:7]
        r_data.qvel[:] = data.qvel
        mujoco.mj_forward(r_model, r_data)
        renderer.update_scene(r_data, camera=cam, scene_option=opt)
        frames.append(renderer.render())
        q_ref, q_m = info["q_ref"], info["q_m"]
        err = float(np.degrees(2 * np.arccos(np.clip(abs(float(np.dot(q_ref, q_m))), 0, 1))))
        stats.append((float(t), err))

    summary = run(scenario, seed=seed, frame_cb=frame_cb,
                  out_root=OUT.parent / "sim_mujoco")
    renderer.close()

    fig = plt.figure(figsize=(7.6, 6.0))
    ax = fig.add_subplot(111)
    ax.set_axis_off()
    im = ax.imshow(np.zeros((H, W, 3), np.uint8))
    title = ax.set_title("", fontsize=11)

    def frame(i: int):
        im.set_array(frames[i])
        t, err = stats[i]
        title.set_text(
            f"Fresk & Nikolakopoulos — {scenario.upper()} test, MuJoCo contact physics\n"
            f"t = {t:6.2f} s    quaternion error = {err:6.1f} deg (paper noise ±0.1)")
        return [im]

    OUT.mkdir(parents=True, exist_ok=True)
    gif = OUT / f"mujoco_{scenario}_RENDER.gif"
    FuncAnimation(fig, frame, frames=len(frames), blit=False).save(
        str(gif), writer=PillowWriter(fps=FPS), dpi=100)
    plt.close(fig)
    print(f"wrote {gif.name} ({gif.stat().st_size // 1024} KB, {len(frames)} frames)")
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", choices=["step", "sine", "flip"], default=None,
                    help="record one scenario (default: all three)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    for scen in ([args.scenario] if args.scenario else ["step", "sine", "flip"]):
        print(record(scen, seed=args.seed))
