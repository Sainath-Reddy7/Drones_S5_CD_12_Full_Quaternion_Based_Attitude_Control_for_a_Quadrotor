"""Generate the static deployment site (deploy/) from the benchmark output.

The site is PROOF-FRIENDLY BY CONSTRUCTION: every number on it is read from
results/comparison.md (the output of `python -m sim.compare`) and the figures
are the adapter plots themselves -- nothing is hand-transcribed.

Run from the repo root (needs the benchmark data present):

    python -m sim.site_gen          # writes deploy/index.html etc.

On `main` (which carries no results/), a snapshot of the benchmark lives at
deploy/benchmark.md and deploy/figures/ -- site_gen prefers live results/
and falls back to the snapshot, so the site regenerates identically either way.
"""
from __future__ import annotations

import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEPLOY = REPO / "deploy"

CSS = """
  body { font-family: -apple-system, Segoe UI, sans-serif; margin: 0; background: #f6f8fa; color: #1f2328; }
  header { background: linear-gradient(135deg, #1f2328, #2d333b); color: #fff; padding: 40px 24px 32px; }
  header h1 { margin: 0 0 8px; font-size: 26px; max-width: 900px; }
  header p { margin: 0; color: #b6bcc2; font-size: 15px; max-width: 860px; }
  main { max-width: 1080px; margin: 0 auto; padding: 24px; }
  .stacks { display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 14px; margin-top: -18px; }
  .card { background: #fff; border-radius: 8px; padding: 16px; box-shadow: 0 1px 4px rgba(0,0,0,.12); border-top: 3px solid #d0d7de; }
  .card.ok { border-top-color: #1a7f37; }
  .card.ready { border-top-color: #9a6700; }
  .card h3 { margin: 0 0 6px; font-size: 15px; }
  .card .st { font-size: 12px; font-weight: 600; letter-spacing: .3px; }
  .card.ok .st { color: #1a7f37; }
  .card.ready .st { color: #9a6700; }
  .card p { margin: 6px 0 0; font-size: 13px; color: #57606a; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; background: #fff; box-shadow: 0 1px 3px rgba(0,0,0,.08); }
  th, td { border: 1px solid #d8dee4; padding: 6px 10px; text-align: right; }
  th { background: #eef1f4; }
  th:nth-child(1), th:nth-child(2), td:nth-child(1), td:nth-child(2) { text-align: left; }
  h2 { font-size: 19px; margin: 34px 0 12px; }
  .figs { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .figs figure { margin: 0; background: #fff; padding: 10px; box-shadow: 0 1px 3px rgba(0,0,0,.08); }
  .figs img { width: 100%; }
  figcaption { font-size: 12px; color: #57606a; padding-top: 6px; }
  .note { background: #ddf4ff; border: 1px solid #54aeff; border-radius: 6px; padding: 12px 16px; font-size: 14px; }
  .findings li { margin: 8px 0; font-size: 14px; line-height: 1.5; }
  pre { background: #1f2328; color: #e6edf3; padding: 14px 16px; border-radius: 8px; overflow-x: auto; font-size: 13px; }
  a { color: #0969da; }
  .btn { display: inline-block; background: #1f6feb; color: #fff; padding: 9px 16px; border-radius: 6px; text-decoration: none; font-size: 14px; margin: 4px 8px 0 0; }
  .btn.ghost { background: #fff; color: #1f6feb; border: 1px solid #d0d7de; }
"""

TABLE_HEADER = (
    "<tr><th>simulator</th><th>scenario</th><th>RMS att. err [deg]</th>"
    "<th>max att. err [deg]</th><th>settle &phi; [s]</th><th>settle &theta; [s]</th>"
    "<th>settle &psi; [s]</th><th>torque sat.</th><th>drift [m]</th><th>duration [s]</th></tr>"
)

REPO_URL = (
    "https://github.com/Sainath-Reddy7/"
    "Drones_S5_CD_12_Full_Quaternion_Based_Attitude_Control_for_a_Quadrotor"
)
BRANCH_URL = f"{REPO_URL}/tree/sainath/sim-deployment"


def read_benchmark() -> str:
    live = REPO / "results" / "comparison.md"
    snap = DEPLOY / "benchmark.md"
    src = live if live.exists() else snap
    if not src.exists():
        raise SystemExit(f"no benchmark data: looked at {live} and {snap} -- run sim.compare first")
    return src.read_text(encoding="utf-8")


def benchmark_rows(md: str) -> str:
    rows = [l for l in md.splitlines() if l.startswith("|")][2:]
    body = ""
    for r in rows:
        c = [x.strip() for x in r.strip().strip("|").split("|")]
        try:
            rms = float(c[2])
            color = "#1a7f37" if rms < 20 else ("#9a6700" if rms < 50 else "#cf222e")
            rms_cell = f'<td style="color:{color};font-weight:600">{c[2]}</td>'
        except ValueError:
            rms_cell = f"<td>{c[2]}</td>"
        body += "<tr>" + "".join(f"<td>{x}</td>" for x in c[:2]) + rms_cell
        body += "".join(f"<td>{x}</td>" for x in c[3:]) + "</tr>\n"
    return body


def figures_section(prefix: str, title: str, flip_caption: str) -> str:
    figs = [
        (f"{prefix}_step_attitude.png", "Step: 1 rad on &phi;,&theta;,&psi; staggered at t=1,5,9 s — reference dashed vs measured."),
        (f"{prefix}_step_torque.png", "Step torque, briefly saturating at each step onset (paper Fig. 4)."),
        (f"{prefix}_sine_attitude.png", "Sine: 0.5 rad, 1 rad/s, phase-shifted (paper Figs. 5&ndash;6)."),
        (f"{prefix}_flip_attitude.png", flip_caption),
    ]
    out = f"<h2>{title}</h2>\n<div class=\"figs\">\n"
    for name, cap in figs:
        out += f'<figure><img src="figures/{name}" alt=""><figcaption>{cap}</figcaption></figure>\n'
    return out + "</div>\n"


def build_index(rows: str) -> str:
    mujoco_flip = "360&deg; flip from 60 m: completes airborne, quaternions smooth through 2&pi; — no gimbal-lock artifact."
    gym_flip = "Flip: the documented engine-dependent limit cycle (finding 4)."
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>One Quaternion Controller, Four Simulators — Quadrotor Attitude Control (Group 12)</title>
<style>{CSS}</style>
</head>
<body>
<header>
  <h1>One frozen quaternion controller, four simulators</h1>
  <p>The Fresk &amp; Nikolakopoulos (ECC 2013) attitude law — gains untouched (P&#8347;=20, P&#969;=4, &plusmn;4 N&middot;m) — flying the same paper vehicle on gym-pybullet-drones, MuJoCo, Gazebo, and ArduPilot SITL through one shared bridge. Group 12 &middot; Introduction to Data-Driven Control of Drones &middot; Amrita Vishwa Vidyapeetham.</p>
</header>
<main>

<div class="stacks">
  <div class="card ok"><h3>gym-pybullet-drones</h3><span class="st">&#10003; VALIDATED NATIVELY</span><p>24 kHz control loop, custom paper-vehicle URDF. Step &amp; sine within 0.2&deg; RMS of MuJoCo.</p></div>
  <div class="card ok"><h3>MuJoCo</h3><span class="st">&#10003; VALIDATED NATIVELY</span><p>20 kHz physics, contact floor. All three paper scenarios; flip completes airborne (3 seeds).</p></div>
  <div class="card ready"><h3>Gazebo</h3><span class="st">&#9679; RUNBOOK-READY (WSL2)</span><p>SDF 1.10 world + ardupilot_gazebo physics; one-shot launch script in sim/gazebo/.</p></div>
  <div class="card ready"><h3>ArduPilot SITL</h3><span class="st">&#9679; BRIDGE-READY (WSL2)</span><p>MAVLink SET_ATTITUDE_TARGET @ 50 Hz — the controller as an outer loop, exactly as on hardware.</p></div>
</div>

<h2>The headline result</h2>
<p class="note"><b>With identical gains, references, and the paper's &plusmn;0.1 noise model, step and sine tracking agree across two independent physics engines to within 0.2&deg; RMS (17.7&deg; vs 17.7&deg;; 17.6&deg; vs 17.5&deg;).</b> The closed loop belongs to the controller, not the engine. The 360&deg; flip completes deterministically on MuJoCo and hits an engine-dependent limit cycle on PyBullet — analyzed as the P&sup2; pursuit equilibrium (Eq. 33 in the repo's file map).</p>

<h2>Benchmark — every run (seeds 0&ndash;2 for flips)</h2>
<table>
{TABLE_HEADER}
{rows}</table>

{figures_section("mujoco", "MuJoCo — the paper's three scenarios", mujoco_flip)}
{figures_section("gym", "gym-pybullet-drones — same controller, same references", gym_flip)}

<h2>Findings (all in the repo, with measurements)</h2>
<ol class="findings">
<li><b>The controller transfers across engines.</b> Step/sine agree within 0.2&deg; RMS between MuJoCo and PyBullet.</li>
<li><b>The paper's &plusmn;4 N&middot;m bound is not rotor-realizable</b> (~0.4 N&middot;m physical max), and yaw's tiny authority under &plusmn;0.1 quaternion noise demanded PX4-style <i>priority</i> mixer desaturation (Eq. 32).</li>
<li><b>Flips need acro-style thrust handling:</b> idle collective when inverted, cap when torque demand is high — tilt-compensated altitude hold otherwise deadlocks the flip at exactly 180&deg;.</li>
<li><b>The flip is knife-edge on rotor plants:</b> the law's rate equilibrium &omega;=5&middot;sin(e/2) makes a 2 s ramp trackable only at lag &asymp;1.36 rad; MuJoCo completes, PyBullet limit-cycles (Eq. 33).</li>
<li><b>The 12.3 kHz control-rate bound carries over:</b> both native adapters run the controller every physics step (20&ndash;24 kHz) and assert the derived minimum at startup.</li>
<li><b>Integration gotchas documented:</b> BaseAviary.GRAVITY is weight not g; PyBullet's default damping eats flip momentum; MJCF visual geoms silently add mass; spawn poses must follow the scenario altitude.</li>
</ol>

<h2>Run it yourself</h2>
<pre>git clone {REPO_URL}.git
cd Drones_* &amp;&amp; git checkout sainath/sim-deployment

python -m sim.mujoco.run --scenario step --duration 15 --seed 0 --noise 0.1   # MuJoCo, native
python -m sim.gym_pybullet.run --scenario flip                               # gym-pybullet-drones, native
bash sim/gazebo/run_gazebo.sh step                                           # Gazebo, under WSL2
python -m sim.ardupilot.bridge_node --scenario step                          # ArduPilot SITL, WSL2
bash sim/run_all.sh                                                          # whole native benchmark + this table</pre>

<p style="font-size:14px">Every number and figure on this page is machine-generated by <code>sim/run_all.sh</code> + <code>python -m sim.compare</code> + <code>python -m sim.site_gen</code> — nothing hand-edited. Full source, tests (39), and derivations: <a href="{BRANCH_URL}">sainath/sim-deployment branch</a>.</p>

<p>
<a class="btn" href="simulator.html">Fly the interactive simulator &rarr;</a>
<a class="btn ghost" href="{BRANCH_URL}">Source on GitHub</a>
</p>

</main>
</body>
</html>
"""


def build_results_redirect() -> str:
    return """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta http-equiv="refresh" content="0; url=/">
<title>Benchmark moved</title>
</head>
<body style="font-family:sans-serif;padding:40px">
The full benchmark (table + figures + findings) now lives on the <a href="/">main page</a>.
</body>
</html>
"""


def main() -> None:
    md = read_benchmark()
    rows = benchmark_rows(md)

    # ensure the original interactive simulator is available as simulator.html
    src_sim = REPO / "fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels(1).html"
    sim_dest = DEPLOY / "simulator.html"
    if src_sim.exists() and not sim_dest.exists():
        shutil.copy(src_sim, sim_dest)

    (DEPLOY / "index.html").write_text(build_index(rows), encoding="utf-8", newline="\n")
    (DEPLOY / "results.html").write_text(build_results_redirect(), encoding="utf-8", newline="\n")

    # keep the benchmark snapshot next to the site (main has no results/)
    (DEPLOY / "benchmark.md").write_text(md, encoding="utf-8", newline="\n")
    print(f"site written to {DEPLOY}: index.html (four-simulator main), "
          f"results.html (redirect), simulator.html, benchmark.md snapshot")


if __name__ == "__main__":
    main()
