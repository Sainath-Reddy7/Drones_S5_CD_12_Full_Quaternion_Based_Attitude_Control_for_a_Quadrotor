"""Interactive MuJoCo flight -- one launcher, every mode.

    1) MANUAL      you fly (W/S/A/D/Q/E/Up/Down, R reset, Esc quit) -- the
                   paper's unmodified controller is the stability loop
    2) FLIP        the 360 backflip from 60 m (through the clouds)
    3) STEP        the staircase scenario
    4) SINE        the wave-tracking scenario
    5) ALL         2 -> 3 -> 4 in a row

Manual mode keys work while the 3D window has focus. Scenarios run the exact
benchmark controller/seed (--noise 0.1). Output goes OUTSIDE the repo
(../_demo_runs/) so committed benchmark CSVs are never touched.

Usage:  python -m sim.mujoco.interactive
"""
from __future__ import annotations

from pathlib import Path

OUT = Path(__file__).resolve().parents[2].parent / "_demo_runs" / "mujoco"

MENU = """
==========================================================
  INTERACTIVE QUADROTOR -- Full Quaternion Control (Group 12)
==========================================================
  1) MANUAL -- fly it yourself (WASD + arrows)
  2) FLIP   -- 360 backflip from 60 m, through the clouds
  3) STEP   -- staircase attitude scenario
  4) SINE   -- smooth wave tracking
  5) ALL    -- run 2, 3, 4 in a row
  q) quit
==========================================================
"""


def run_scenario(name: str) -> None:
    from sim.mujoco.run import run

    m = run(name, seed=0, noise=0.1, gui=True, out_root=OUT)
    print(f"    -> {name}: RMS {m['rms_alpha_deg']:.1f} deg, "
          f"settle {m['settle_phi_s']:.2f} s, sat {100*m['sat_fraction']:.1f}%\n")


def run_manual() -> None:
    from sim.mujoco.manual import main as manual_main

    manual_main()


if __name__ == "__main__":
    picks = {"1": [None], "2": ["flip"], "3": ["step"], "4": ["sine"],
             "5": ["flip", "step", "sine"]}
    while True:
        choice = input(MENU + "choice> ").strip().lower()
        if choice == "q":
            break
        if choice not in picks:
            print("  (press 1-5 or q)\n")
            continue
        if choice == "1":
            run_manual()
        else:
            for scen in picks[choice]:
                print(f"\n>>> {scen.upper()} -- watch the viewer window\n")
                try:
                    run_scenario(scen)
                except Exception as e:
                    print(f"    restart-after: {e}")
    print("bye")
