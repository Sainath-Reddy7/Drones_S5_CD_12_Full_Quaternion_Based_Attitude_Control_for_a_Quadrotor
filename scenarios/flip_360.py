"""360-degree flip scenario — reproduces paper Figs 7-8: ramp 0->2pi rad about x
over 2s, 5s total, shortest_path DISABLED so the controller commits to the full
rotation instead of taking the short way round."""
from __future__ import annotations

from quat_sitl import dynamics, controller, references, simulator, plotting


def main() -> None:
    df = simulator.run_simulation(
        scenario="flip",
        duration=5.0,
        seed=0,
        noise_amplitude=0.1,
        reference_fn=references.flip_reference,
        controller=controller.NonlinearP2Controller(shortest_path=False),
        inertia=dynamics.InertiaParams(),
        motor_model=dynamics.IdentityMotorModel(),
    )
    path = simulator.save_run(df, "flip")
    plotting.plot_flip(df, "360-degree flip", path.with_suffix(""))
    print(f"wrote {path} and figures")


if __name__ == "__main__":
    main()
