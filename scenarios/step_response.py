"""Step response scenario — reproduces paper Figs 3-4: 1 rad step around each axis
at staggered time instants, 15s total."""
from __future__ import annotations

from quat_sitl import dynamics, controller, references, simulator, plotting


def main() -> None:
    df = simulator.run_simulation(
        scenario="step",
        duration=15.0,
        seed=0,
        noise_amplitude=0.1,
        reference_fn=references.step_reference,
        controller=controller.NonlinearP2Controller(),
        inertia=dynamics.InertiaParams(),
        motor_model=dynamics.IdentityMotorModel(),
    )
    path = simulator.save_run(df, "step")
    plotting.plot_scenario(df, "Step response", path.with_suffix(""))
    print(f"wrote {path} and figures")


if __name__ == "__main__":
    main()
