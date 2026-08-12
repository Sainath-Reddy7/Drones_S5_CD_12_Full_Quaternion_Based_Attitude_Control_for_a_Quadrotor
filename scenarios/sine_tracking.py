"""Sine tracking scenario — reproduces paper Figs 5-6: 0.5 rad amplitude, 1 rad/s
sine, phase-shifted on z so torques are not in phase, 15s total."""
from __future__ import annotations

from quat_sitl import dynamics, controller, references, simulator, plotting


def main() -> None:
    df = simulator.run_simulation(
        scenario="sine",
        duration=15.0,
        seed=0,
        noise_amplitude=0.1,
        reference_fn=references.sine_reference,
        controller=controller.NonlinearP2Controller(),
        inertia=dynamics.InertiaParams(),
        motor_model=dynamics.IdentityMotorModel(),
    )
    path = simulator.save_run(df, "sine")
    plotting.plot_scenario(df, "Sine tracking", path.with_suffix(""))
    if df[["sat_x", "sat_y", "sat_z"]].any().any():
        print("WARNING: saturation occurred during sine tracking (paper expects none)")
    print(f"wrote {path} and figures")


if __name__ == "__main__":
    main()
