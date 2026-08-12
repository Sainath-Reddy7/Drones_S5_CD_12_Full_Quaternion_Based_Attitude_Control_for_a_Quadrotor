"""Main SITL loop: plant integrated at a numerically stable control-tick rate,
state logged at 1 kHz.

IMPORTANT DEVIATION FROM THE NOMINAL "1 kHz plant / 200 Hz control" SITL SPEC:
the paper's own gains (Pq=20, Pw=4) combined with its own tiny inertia
(Ixx=Iyy=6.5e-4 kg*m^2) produce a fast closed-loop pole around Pw/Ixx ~ 6150 rad/s
(~980 Hz). Sampling the rate-feedback term -Pw*omega at 200 Hz (or even 1 kHz) is
an unstable discretization of that pole -- verified via the exact zero-order-hold
discrete-time closed-loop transition matrix, not a plant-integration accuracy
issue (RK4 at 1e-3 s is already fine for a FIXED torque). Concretely, the exact
discrete closed-loop spectral radius is ~29.9 at 200 Hz and ~5.16 at 1 kHz (both
> 1, i.e. unstable); it only drops below 1 somewhere around 3-5 kHz. See
dynamics.stable_control_rate_hz and README "Results vs Paper" for the full
derivation.

Resolution adopted here: the controller runs at dynamics.stable_control_rate_hz
(computed from Pw/inertia, ~12.3 kHz for the paper's defaults, comfortable margin
below the unstable region) instead of a literal 200 Hz, while state is still
LOGGED once per dt_plant=1e-3 (1 kHz), and the plant equations (eq. 18), the
controller (eq. 19-21), and the gains (Pq=20, Pw=4) are all exactly as specified
-- only the control SAMPLE rate changed, and only because the literal 200 Hz/1 kHz
rates are provably unstable for this system, not by choice.
"""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from typing import Callable

import numpy as np
import pandas as pd
from numpy.typing import NDArray

from . import dynamics, integrator, quaternion as quat, sensors
from .controller import ControllerGains, NonlinearP2Controller
from .dynamics import IdentityMotorModel, InertiaParams, MotorModel

FloatArr = NDArray[np.float64]

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


def run_simulation(
    scenario: str,
    duration: float,
    seed: int,
    noise_amplitude: float,
    reference_fn: Callable[[float], FloatArr],
    controller: NonlinearP2Controller,
    inertia: InertiaParams,
    motor_model: MotorModel,
    dt_plant: float = 1e-3,
    control_rate_hz: float | None = None,
    torque_limit: float = 4.0,
) -> pd.DataFrame:
    """Runs the closed-loop SITL and returns one logged row per dt_plant (1 kHz).

    control_rate_hz: if None (default), computed via dynamics.stable_control_rate_hz
    for numerical stability (see module docstring). Pass an explicit value (e.g.
    200.0) to reproduce the literal spec rate and observe the resulting instability.
    """
    if control_rate_hz is None:
        control_rate_hz = dynamics.stable_control_rate_hz(controller.gains.Pw, inertia)

    n_log_steps = round(duration / dt_plant)
    n_control_per_log = max(1, round(control_rate_hz * dt_plant))
    dt_control = dt_plant / n_control_per_log

    sensor = sensors.SensorModel(noise_amplitude=noise_amplitude, rng=np.random.default_rng(seed))
    state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    q_ref = reference_fn(0.0)
    q_meas, omega_meas = state[0:4].copy(), state[4:7].copy()
    tau_held = np.zeros(3)
    saturated_held = np.zeros(3, dtype=bool)

    rows: list[dict] = []

    for k in range(n_log_steps):
        t_log = k * dt_plant
        saturated_any = np.zeros(3, dtype=bool)

        for j in range(n_control_per_log):
            t = t_log + j * dt_control
            q_ref = reference_fn(t)
            q_meas, omega_meas = sensor.measure(state[0:4], state[4:7])
            tau_cmd = controller.compute_torque(q_ref, q_meas, omega_meas)
            tau_actual = motor_model.step(tau_cmd, dt=dt_control)
            tau_held, saturated_held = dynamics.torque_saturate(tau_actual, torque_limit)
            saturated_any |= saturated_held

            def f(s: FloatArr, tau: FloatArr = tau_held) -> FloatArr:
                return dynamics.state_derivative(s, tau, inertia)

            state = integrator.rk4_step(f, state, dt_control)
            state[0:4] = quat.normalize(state[0:4])
            assert abs(quat.norm(state[0:4]) - 1.0) < 1e-6, "quaternion drifted off unit norm"

        saturated_held = saturated_any

        phi, theta, psi = quat.to_euler(state[0:4])
        phi_ref, theta_ref, psi_ref = quat.to_euler(q_ref)

        rows.append(
            {
                "time": (k + 1) * dt_plant,
                "q_ref_0": q_ref[0], "q_ref_1": q_ref[1], "q_ref_2": q_ref[2], "q_ref_3": q_ref[3],
                "q_meas_0": q_meas[0], "q_meas_1": q_meas[1], "q_meas_2": q_meas[2], "q_meas_3": q_meas[3],
                "q_true_0": state[0], "q_true_1": state[1], "q_true_2": state[2], "q_true_3": state[3],
                "omega_x": state[4], "omega_y": state[5], "omega_z": state[6],
                "omega_meas_x": omega_meas[0], "omega_meas_y": omega_meas[1], "omega_meas_z": omega_meas[2],
                # tau_* is the last control substep's torque within this log interval;
                # sat_* is OR'd across every control substep in the interval so transient
                # saturation isn't aliased away by the (much coarser) 1 kHz log rate.
                "tau_x": tau_held[0], "tau_y": tau_held[1], "tau_z": tau_held[2],
                "sat_x": saturated_held[0], "sat_y": saturated_held[1], "sat_z": saturated_held[2],
                "phi": phi, "theta": theta, "psi": psi,
                "phi_ref": phi_ref, "theta_ref": theta_ref, "psi_ref": psi_ref,
            }
        )

    return pd.DataFrame(rows)


def save_run(df: pd.DataFrame, scenario: str) -> Path:
    RESULTS_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    path = RESULTS_DIR / f"{scenario}_{timestamp}.csv"
    df.to_csv(path, index=False)
    df.to_csv(RESULTS_DIR / f"{scenario}_latest.csv", index=False)
    return path


def _build_cli_config(scenario: str):
    from . import references

    inertia = InertiaParams()
    if scenario == "step":
        reference_fn = references.step_reference
        ctrl = NonlinearP2Controller()
    elif scenario == "sine":
        reference_fn = references.sine_reference
        ctrl = NonlinearP2Controller()
    elif scenario == "flip":
        reference_fn = references.flip_reference
        ctrl = NonlinearP2Controller(shortest_path=False)
    else:
        raise ValueError(f"unknown scenario: {scenario}")
    return reference_fn, ctrl, inertia


def main() -> None:
    parser = argparse.ArgumentParser(description="Quaternion-based quadrotor SITL")
    parser.add_argument("--scenario", choices=["step", "sine", "flip"], required=True)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--noise", type=float, default=0.1)
    args = parser.parse_args()

    reference_fn, ctrl, inertia = _build_cli_config(args.scenario)
    df = run_simulation(
        scenario=args.scenario,
        duration=args.duration,
        seed=args.seed,
        noise_amplitude=args.noise,
        reference_fn=reference_fn,
        controller=ctrl,
        inertia=inertia,
        motor_model=IdentityMotorModel(),
    )
    path = save_run(df, args.scenario)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
