"""Vehicle and simulation parameters.

The paper (Fresk & Nikolakopoulos, ECC 2013) gives inertia but never mass or
geometry -- it never needed them, since it models control-signal -> torque as
the identity and never introduces rotors. To fly a real 6-DOF vehicle we need
both, so they are *derived* from the paper's own inertia rather than picked
arbitrarily, and documented here as the assumption they are.

Derivation: Izz/Ixx = 1.846, and a planar X point-mass quadrotor predicts
exactly Izz = 2*Ixx (rotors sit in the horizontal plane, so each contributes
equally to yaw as roll+pitch combined). Decomposing four rotor point-masses at
arm length L plus an isotropic central-body inertia I_b to reproduce BOTH
published values exactly:

    Ixx = Iyy = 2*m_r*L^2 + I_b
    Izz        = 4*m_r*L^2 + I_b

    => m_r = (Izz - Ixx) / (2*L^2)      (rotor point mass)
    => I_b = 2*Ixx - Izz                 (central body inertia, isotropic)

With L = 0.15 m: m_r = 12.2 g/rotor, I_b = 1.0e-4 kg*m^2. I_b alone doesn't
fix a body mass (inertia is shape-weighted, not mass alone), so a central
body mass of 150 g is assumed (a battery+electronics puck of that mass and
I_b implies an ~4 cm equivalent radius -- physically reasonable for this
vehicle scale). Total mass ~200 g, hover thrust ~2.0 N.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from quat_sitl.dynamics import InertiaParams

G = 9.81  # m/s^2

# Paper's own values (Section V), reused verbatim.
PAPER_IXX = 6.5e-4
PAPER_IYY = 6.5e-4
PAPER_IZZ = 1.2e-3
PAPER_TORQUE_LIMIT = 4.0  # N*m, the controller-output clip from the paper


@dataclass(frozen=True)
class VehicleParams:
    """Derived airframe: paper inertia + assumed geometry -> mass, arm length,
    per-rotor thrust limit, yaw-reaction coefficient, motor lag."""

    inertia: InertiaParams = field(default_factory=InertiaParams)
    arm_length: float = 0.15  # m, tip-to-center distance L (assumption, see module docstring)
    body_mass: float = 0.15  # kg (assumption, see module docstring)
    thrust_to_weight: float = 4.0  # per-rotor headroom over hover; sets rotor_thrust_max
    yaw_torque_coeff: float = 0.016  # k_m/k_f, typical small-prop ratio (assumption)
    motor_tau: float = 0.03  # s, first-order spin-up/spin-down time constant
    linear_drag_coeff: float = 0.15  # N per (m/s), simple body-drag realism layer
    ground_friction: float = 3.0  # 1/s, horizontal-velocity damping while landed

    @property
    def rotor_mass(self) -> float:
        """m_r, derived so Ixx/Izz match the paper exactly (see module docstring)."""
        return (self.inertia.Izz - self.inertia.Ixx) / (2.0 * self.arm_length**2)

    @property
    def mass(self) -> float:
        return self.body_mass + 4.0 * self.rotor_mass

    @property
    def hover_thrust_total(self) -> float:
        return self.mass * G

    @property
    def rotor_thrust_hover(self) -> float:
        return self.hover_thrust_total / 4.0

    @property
    def rotor_thrust_max(self) -> float:
        return self.thrust_to_weight * self.rotor_thrust_hover

    @property
    def rotor_arm(self) -> float:
        """Perpendicular distance d from each body axis to a rotor, for an X
        configuration with tip-to-center distance L: d = L/sqrt(2)."""
        return self.arm_length / (2.0**0.5)

    def verify_inertia_match(self, atol: float = 1e-9) -> bool:
        """Sanity check: the derived rotor/body split reproduces the paper's
        published Ixx/Iyy/Izz exactly. Used by tests, not called at runtime.

        Four rotor point-masses at (+-d,+-d,0): each contributes m_r*d^2 to
        Ixx (and to Iyy, by symmetry) and m_r*2d^2 to Izz, so summed over all
        four rotors: Ixx_rotors = 4*m_r*d^2, Izz_rotors = 8*m_r*d^2."""
        d = self.rotor_arm
        Ib = 2.0 * self.inertia.Ixx - self.inertia.Izz
        ixx = 4.0 * self.rotor_mass * d * d + Ib
        izz = 8.0 * self.rotor_mass * d * d + Ib
        return abs(ixx - self.inertia.Ixx) < atol and abs(izz - self.inertia.Izz) < atol


@dataclass(frozen=True)
class GainParams:
    """Controller gains. Pq/Pw are the paper's own tuned values (Section V);
    altitude/manual gains are new (the paper has no translational control)."""

    Pq: float = 20.0
    Pw: float = 4.0
    torque_limit: float = PAPER_TORQUE_LIMIT
    alt_kp: float = 6.0
    alt_kd: float = 4.0
    alt_ki: float = 1.5
    alt_ki_max: float = 2.0  # anti-windup clamp on the integrator term itself


@dataclass(frozen=True)
class SimParams:
    """Scheduler/integration rates. base_hz must clear
    dynamics.stable_control_rate_hz for the chosen gains/inertia -- see
    scheduler.py, which asserts this at startup rather than trusting it."""

    base_hz: float = 16_000.0  # inner physics + attitude-loop rate
    sensor_hz: float = 1_000.0
    altitude_hz: float = 250.0
    commander_hz: float = 50.0
    logger_hz: float = 200.0
    noise_amplitude: float = 0.1  # paper's measurement noise, uniform +/- amplitude
    seed: int = 0
