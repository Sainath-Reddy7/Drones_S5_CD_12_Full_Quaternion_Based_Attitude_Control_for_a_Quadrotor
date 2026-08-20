"""Flight-mode state machine and arming/preflight logic -- PX4's Commander,
scoped to what this sim supports. No GPS/navigation modes: the user asked to
skip waypoint missions and RTL. Manual stabilized/altitude-hold flying, plus
the paper's own three scenarios (step/sine/flip) as automatic modes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class FlightMode(Enum):
    STABILIZED = "STABILIZED"  # sticks -> attitude, throttle -> direct thrust
    ALTITUDE = "ALTITUDE"  # sticks -> attitude, throttle -> climb rate around held altitude
    AUTO_STEP = "AUTO_STEP"  # paper step scenario
    AUTO_SINE = "AUTO_SINE"  # paper sine scenario
    AUTO_FLIP = "AUTO_FLIP"  # paper flip scenario, shortest_path=False


AUTO_MODES = {FlightMode.AUTO_STEP, FlightMode.AUTO_SINE, FlightMode.AUTO_FLIP}


@dataclass
class PreflightCheck:
    name: str
    passed: bool
    detail: str = ""


@dataclass
class Commander:
    """Arming + mode state. Not itself a control law -- the scheduler's
    control task reads `.armed` and `.mode` to decide which controller path
    (manual/altitude/auto) drives the vehicle this tick, and zeroes thrust
    whenever disarmed."""

    mode: FlightMode = FlightMode.STABILIZED
    armed: bool = False
    last_checks: list[PreflightCheck] = field(default_factory=list)
    _failsafe_reason: str | None = None

    def preflight_checks(self, altitude: float) -> list[PreflightCheck]:
        checks = [PreflightCheck("on_ground", abs(altitude) < 0.05, f"altitude={altitude:.3f}m")]
        self.last_checks = checks
        return checks

    def try_arm(self, altitude: float) -> bool:
        checks = self.preflight_checks(altitude)
        if all(c.passed for c in checks):
            self.armed = True
            self._failsafe_reason = None
            return True
        return False

    def disarm(self, reason: str | None = None) -> None:
        self.armed = False
        self._failsafe_reason = reason

    def set_mode(self, mode: FlightMode) -> None:
        self.mode = mode

    def check_failsafe(self, altitude: float, max_altitude: float = 50.0, min_altitude: float = -1.0) -> None:
        """Bounds failsafe: disarm if the vehicle leaves a sane envelope
        (e.g. a runaway from a bad gain-tuning experiment on the dashboard)."""
        if self.armed and (altitude > max_altitude or altitude < min_altitude):
            self.disarm(reason=f"altitude envelope exceeded ({altitude:.1f} m)")

    @property
    def failsafe_reason(self) -> str | None:
        return self._failsafe_reason
