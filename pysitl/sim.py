"""Orchestrator: wires the plant, airframe, controllers, bus, scheduler and
commander into one Simulation object -- the PX4-shaped module architecture
the plan's phase 3 calls for. Every module talks to every other module only
through `self.bus` (see bus.py's docstring for why), even though they're all
in one process; run.py and gcs/server.py drive a Simulation, never the plant
directly.

Auto (paper-scenario) modes are wired in by set_autopilot() -- see
control/autopilot.py, added in phase 4. Without it, AUTO_* modes safely fall
back to holding the last manual attitude reference rather than raising, since
disarming/holding is always a safe default for an unconfigured mode.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Callable, Optional

from quat_sitl.quaternion import to_euler

from . import topics
from .airframe import Airframe, mix, thrusts_to_wrench
from .bus import MessageBus
from .control.altitude import AltitudeController
from .control.attitude import NonlinearP2Controller, compute_torque_scalar
from .control.autopilot import is_autopilot_mode, reference_fn_for, shortest_path_for
from .control.manual import ManualInput, StabilizedMapper
from .modes import AUTO_MODES, Commander, FlightMode
from .params import GainParams, SimParams, VehicleParams
from .plant import Plant
from .scheduler import Scheduler
from .sensors import SensorModel

MAX_CLIMB_RATE = 1.5  # m/s, throttle full-deflection climb/descend rate in ALTITUDE mode


@dataclass
class Simulation:
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    gains: GainParams = field(default_factory=GainParams)
    sim_params: SimParams = field(default_factory=SimParams)

    def __post_init__(self) -> None:
        self.bus = MessageBus()
        self.plant = Plant(vehicle=self.vehicle)
        self.airframe = Airframe(vehicle=self.vehicle)
        self.commander = Commander()
        self.attitude_ctrl = NonlinearP2Controller()
        self.attitude_ctrl.gains.Pq = self.gains.Pq
        self.attitude_ctrl.gains.Pw = self.gains.Pw
        self.altitude_ctrl = AltitudeController(gains=self.gains, vehicle=self.vehicle)
        self.manual_mapper = StabilizedMapper()
        self.manual_input = ManualInput()
        self.sensor_model = SensorModel(
            noise_amplitude=self.sim_params.noise_amplitude,
            rng=random.Random(self.sim_params.seed),
        )
        self.target_altitude = 0.5
        self.autopilot_fn: Optional[Callable[[float], tuple]] = None
        self.autopilot_shortest_path = True

        self.scheduler = Scheduler(self.sim_params.base_hz, self.gains.Pw, self.vehicle.inertia)
        self.scheduler.register("sensors", self.sim_params.sensor_hz, self._task_sensors)
        self.scheduler.register("altitude", self.sim_params.altitude_hz, self._task_altitude)
        self.scheduler.register("commander", self.sim_params.commander_hz, self._task_commander)
        self.scheduler.register("control_and_physics", self.sim_params.base_hz, self._task_control_and_physics)

        self._publish_truth(0.0)
        self._task_sensors(0.0, 1.0 / self.sim_params.sensor_hz)
        self._task_altitude(0.0, 1.0 / self.sim_params.altitude_hz)

    def set_autopilot(self, reference_fn: Optional[Callable[[float], tuple]], shortest_path: bool = True) -> None:
        """reference_fn(t) -> q_ref tuple, or None to clear (fall back to manual hold)."""
        self.autopilot_fn = reference_fn
        self.autopilot_shortest_path = shortest_path

    def set_flight_mode(self, mode: FlightMode) -> None:
        """Sets the commander's mode and, for AUTO_* modes, wires the
        matching paper scenario in automatically. Entering an AUTO mode
        resets the vehicle to identity/t=0 first, since the paper's
        scenarios (step onset times, sine phase, flip ramp) are all defined
        relative to a fresh start -- matching how scenarios/*.py runs them.
        Switching between manual modes (STABILIZED <-> ALTITUDE) does not
        reset, so it doesn't interrupt interactive flying."""
        if is_autopilot_mode(mode):
            self.reset()
            self.set_autopilot(reference_fn_for(mode), shortest_path=shortest_path_for(mode))
        else:
            self.set_autopilot(None)
        self.commander.set_mode(mode)

    def reset(self) -> None:
        self.plant.reset()
        self.airframe.reset()
        self.altitude_ctrl.reset()
        self.manual_mapper.reset()
        self.target_altitude = 0.5
        self.scheduler.reset()
        self._publish_truth(0.0)

    # ---- scheduler tasks ---------------------------------------------

    def _publish_truth(self, t: float) -> None:
        self.bus.publish(topics.TOPIC_ATTITUDE, topics.VehicleAttitude(self.plant.quaternion, self.plant.omega), t)
        x, y, z = self.plant.position
        vx, vy, vz = self.plant.velocity
        self.bus.publish(topics.TOPIC_LOCAL_POSITION, topics.VehicleLocalPosition(x, y, z, vx, vy, vz), t)

    def _task_sensors(self, t: float, dt: float) -> None:
        """Sampled at the sensor rate with zero-order hold in between (NOT
        redrawn every base-rate tick -- redrawing would inject far more
        high-frequency noise energy than a real sensor; see plan finding 6)."""
        q_meas, w_meas = self.sensor_model.measure(self.plant.quaternion, self.plant.omega)
        self.bus.publish(topics.TOPIC_SENSOR, topics.SensorCombined(q_meas, w_meas), t)

    def _task_altitude(self, t: float, dt: float) -> None:
        stick = self.manual_input
        if self.commander.mode == FlightMode.ALTITUDE:
            self.target_altitude += (stick.throttle - 0.5) * 2.0 * MAX_CLIMB_RATE * dt
            self.target_altitude = max(0.1, self.target_altitude)
        thrust = self.altitude_ctrl.compute_thrust(
            self.target_altitude, self.plant.altitude, -self.plant.velocity[2], self.plant.quaternion, dt
        )
        self.bus.publish(topics.TOPIC_THRUST_SETPOINT, topics.ThrustSetpoint(thrust), t)

    def _task_commander(self, t: float, dt: float) -> None:
        self.commander.check_failsafe(self.plant.altitude)
        self.bus.publish(
            topics.TOPIC_STATUS,
            topics.VehicleStatus(self.commander.armed, self.commander.mode.value, self.commander.failsafe_reason),
            t,
        )

    def _attitude_setpoint(self, t: float, dt: float) -> tuple:
        if self.commander.mode in AUTO_MODES and self.autopilot_fn is not None:
            return tuple(self.autopilot_fn(t))
        return self.manual_mapper.attitude_reference(self.manual_input, dt)

    def _task_control_and_physics(self, t: float, dt: float) -> None:
        shortest_path = not (self.commander.mode in AUTO_MODES and self.autopilot_fn is not None) or self.autopilot_shortest_path
        self.attitude_ctrl.shortest_path = shortest_path

        q_ref = self._attitude_setpoint(t, dt)
        sensed = self.bus.get_value(topics.TOPIC_SENSOR)
        q_meas, w_meas = (sensed.q_meas, sensed.omega_meas) if sensed else (self.plant.quaternion, self.plant.omega)

        tau = compute_torque_scalar(self.attitude_ctrl, q_ref, q_meas, w_meas, self.gains.torque_limit)
        self.bus.publish(topics.TOPIC_ATTITUDE_SETPOINT, topics.AttitudeSetpoint(q_ref, shortest_path), t)
        self.bus.publish(topics.TOPIC_TORQUE_SETPOINT, topics.VehicleTorqueSetpoint(tau), t)

        if self.commander.mode == FlightMode.STABILIZED:
            thrust_cmd = self.manual_mapper.thrust_from_throttle(self.manual_input, self.vehicle.rotor_thrust_max)
        else:
            ts = self.bus.get_value(topics.TOPIC_THRUST_SETPOINT)
            thrust_cmd = ts.thrust if ts else self.vehicle.hover_thrust_total

        if not self.commander.armed:
            thrust_cmd = 0.0
            tau = (0.0, 0.0, 0.0)

        rotor_cmd, desat_scale = mix(thrust_cmd, tau, self.vehicle)
        self.bus.publish(topics.TOPIC_ACTUATOR_CONTROLS, topics.ActuatorControls(rotor_cmd, desat_scale), t)

        rotor_actual = self.airframe.actuate(rotor_cmd, dt)
        self.bus.publish(topics.TOPIC_ACTUATOR_OUTPUTS, topics.ActuatorOutputs(rotor_actual), t)

        Fz, tx, ty, tz = thrusts_to_wrench(rotor_actual, self.vehicle)
        self.plant.step(-Fz, (tx, ty, tz), dt)
        self._publish_truth(t + dt)

    # ---- convenience ----------------------------------------------------

    def euler(self) -> tuple:
        return tuple(to_euler(self.plant.quaternion))

    def run_for(self, seconds: float, realtime: bool = False) -> None:
        self.scheduler.run_for(seconds, realtime=realtime)
