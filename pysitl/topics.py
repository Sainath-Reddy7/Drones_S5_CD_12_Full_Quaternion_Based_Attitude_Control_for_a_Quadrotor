"""uORB-style topic definitions: typed messages + the topic-name constants
modules publish/subscribe under. Named and shaped after PX4's own uORB
messages (vehicle_attitude, vehicle_local_position, sensor_combined,
actuator_controls, manual_control_setpoint, vehicle_status) so the workflow
matches PX4's, even though the underlying physics/control content is
paper-only (no estimator: SensorCombined carries the paper's own noisy
quaternion+rate measurements directly, not raw accel/gyro).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class VehicleAttitude:
    """Ground truth. In real PX4 this would be the ESTIMATE; this sim has no
    estimator (matches the paper, which has none), so control runs on
    SensorCombined below instead, and this topic exists for logging/the GCS."""

    q: tuple
    omega: tuple


@dataclass
class VehicleLocalPosition:
    """Ground truth, NED."""

    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


@dataclass
class SensorCombined:
    """The paper's own measurement model: uniform +/-amplitude noise on the
    quaternion and body rates, nothing else -- no accel/gyro split, no bias,
    no estimator. This is what the attitude controller actually reads."""

    q_meas: tuple
    omega_meas: tuple


@dataclass
class ManualControlSetpoint:
    roll: float
    pitch: float
    yaw_rate: float
    throttle: float


@dataclass
class AttitudeSetpoint:
    q_ref: tuple
    shortest_path: bool


@dataclass
class VehicleTorqueSetpoint:
    """Controller output, already clipped to the paper's +/-torque limit."""

    tau: tuple


@dataclass
class ThrustSetpoint:
    """Collective thrust command, pre-mixer."""

    thrust: float


@dataclass
class ActuatorControls:
    """Post-mixer, pre-actuator-lag commanded rotor thrusts."""

    rotor_cmd: tuple
    desat_scale: float


@dataclass
class ActuatorOutputs:
    """Post actuator-lag: what the plant actually sees."""

    rotor_thrust: tuple


@dataclass
class VehicleStatus:
    armed: bool
    mode: str
    failsafe_reason: str | None = None


TOPIC_ATTITUDE = "vehicle_attitude"
TOPIC_LOCAL_POSITION = "vehicle_local_position"
TOPIC_SENSOR = "sensor_combined"
TOPIC_MANUAL = "manual_control_setpoint"
TOPIC_ATTITUDE_SETPOINT = "vehicle_attitude_setpoint"
TOPIC_TORQUE_SETPOINT = "vehicle_torque_setpoint"
TOPIC_THRUST_SETPOINT = "vehicle_thrust_setpoint"
TOPIC_ACTUATOR_CONTROLS = "actuator_controls"
TOPIC_ACTUATOR_OUTPUTS = "actuator_outputs"
TOPIC_STATUS = "vehicle_status"
