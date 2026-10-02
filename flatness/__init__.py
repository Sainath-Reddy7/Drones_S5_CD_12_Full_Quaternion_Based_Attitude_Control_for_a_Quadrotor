"""Differential flatness — quaternion trajectory tracking (Choutri & Lagha
2017, "Quadrotors Trajectory Tracking using a Differential Flatness-
Quaternion based Approach", IEEE). Implements the paper on the repo's
airframe: Eq. (8) model, Eq. (9) mixer, Eqs. (13)-(15) flat outputs, the
double-loop LQR of Eqs. (17)-(18) + (22)-(23), and the Section V circular
study case at the paper's 200 Hz control rate."""
from flatness.model import G, QuadrotorModel, Vehicle
from flatness.trajectories import CircleTrajectory, FlatRef
from flatness.flatness import attitude_error, dcm_to_quat, flatness_reference
from flatness.lqr import AttitudeLQR, PositionLQR, lqr_gain

__all__ = [
    "G", "QuadrotorModel", "Vehicle",
    "CircleTrajectory", "FlatRef",
    "attitude_error", "dcm_to_quat", "flatness_reference",
    "AttitudeLQR", "PositionLQR", "lqr_gain",
]
