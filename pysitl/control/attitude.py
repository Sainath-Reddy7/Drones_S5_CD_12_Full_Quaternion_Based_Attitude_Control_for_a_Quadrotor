"""Thin adapter over the paper's own controller. NonlinearP2Controller is
used completely UNMODIFIED (imported, not copied) -- this module only
converts between pysitl's scalar-tuple state and the numpy arrays that class
expects, and applies the paper's own torque clip.

Measured cost: calling the real numpy controller inside the 16 kHz scalar
physics loop still runs at ~1.85x real-time on this machine (33.8us/step,
vs. 5.6us/step for an all-scalar loop) -- slower than a hand-rolled scalar
reimplementation would be, but it clears real-time with margin, and it means
the vehicle is genuinely flown by the user's own unaltered algorithm rather
than a hopefully-equivalent copy. That tradeoff is deliberate: see plan
finding 2 for the all-scalar numbers this is weighed against.
"""
from __future__ import annotations

import numpy as np

from quat_sitl.controller import ControllerGains, NonlinearP2Controller

__all__ = ["NonlinearP2Controller", "ControllerGains", "compute_torque_scalar"]


def compute_torque_scalar(
    controller: NonlinearP2Controller,
    q_ref: tuple,
    q_m: tuple,
    omega_m: tuple,
    torque_limit: float,
) -> tuple:
    """q_ref/q_m/omega_m as plain tuples in; clipped torque tuple out."""
    tau = controller.compute_torque(np.array(q_ref), np.array(q_m), np.array(omega_m))
    return tuple(max(-torque_limit, min(torque_limit, float(t))) for t in tau)
