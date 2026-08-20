"""ULog-equivalent structured logging: a scheduler task that samples the bus
at sim_params.logger_hz and appends one wide row (every topic flattened) per
tick to an in-memory buffer, exportable to CSV/DataFrame. Deliberately close
in spirit and column-naming to quat_sitl.simulator's own CSV logging, so
results from both packages read the same way.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

from . import topics
from .sim import Simulation
from quat_sitl.quaternion import to_euler

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


@dataclass
class Recorder:
    sim: Simulation
    rows: list = field(default_factory=list)
    _attached: bool = False

    def attach(self) -> None:
        """Registers the logger task on the sim's scheduler. Call once,
        before running -- registering twice would double-log."""
        if self._attached:
            return
        self.sim.scheduler.register("logger", self.sim.sim_params.logger_hz, self._log_tick)
        self._attached = True

    def _log_tick(self, t: float, dt: float) -> None:
        b = self.sim.bus
        att = b.get_value(topics.TOPIC_ATTITUDE)
        pos = b.get_value(topics.TOPIC_LOCAL_POSITION)
        sp = b.get_value(topics.TOPIC_ATTITUDE_SETPOINT)
        tq = b.get_value(topics.TOPIC_TORQUE_SETPOINT)
        act = b.get_value(topics.TOPIC_ACTUATOR_OUTPUTS)
        status = b.get_value(topics.TOPIC_STATUS)
        sensed = b.get_value(topics.TOPIC_SENSOR)
        if att is None or pos is None:
            return

        phi, theta, psi = to_euler(np.array(att.q))
        q_ref = sp.q_ref if sp else (1.0, 0.0, 0.0, 0.0)
        phi_r, theta_r, psi_r = to_euler(np.array(q_ref))
        tau = tq.tau if tq else (0.0, 0.0, 0.0)
        rotors = act.rotor_thrust if act else (0.0, 0.0, 0.0, 0.0)
        q_meas = sensed.q_meas if sensed else att.q

        self.rows.append(
            {
                "time": t,
                "x": pos.x, "y": pos.y, "z": pos.z, "altitude": -pos.z,
                "vx": pos.vx, "vy": pos.vy, "vz": pos.vz,
                "q0": att.q[0], "q1": att.q[1], "q2": att.q[2], "q3": att.q[3],
                "wx": att.omega[0], "wy": att.omega[1], "wz": att.omega[2],
                "q_meas0": q_meas[0], "q_meas1": q_meas[1], "q_meas2": q_meas[2], "q_meas3": q_meas[3],
                "q_ref0": q_ref[0], "q_ref1": q_ref[1], "q_ref2": q_ref[2], "q_ref3": q_ref[3],
                "phi": phi, "theta": theta, "psi": psi,
                "phi_ref": phi_r, "theta_ref": theta_r, "psi_ref": psi_r,
                "tau_x": tau[0], "tau_y": tau[1], "tau_z": tau[2],
                "rotor1": rotors[0], "rotor2": rotors[1], "rotor3": rotors[2], "rotor4": rotors[3],
                "armed": status.armed if status else False,
                "mode": status.mode if status else "",
            }
        )

    def to_dataframe(self) -> pd.DataFrame:
        return pd.DataFrame(self.rows)

    def save_csv(self, name: str) -> Path:
        RESULTS_DIR.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        path = RESULTS_DIR / f"pysitl_{name}_{timestamp}.csv"
        df = self.to_dataframe()
        df.to_csv(path, index=False)
        df.to_csv(RESULTS_DIR / f"pysitl_{name}_latest.csv", index=False)
        return path
