"""Multi-rate deterministic lockstep scheduler.

PX4 runs many modules (IMU driver, rate controller, attitude controller,
commander, logger...) each at its own fixed rate, all decimated from one
clock. This reproduces that: every module registers a callback and a rate
that must evenly divide base_hz; a single base-rate tick decimates into each
module's own cadence. Headless runs are fully deterministic (no wall-clock
coupling) unless real-time pacing is explicitly requested for interactive
flying.

base_hz must clear dynamics.stable_control_rate_hz for the configured gains
and inertia -- see plan finding 2 (the paper's Pw/inertia force a fast
closed-loop pole that an under-rate discrete loop cannot stabilize). This is
asserted at construction time rather than trusted.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from quat_sitl.dynamics import stable_control_rate_hz


@dataclass
class _Task:
    name: str
    hz: float
    period_ticks: int
    callback: Callable[[float, float], None]  # (sim_time, dt) -> None
    counter: int = 0


class Scheduler:
    def __init__(self, base_hz: float, Pw: float, inertia, margin: float = 0.5) -> None:
        floor_hz = stable_control_rate_hz(Pw, inertia, margin)
        if base_hz < floor_hz:
            raise ValueError(
                f"base_hz={base_hz} is below the numerically-required stable control "
                f"rate {floor_hz:.1f} Hz for Pw={Pw} -- the attitude loop would chatter "
                f"(see plan finding 2 / dynamics.stable_control_rate_hz)"
            )
        self.base_hz = base_hz
        self.base_dt = 1.0 / base_hz
        self.sim_time = 0.0
        self._tasks: list[_Task] = []

    def register(self, name: str, hz: float, callback: Callable[[float, float], None]) -> None:
        period_ticks = round(self.base_hz / hz)
        if abs(self.base_hz / period_ticks - hz) > 1e-6:
            raise ValueError(f"task '{name}' rate {hz} Hz does not evenly divide base {self.base_hz} Hz")
        self._tasks.append(_Task(name, hz, period_ticks, callback))

    def reset(self) -> None:
        """Rewind sim time and every task's decimation counter to zero."""
        self.sim_time = 0.0
        for task in self._tasks:
            task.counter = 0

    def tick(self) -> None:
        for task in self._tasks:
            if task.counter % task.period_ticks == 0:
                task.callback(self.sim_time, 1.0 / task.hz)
            task.counter += 1
        self.sim_time += self.base_dt

    def run_for(self, seconds: float, realtime: bool = False) -> None:
        n = round(seconds / self.base_dt)
        t_wall_start = time.perf_counter()
        sim_start = self.sim_time
        for _ in range(n):
            self.tick()
            if realtime:
                target = t_wall_start + (self.sim_time - sim_start)
                lag = target - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
