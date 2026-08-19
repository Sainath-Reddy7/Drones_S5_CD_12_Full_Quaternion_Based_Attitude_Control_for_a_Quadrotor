"""CLI entry point.

  python -m pysitl.run --gcs                            interactive: physics + web dashboard
  python -m pysitl.run --mode auto_step --duration 15    headless: run one paper scenario, print a summary
"""
from __future__ import annotations

import argparse
import threading
import time

from .gcs.server import run_server
from .modes import FlightMode
from .params import SimParams
from .sim import Simulation


def _physics_loop(sim: Simulation, stop: threading.Event) -> None:
    """Runs in real time until `stop` is set -- used for --gcs sessions,
    where the flight duration isn't known up front (the pilot decides).

    Selecting an AUTO mode (or hitting Reset) rewinds scheduler.sim_time to 0.
    The wall-clock baseline must be re-established when that happens: otherwise
    (sim_time - sim_start) goes negative, `target` lands far in the past, `lag`
    is never positive, and the loop stops sleeping altogether -- sprinting at
    max CPU until sim_time catches back up. Measured before this fix: a
    scenario ran ~6x faster than real time after a single mode switch.
    """
    t_wall_start = time.perf_counter()
    sim_start = sim.scheduler.sim_time
    prev_sim_t = sim_start

    # Step physics in batches rather than sleeping after every single 62.5 us
    # tick: a per-tick sleep leaves almost no window for the GIL to transfer,
    # starving the HTTP/SSE thread. Measured telemetry delivery improved from
    # ~2 Hz to ~6 Hz with batching.
    #
    # ~6 Hz is roughly the ceiling here, not a tuning miss: sweeping the batch
    # size (5/15/30 ms) moved it only between 5.2 and 6.0 Hz, so the limit is
    # raw CPU rather than sleep granularity. The attitude loop calls the
    # paper's real numpy NonlinearP2Controller 16000x/second (~34 us/call vs
    # ~5.6 us for equivalent scalar code -- see control/attitude.py for why
    # that tradeoff was taken deliberately), which keeps this thread busy for
    # most of a core and leaves the GIL scarce. Lowering sys.setswitchinterval
    # was also tried and made things worse (3.5 Hz, and RTF fell to 0.86x).
    # The dashboard stays smooth enough at this rate; real-time accuracy of
    # the simulation itself is unaffected (RTF holds ~1.0x).
    batch = max(1, int(0.015 / sim.scheduler.base_dt))  # ~15 ms of sim per batch

    while not stop.is_set():
        for _ in range(batch):
            sim.scheduler.tick()
            if stop.is_set():
                break
        sim_t = sim.scheduler.sim_time
        if sim_t < prev_sim_t:  # rewind detected -- re-baseline the pacing
            t_wall_start = time.perf_counter()
            sim_start = sim_t
        prev_sim_t = sim_t
        target = t_wall_start + (sim_t - sim_start)
        lag = target - time.perf_counter()
        if lag > 0:
            time.sleep(lag)


def main() -> None:
    parser = argparse.ArgumentParser(description="pysitl -- PX4-style quadrotor SITL")
    parser.add_argument("--gcs", action="store_true", help="start the web dashboard and fly interactively")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--mode",
        choices=[m.value.lower() for m in FlightMode],
        default=None,
        help="headless: run a single flight mode (typically auto_step/auto_sine/auto_flip)",
    )
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--log", action="store_true", help="headless: save CSV + analysis plots to results/")
    args = parser.parse_args()

    sim = Simulation(sim_params=SimParams(seed=args.seed))

    if args.gcs:
        httpd = run_server(sim, host=args.host, port=args.port)
        print(f"pysitl dashboard: http://{args.host}:{args.port}")
        sim.commander.try_arm(sim.plant.altitude)
        sim.set_flight_mode(FlightMode.ALTITUDE)
        stop = threading.Event()
        try:
            _physics_loop(sim, stop)
        except KeyboardInterrupt:
            stop.set()
            httpd.shutdown()
        return

    if args.mode:
        mode = FlightMode[args.mode.upper()]
        recorder = None
        if args.log:
            from .recorder import Recorder

            recorder = Recorder(sim)
            recorder.attach()

        sim.commander.try_arm(sim.plant.altitude)
        sim.set_flight_mode(mode)
        sim.run_for(args.duration)
        e = sim.euler()
        print(
            f"{mode.value}: t={sim.scheduler.sim_time:.2f}s alt={sim.plant.altitude:.3f}m "
            f"euler=({e[0]:+.4f},{e[1]:+.4f},{e[2]:+.4f})"
        )

        if recorder is not None:
            from .analysis import plot_flight

            path = recorder.save_csv(args.mode)
            plot_flight(recorder.to_dataframe(), mode.value, path.with_suffix(""))
            print(f"wrote {path} and analysis plots")
        return

    parser.print_help()


if __name__ == "__main__":
    main()
