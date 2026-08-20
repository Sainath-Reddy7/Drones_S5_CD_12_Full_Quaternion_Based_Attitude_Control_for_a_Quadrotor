# pysitl Ground Station — UI Guide

## Launch

**Recommended:**

```bash
bash run_gcs.sh          # default port 8765
bash run_gcs.sh 8080      # custom port
```

This picks whichever Python interpreter on the machine actually has
`numpy`/`pandas`/`matplotlib` installed (some Macs have several interpreters
and only one with the right packages), refuses to start if the port is
already occupied by a stale server, and opens the dashboard in the default
browser automatically.

**Manual (if you already know your interpreter has the dependencies):**

```bash
python3 -m pysitl.run --gcs
```

Opens the server at `http://127.0.0.1:8765` (add `--port N` to change it, `--host`
to bind elsewhere). Open that URL in a browser. The vehicle boots **armed**, in
**Altitude** mode, hovering. Ctrl+C in the terminal stops the server.

## Layout

```
┌─────────────┬───────────────────────────────┬─────────────┐
│  LEFT RAIL  │        3D VIEWPORT             │ RIGHT RAIL  │
│  Arming     │   (drag to orbit, scroll zoom) │ Attitude    │
│  Flight mode│                                 │ chart       │
│  Gains      │        (HUD overlay)            │ Torque      │
│  Manual ctrl│                                 │ chart       │
│             ├───────────────────────────────┤ Readout      │
│             │  Saturation LEDs (Mx/My/Mz)     │ panel       │
└─────────────┴───────────────────────────────┴─────────────┘
```

## Left rail

**Arming**
- **Arm** — runs preflight checks (must be on the ground, altitude ≈ 0) and
  arms the vehicle. Rotors stay at zero thrust until armed.
- **Disarm** — immediately zeros thrust and torque, regardless of mode or
  stick input. Also what the failsafe triggers automatically.

**Flight mode**
| Button | Behavior |
|---|---|
| **Stabilized** | Sticks → attitude reference. Throttle → **direct** collective thrust (no altitude hold — climbs/descends freely with stick position). |
| **Altitude** | Sticks → attitude reference. Throttle → **climb-rate** command around a held target altitude (0.5 = hold, above/below climbs/descends). |
| **Auto Step** | Runs the paper's step scenario (1 rad step on φ, θ, ψ staggered over 15s). Resets the vehicle to t=0 on entry. |
| **Auto Sine** | Runs the paper's 0.5 rad / 1 rad/s sine-tracking scenario. Resets on entry. |
| **Auto Flip** | Runs the paper's 360° flip (shortest-path disabled — commits to the full rotation). Resets on entry. |
| **Reset** | Returns the vehicle to identity attitude/position at t=0, without changing arm state or mode. |

**Controller gains** — $P_q$ and $P_\omega$ sliders, live-adjustable, feed
directly into the actual `NonlinearP2Controller` (paper defaults: 20 / 4).
The **CTRL** readout (top-right of the viewport) shows the control-tick rate
the scheduler is running at — it recomputes automatically from whatever
$P_\omega$ is currently set, so dragging the slider up and watching CTRL
follow is a direct demonstration of the discrete-stability requirement
documented in `REPORT.md`.

**Manual controls**
- **Throttle slider** — drag or click to set throttle directly (0 to 1).
- Keyboard (page must have focus): `W`/`S` pitch, `A`/`D` roll, `Q`/`E` yaw
  rate, `↑`/`↓` throttle. Roll/pitch/yaw return to zero on key release;
  throttle holds wherever you leave it (matches a real transmitter).

## 3D viewport

- **Drag** to orbit the camera, **scroll** to zoom.
- Solid cyan/orange frame with colored motors = the real vehicle (front
  motors orange, rear cyan). Dashed amber outline = the current attitude
  reference — the gap between the two is the tracking error, live.
- Props spin at a rate proportional to actual commanded rotor thrust.
- **Coordinate axes** — two labeled triads are drawn so you can read
  orientation/position directly off the viewport instead of only the charts:
  - **World axes (dashed, faint, fixed at the origin)** — `X`/`Y`/`Z`, NED
    convention: X = north, Y = east, Z = down. These never move; they are
    your fixed reference frame.
  - **Body axes (solid, bright, attached to the vehicle)** — lowercase
    `x`/`y`/`z`, same red/green/blue coloring, rigidly attached to the
    airframe and rotating with it. Comparing the tilt of the solid body
    triad against the dashed world triad is a direct visual read of the
    current attitude, independent of the charts.
- HUD (top corners): flight mode, control-tick rate, current/target altitude,
  live **POS** readout (`N <north> E <east>`, meters, world frame — this is
  where the vehicle has actually translated to, useful in AUTO scenarios
  since they only hold attitude/altitude and never correct drift), and
  simulation time.
- Below the viewport: three saturation LEDs (Mx/My/Mz) — light up red when
  that axis's torque command is hitting the ±4 N·m clip.

## Right rail

- **Attitude chart** — φ/θ/ψ vs. time, last ~12s scrolling window. **Dashed =
  reference, solid = output**, same convention as the paper's Figs. 3/5 — the
  gap between the pair is the tracking error. In **Auto Flip** the chart
  switches automatically to raw `q0`/`q1`, because wrapped Euler φ jumps
  discontinuously at ±π through a full rotation (an `atan2` artifact, not a
  controller fault — the paper pairs its Fig. 7 with Fig. 8 for exactly this
  reason).
- **Torque chart** — commanded Mx/My/Mz vs. time, same window. With the
  paper's noise amplitude (0.1) and gain ($P_q=20$), several N·m of
  noise-driven torque is expected and real, not a display fault — it is why
  this simulation's torque traces are busier than the paper's Fig. 6.

Both charts **auto-scale their Y axis** to whatever is in the visible window
and print numeric axis labels. A single fixed scale cannot serve every mode —
the paper rescales per scenario too (Fig. 3 is −0.5…1.5 rad, Fig. 5 is
−0.5…0.5, Fig. 7 is −4…4) — and a fixed ±3.3 squashed the sine scenario, whose
real swing is only ±0.48 rad, into a near-flat line.

Note: the history buffer restarts whenever the simulation is reset (entering
an AUTO mode, or **Reset**), so the trace begins fresh from t=0 rather than
splicing onto the previous run.

Charts refresh at roughly 5–7 Hz rather than the nominal 50 Hz stream rate.
That is a CPU/GIL limit, not a fault: the attitude loop calls the paper's real
numpy controller 16000×/second, which keeps the physics thread busy for most
of a core. The simulation itself still runs at ~1.0× real time — only the
telemetry refresh is coarser. See `run.py:_physics_loop` for the measurements.
- **Readout panel** — numeric snapshot: attitude error, |ω|, control rate,
  substep count, shortest-path flag, current gains.

## Safety behavior

- Arming requires the vehicle to already be on the ground (altitude ≈ 0);
  arming while airborne is refused.
- A bounds failsafe watches altitude continuously — if it leaves a sane
  envelope (below −1 m or above 50 m), the vehicle auto-disarms and a red
  **FAILSAFE** banner appears in the header with the reason. This is most
  likely to trigger in **Stabilized** mode: throttle there is direct thrust,
  so leaving it at the default 0.5 with no altitude hold can climb hard. Hit
  **Reset** then **Arm** to recover.

## Troubleshooting

- **Page stuck on "connecting to pysitl backend..."** — the server isn't
  running (or was killed). Restart with the launch command above and reload
  the page.
- **Buttons feel laggy** — shouldn't happen; a stale build might not have the
  HTTP keep-alive fix. Pull latest and restart the server.
- **`ModuleNotFoundError: No module named 'numpy'`** — the `python3` on your
  `PATH` doesn't have the project's dependencies. Use `bash run_gcs.sh`
  instead of calling `pysitl.run` directly; it searches known interpreter
  locations for one that has `numpy`/`pandas`/`matplotlib` and uses that.
- **`Port 8765 is already in use`** — `run_gcs.sh` refuses to start rather
  than fail confusingly later. A previous server is likely still running;
  stop it with `lsof -ti tcp:8765 | xargs kill`, or launch on a different
  port with `bash run_gcs.sh 8080`.
