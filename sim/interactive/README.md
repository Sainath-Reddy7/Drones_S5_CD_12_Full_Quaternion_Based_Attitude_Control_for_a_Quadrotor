# Interactive MuJoCo Drone Simulator

Live, real-time MuJoCo simulation with **manual** and **our-control** modes,
an urban testing environment (buildings, trees, roads, cars, trucks, street
lamps), collision detection + logging, three cameras, and a live telemetry
HUD. One law always flies: the paper's unmodified quaternion controller
(Pq=20, Pω=4, ±4 N·m) — the mode decides only who commands the reference.

## Launch

```bash
pip install mujoco keyboard          # once
python -m sim.interactive.app
```
(On the team machine: `.venv310\Scripts\python.exe -m sim.interactive.app`,
or double-click the desktop icon **DRONE SIM - INTERACTIVE (click me).bat**.)

## Controls

| Keys | Action | | Keys | Action |
|---|---|---|---|---|
| `W/S` | pitch fwd/back | | `M` | switch MANUAL ⇄ OUR CONTROL |
| `A/D` | roll left/right | | `G` | start mission (our control) |
| `Q/E` | yaw left/right | | `H` | hold (our control) |
| `↑/↓` or `R/F` | climb / descend | | `L` | land (our control) |
| `Space` | hover (zero refs) | | `1/2/3` or `C` | free / follow / top camera |
| `Esc` | quit | | | |

Keys work while the 3D window has focus (global listener).

## Modes

- **MANUAL** — your keys become attitude/altitude references; the paper law
  tracks them at 1 kHz (a pilot's sticks, with the paper controller in the loop).
- **OUR CONTROL** — a mission state machine (takeoff → waypoints over the
  city corridor → return → land) generates references through a guidance
  layer (position → velocity → tilt). The guidance is scenario scaffolding,
  exactly like the step/sine/flip generators — the controller itself is the
  unmodified paper law. `M` hands over safely: references initialize from
  the drone's current state.

## Environment

Buildings (6, collision), trees (8, trunk+canopy collision), cars (4) and
trucks (2) on a two-road network with lane dashes and sidewalks, street
lamps, a helipad launch pad. Collisions are detected per physics step,
shown in the HUD (`!! COLLISION: <object>`), and logged to
`sim/interactive/collisions.csv` on exit. Ground/pad touches count as
landings, not crashes.

## Physics notes

- Drone = the exact benchmark paper vehicle (0.2 kg, paper inertia).
- Interactive physics 10 kHz (dt 1e-4): the repo's own fast-pole analysis
  puts empirical stability at fast_pole·dt ≲ 1.6 (here 0.62); benchmark
  adapters keep 20 kHz and the strict 12.3 kHz assertion.
- Controller + wrench refresh at 1 kHz with zero-order hold — the same
  rate as the paper's sensor model.
- Rendering uses the GPU when available (viewer auto-selects) with
  automatic CPU fallback.
