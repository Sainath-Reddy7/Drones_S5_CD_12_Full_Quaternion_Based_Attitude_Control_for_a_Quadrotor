"""Manual flight in MuJoCo: YOU fly, the paper's controller is the pilot.

Keys move the ATTITUDE/ALTITUDE REFERENCE; the frozen paper law (gains
Pq=20, Pw=4, +/-4 N*m) always closes the loop -- exactly how a pilot's
sticks work on a real drone with this control law in the loop.

    W / S       pitch forward / back        R     reset to hover
    A / D       roll  left / right          Esc   quit
    Q / E       yaw   left / right
    Up / Down   climb / descend

The global `keyboard` listener works while the MuJoCo viewer has focus.
Usage:  python -m sim.mujoco.manual
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from quat_sitl import quaternion as quat
from sim.common import (
    AltitudeHold,
    PAPER_VEHICLE,
    QuatBridge,
    thrusts_to_wrench_zup,
)

XML_PATH = Path(__file__).with_name("quadrotor.xml")
VEHICLE_MASS = 0.2
MAX_TILT = 0.45  # rad pilot authority (~26 deg) -- the law stays smooth
YAW_RATE = 1.2  # rad/s
CLIMB = 0.8  # m/s
POLL_EVERY = 400  # keyboard poll interval in physics steps (20 ms / 50 Hz)
HUD_EVERY = 10_000  # HUD print interval (0.5 s)


def main() -> None:
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    dt = float(model.opt.timestep)

    import keyboard  # global listener; install: pip install keyboard

    bridge = QuatBridge(geometry=PAPER_VEHICLE, shortest_path=True)
    alt = AltitudeHold(max_thrust_factor=2.2)

    state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "z_ref": 1.5, "run": True}
    keyboard.on_press_key("esc", lambda _: state.__setitem__("run", False))

    def reset() -> None:
        data.qpos[:] = [0, 0, 1.5, 1, 0, 0, 0]
        data.qvel[:] = 0
        data.qacc[:] = 0
        state.update(roll=0.0, pitch=0.0, yaw=0.0, z_ref=1.5)
        alt.reset()

    print(__doc__)
    print(">>> click the MuJoCo window and FLY. Keys work from any window. Esc quits.\n")
    reset()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        k = 0
        t0 = time.time()
        while state["run"] and viewer.is_running():
            # --- pilot input at 50 Hz --------------------------------------
            if k % POLL_EVERY == 0:
                if keyboard.is_pressed("r"):
                    reset()
                tgt_p = (0.35 if keyboard.is_pressed("w") else 0) - (0.35 if keyboard.is_pressed("s") else 0)
                tgt_r = (0.35 if keyboard.is_pressed("d") else 0) - (0.35 if keyboard.is_pressed("a") else 0)
                # smooth the reference like a real stick
                state["pitch"] += 0.35 * (tgt_p - state["pitch"])
                state["roll"] += 0.35 * (tgt_r - state["roll"])
                state["roll"] = float(np.clip(state["roll"], -MAX_TILT, MAX_TILT))
                state["pitch"] = float(np.clip(state["pitch"], -MAX_TILT, MAX_TILT))
                if keyboard.is_pressed("q"):
                    state["yaw"] += YAW_RATE * POLL_EVERY * dt
                if keyboard.is_pressed("e"):
                    state["yaw"] -= YAW_RATE * POLL_EVERY * dt
                climb = (1 if keyboard.is_pressed("up") else 0) - (1 if keyboard.is_pressed("down") else 0)
                state["z_ref"] = float(np.clip(state["z_ref"] + climb * CLIMB * POLL_EVERY * dt, 0.25, 45.0))

            # --- paper controller, every step (20 kHz) ----------------------
            q_ref = quat.from_euler(state["roll"], state["pitch"], state["yaw"])
            q_sim = np.array(data.qpos[3:7])  # w-first, body->world
            q_m = quat.conj(q_sim)
            R_bw = quat.to_dcm(q_sim)
            omega = np.array(data.qvel[3:6])  # local angular velocity

            tau, _sat = bridge.torque(q_ref, q_m, omega)
            tilt_cos = float(np.clip(R_bw[2, 2], -1.0, 1.0))
            thrust = alt.thrust(
                state["z_ref"], float(data.qpos[2]), float(data.qvel[2]),
                tilt_cos, VEHICLE_MASS, PAPER_VEHICLE.thrust_max_total, dt,
            )
            thrusts, _desat = bridge.mix(thrust, tau)
            fz, tau_actual = thrusts_to_wrench_zup(thrusts, PAPER_VEHICLE)
            data.xfrc_applied[body_id, 0:3] = R_bw @ np.array([0.0, 0.0, fz])
            data.xfrc_applied[body_id, 3:6] = R_bw @ tau_actual

            mujoco.mj_step(model, data)

            if k % 40 == 0:
                viewer.sync()
            if k % HUD_EVERY == 0:
                phi, th, ps = quat.to_euler(q_m)
                print(f"  z={data.qpos[2]:5.2f} m  roll={np.degrees(phi):+6.1f}  "
                      f"pitch={np.degrees(th):+6.1f}  yaw={np.degrees(ps):+7.1f}  "
                      f"({time.time()-t0:5.0f}s)")
            k += 1

    print("bye -- safe landing not guaranteed, physics was honest")


if __name__ == "__main__":
    main()
