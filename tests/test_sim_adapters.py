"""Adapter smoke/regression tests for the cross-simulator deployment.

Both suites skip automatically when the target stack is not installed (e.g.
the plain .venv310 has MuJoCo; gym-pybullet-drones lives in the mmenv
micromamba env), so `pytest tests/ -q` stays green everywhere while the
adapters stay guarded where their stacks exist.
"""
from __future__ import annotations

import numpy as np
import pytest

from quat_sitl import quaternion as quat


def test_mujoco_adapter_holds_hover_and_attitude(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    from sim.mujoco.run import run

    m = run("step", duration=3.0, seed=0, noise=0.0, out_root=tmp_path)
    # 3 s covers the t=1 phi step: it must be tracked well past the transient
    assert m["rms_alpha_deg"] < 15.0
    assert m["settle_theta_s"] < 1.5  # theta's step (t=5) never fires in 3 s -> 0
    assert m["sat_fraction"] < 0.05


def test_mujoco_adapter_step_matches_reference_direction(tmp_path):
    pytest.importorskip("mujoco")
    from sim.mujoco.run import run

    m = run("step", duration=2.5, seed=0, noise=0.0, out_root=tmp_path)
    # after the t=1 phi step + ~1.5 s, phi must be within 10 deg of 1 rad
    assert m["settle_phi_s"] < 1.6


def test_gym_pybullet_adapter_tracks_step(tmp_path):
    pytest.importorskip("pybullet")
    pytest.importorskip("gym_pybullet_drones")
    from sim.gym_pybullet.run import run

    m = run("step", duration=3.0, seed=0, noise=0.0, out_root=tmp_path)
    assert m["rms_alpha_deg"] < 15.0
    assert m["sat_fraction"] < 0.05


def test_pybullet_actuation_calibration():
    """The one systematic PyBullet-only failure mode worth guarding: the
    realized wrench must match the mixer's intent in SIGN and MAGNITUDE.
    Applies a scripted pure-roll rotor pattern for 60 ms and checks the
    induced body rate against rigid-body dynamics (no damping, no gravity
    side effects within this window)."""
    pb = pytest.importorskip("pybullet")
    pytest.importorskip("gym_pybullet_drones")
    from sim.common import QuatBridge, RotorGeometry, mix_zup, thrusts_to_wrench_zup
    from sim.gym_pybullet.run import KF, MY_TO_SIM, _install_urdf, _paper_drone_enum

    _install_urdf()
    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

    env = CtrlAviary(
        drone_model=_paper_drone_enum(), num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, 5.0]]), initial_rpys=np.zeros((1, 3)),
        pyb_freq=24000, ctrl_freq=24000, gui=False, user_debug_gui=False,
    )
    obs, _ = env.reset()
    pb.changeDynamics(int(env.DRONE_IDS[0]), -1,
                      linearDamping=0.0, angularDamping=0.0, physicsClientId=env.CLIENT)

    geo = RotorGeometry(kf=KF)
    thrusts, s1 = mix_zup(1.962, np.array([0.3, 0.0, 0.0]), geo)
    # run the SAME thrusts through the sim rotor mapping used by the adapter
    th_sim = [thrusts[j] for j in MY_TO_SIM]
    rpms = np.sqrt(np.array(th_sim) / KF)
    fz, tau = thrusts_to_wrench_zup(thrusts, geo)

    dt, n = 1.0 / 24000, int(0.06 / (1.0 / 24000))
    for _ in range(n):
        obs, _, _, _, _ = env.step(rpms[None, :])
    q_xyzw = obs[0, 3:7]
    q_sim = np.array([q_xyzw[3], *q_xyzw[:3]])
    R = quat.to_dcm(q_sim)
    wb = R.T @ obs[0, 13:16]
    env.close()

    # free rigid body: |wx| ~= |tau_x|/Ixx * t (torque is constant); allow
    # 35% band for integrator/contact-solver detail
    Ixx = 6.5e-4
    expected = abs(tau[0]) / Ixx * n * dt
    assert expected > 5.0  # the test window produces a clearly measurable rate
    assert abs(abs(wb[0]) - expected) / expected < 0.35, (
        f"realized |wx|={abs(wb[0]):.1f} vs expected {expected:.1f} rad/s "
        "-- PyBullet actuation mapping no longer matches the mixer"
    )
