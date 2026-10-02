"""Tests for the differential-flatness + double-loop LQR implementation
(Choutri & Lagha 2017)."""
import numpy as np
import pytest

from quat_sitl import quaternion as quat

from flatness.flatness import attitude_error, dcm_to_quat, flatness_reference
from flatness.lqr import AttitudeLQR, PositionLQR, lqr_gain
from flatness.model import G, QuadrotorModel, Vehicle
from flatness.trajectories import CircleTrajectory, FlatRef

V = Vehicle()


def ref_of(a, psi=0.0):
    return FlatRef(p=np.zeros(3), v=np.zeros(3),
                   a=np.asarray(a, float), psi=psi, psi_dot=0.0)


class TestFlatnessMap:
    """Eqs. (15), (19)-(21): (T_d, q_d) from the flat outputs."""

    def test_level_hover(self):
        T_d, q_d = flatness_reference(ref_of([0, 0, 0]), V)
        assert T_d == pytest.approx(V.mass * G)
        assert np.allclose(q_d, [1, 0, 0, 0], atol=1e-12)

    def test_body_z_along_thrust(self):
        rng = np.random.default_rng(3)
        for _ in range(50):
            a = rng.uniform(-8, 8, 3)
            psi = rng.uniform(-np.pi, np.pi)
            T_d, q_d = flatness_reference(ref_of(a, psi), V)
            f = V.mass * (a + np.array([0, 0, G]))
            d = f / np.linalg.norm(f)
            assert T_d == pytest.approx(np.linalg.norm(f))
            assert np.allclose(quat.to_dcm(q_d) @ [0, 0, 1], d, atol=1e-10)

    def test_yaw_preserved_when_level(self):
        for psi in (-2.5, -0.4, 0.0, 1.3, 2.9):
            _, q_d = flatness_reference(ref_of([0, 0, 0], psi), V)
            assert quat.to_euler(q_d)[2] == pytest.approx(psi, abs=1e-10)

    def test_feedforward_consistency(self):
        """Instantaneous application of (T_d, q_d) produces exactly the flat
        output's acceleration: R(q_d) z T_d / m - g = a_ref."""
        rng = np.random.default_rng(7)
        for _ in range(30):
            a = rng.uniform(-6, 6, 3)
            psi = rng.uniform(-np.pi, np.pi)
            T_d, q_d = flatness_reference(ref_of(a, psi), V)
            thrust = quat.to_dcm(q_d) @ np.array([0, 0, T_d / V.mass])
            assert np.allclose(thrust - np.array([0, 0, G]), a, atol=1e-9)


class TestDcmToQuat:
    def test_roundtrip_random_rotations(self):
        rng = np.random.default_rng(11)
        for _ in range(100):
            q = quat.normalize(rng.normal(size=4))
            R = quat.to_dcm(q)
            assert np.allclose(quat.to_dcm(dcm_to_quat(R)), R, atol=1e-9)


class TestMixer:
    """Eq. (9): feasible wrenches round-trip; infeasible torques are scaled
    with the thrust preserved exactly (priority mixing)."""

    def test_roundtrip_feasible(self):
        m = QuadrotorModel(V)
        for _ in range(20):
            rng = np.random.default_rng(_)
            # roll/pitch inside the hover arm headroom; yaw authority is far
            # smaller (km = 0.016 kf), so keep |tau_z| ~ 0.01 N*m
            tau = np.array([rng.uniform(-0.03, 0.03),
                            rng.uniform(-0.03, 0.03),
                            rng.uniform(-0.01, 0.01)])
            T = V.mass * G + rng.uniform(-0.2, 0.5)
            w = m.mix(T, tau)
            T2, tau2 = m.wrench(w)
            assert T2 == pytest.approx(T, abs=1e-9)
            assert np.allclose(tau2, tau, atol=1e-9)

    def test_thrust_preserved_under_infeasible_torque(self):
        m = QuadrotorModel(V)
        w = m.mix(V.mass * G, np.array([2.0, -2.0, 1.0]))  # way beyond hover
        T2, _ = m.wrench(w)
        assert T2 == pytest.approx(V.mass * G, abs=1e-9)
        assert w.min() >= 0.0 and w.max() <= V.w_max

    def test_hover_is_equilibrium(self):
        m = QuadrotorModel(V)
        m.reset(p0=(0.3, -0.2, 1.0))
        for _ in range(3000):
            m.step(0.001, V.mass * G, np.zeros(3))
        assert np.linalg.norm(m.p - [0.3, -0.2, 1.0]) < 1e-6
        assert abs(np.linalg.norm(m.q) - 1.0) < 1e-9


class TestLQR:
    def test_stabilizing_gains(self):
        """Closed A - B K eigenvalues strictly in the left half plane."""
        pos = PositionLQR()
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        eig = np.linalg.eigvals(A - B @ pos.K)
        assert np.all(eig.real < 0)
        att = AttitudeLQR()
        eig = np.linalg.eigvals(att.A - att.B @ att.K)
        assert np.all(eig.real < 0)

    def test_attitude_loop_converges(self):
        """30-deg tilt on the full Eq. (8) model recovers to <2 deg in 3 s."""
        m = QuadrotorModel(V)
        m.reset(p0=(0, 0, 5.0), q0=quat.from_axis_angle([0, 1, 0.0], np.radians(30)))
        inner = AttitudeLQR()
        T = V.mass * G
        q_d = np.array([1.0, 0, 0, 0])
        tau = np.zeros(3)
        for k in range(3000):
            if k % 5 == 0:
                qv, _ = attitude_error(q_d, m.q)
                tau = inner.torque(qv, m.omg)
            m.step(0.001, T, tau)
        err = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q_d, m.q))))))
        assert err < 2.0


class TestCircleStudyCase:
    """The paper's Section V closed loop: circle radius 1 m at 1 m altitude,
    double-loop LQR at 200 Hz — steady-state radial RMS below 5 cm."""

    def test_closed_loop_tracking(self):
        from flatness.run_paper import run
        s = run(cycles=0.75, out_root=None)
        assert s["rms_radial_m"] < 0.05, s
        assert s["rms_z_m"] < 0.02, s
        assert s["max_rotor_thrust_n"] <= V.rotor_thrust_max + 1e-9

    def test_reference_starts_at_rest_on_start_point(self):
        traj = CircleTrajectory()
        ref = traj.evaluate(0.0)
        assert np.allclose(ref.p, [-1.0, 0.0, 0.0], atol=1e-12)
        assert np.linalg.norm(ref.v) < 1e-12


class TestMuJoCoAdapter:
    """The same flatness + double-loop LQR stack flying the repo's MuJoCo
    contact physics (sim/mujoco/quadrotor.xml, 20 kHz) — cross-simulator
    validation: tracking must match the ideal Eq. (8) model's accuracy."""

    def test_closed_loop_tracking_mujoco(self):
        mujoco = pytest.importorskip("mujoco")
        from flatness.run_mujoco import run
        s = run(cycles=0.75, out_root=None)
        assert s["rms_radial_m"] < 0.06, s
        assert s["rms_z_m"] < 0.02, s
        assert s["max_rotor_thrust_n"] <= V.rotor_thrust_max + 1e-9
