"""Model, design, estimator, controllers, safety. No hardware, no network."""
import math
import unittest
from dataclasses import replace

import numpy as np

from cartpole import dynamics as dyn
from cartpole import lin
from cartpole.estimator import Estimator, FilteredDerivative
from cartpole.params import Plant
from cartpole.safety import Safety, SafetyConfig
from cartpole.simulate import run


class Model(unittest.TestCase):
    def test_linearisation_matches_finite_differences(self):
        p = Plant()
        A, B = dyn.linearize(p)
        An, Bn = dyn.linearize_numeric(p)
        np.testing.assert_allclose(A, An, atol=1e-6)
        np.testing.assert_allclose(B, Bn, atol=1e-6)

    def test_pdf_numbers(self):
        """The PDF (cartpole_model_solved.pdf, section 7): eig 0, 6.60, -5.97, -13.47."""
        p = replace(Plant(), B_f=0, B_p=0, rotor_mass=False)
        A, B = dyn.linearize(p)
        ev = np.sort(np.linalg.eigvals(A).real)
        np.testing.assert_allclose(ev, [-13.4655, -5.9732, 0.0, 6.6035], atol=2e-3)
        np.testing.assert_allclose(B.ravel(), [0, 0, 2.864, 12.082], atol=2e-3)

    def test_energy_conserved_without_friction_or_input(self):
        p = replace(Plant(), B_f=0, B_p=0, F_coul=0, K_m=0.0, K_t=0.0)
        P = dyn.PlantArrays(p)
        s = np.array([0.0, 2.0, 0.0, 0.0])
        E0 = dyn.energy(s, p)
        for _ in range(2000):
            s = dyn.rk4(s, 0.0, P)
        # cart moves freely, the pole's energy in the cart frame is not conserved
        # alone; total mechanical energy is
        T = 0.5 * p.M_tot * s[2] ** 2 - p.ml * math.cos(s[1]) * s[2] * s[3] + 0.5 * p.J * s[3] ** 2
        Etot = T + p.ml * p.g * (math.cos(s[1]) - 1)
        self.assertAlmostEqual(Etot, E0, places=4)


class Design(unittest.TestCase):
    def setUp(self):
        self.A, self.B = dyn.linearize(Plant())

    def test_lqr_stabilises_and_matches_discrete(self):
        Q, R = np.diag([10, 40, 0, 0.0]), np.array([[0.1]])
        K, _ = lin.lqr(self.A, self.B, Q, R)
        self.assertTrue(np.all(np.linalg.eigvals(self.A - self.B @ K).real < 0))
        Ad, Bd = lin.c2d(self.A, self.B, 0.002)
        Kd, _ = lin.dlqr(Ad, Bd, Q * 0.002, R * 0.002)
        np.testing.assert_allclose(K, Kd, rtol=0.05)

    def test_place(self):
        K = lin.place(self.A, self.B, [-5, -6, -7, -8])
        np.testing.assert_allclose(np.sort(np.linalg.eigvals(self.A - self.B @ K).real), [-8, -7, -6, -5], atol=1e-6)

    def test_ctrb_obsv(self):
        C = np.array([[1, 0, 0, 0], [0, 1, 0, 0.0]])
        self.assertEqual(np.linalg.matrix_rank(lin.ctrb(self.A, self.B)), 4)
        self.assertEqual(np.linalg.matrix_rank(lin.obsv(self.A, C)), 4)


class Estimation(unittest.TestCase):
    def test_derivative_filter_slope(self):
        f = FilteredDerivative()
        y = [f(0.5 * k * 0.002) for k in range(1500)]
        self.assertAlmostEqual(y[-1], 0.5, places=4)

    def test_kalman_tracks_velocity(self):
        e = Estimator("kalman")
        for k in range(3000):
            s = e(0.1 * k * 0.002, -math.pi + 2.0 * k * 0.002)
        self.assertAlmostEqual(s[2], 0.1, places=2)
        self.assertAlmostEqual(s[3], 2.0, places=1)


class Controllers(unittest.TestCase):
    def test_balancers_recover_from_a_tap(self):
        p = Plant()
        for key, prm in [("pid", {}), ("statefb", {}), ("statefb", {"method": "place"}),
                         ("statefb", {"time": "continuous"})]:
            log, sm, _ = run(key, p, p, 7, params=prm, s0=[0, 0.05, 0, 0], pokes=[(3, 1.0)])
            self.assertTrue(sm["balanced_at_end"], (key, prm, sm))
            self.assertLess(sm["max_abs_x_cm"], 15, (key, prm))

    def test_swing_ups(self):
        p = Plant()
        for key in ("energy", "sine"):
            log, sm, _ = run(key, p, p, 12)
            self.assertTrue(sm["balanced_at_end"], (key, sm))
            self.assertLess(sm["t_upright"], 6.0)

    def test_swing_up_with_model_mismatch(self):
        for pt in (replace(Plant(), B_f=3.5), replace(Plant(), k_amp=0.9), replace(Plant(), delay_steps=1)):
            log, sm, _ = run("energy", Plant(), pt, 12)
            self.assertTrue(sm["balanced_at_end"], (pt, sm))

    def test_balancing_is_quiet(self):
        log, sm, _ = run("statefb", Plant(), Plant(), 8, s0=[0, 0.03, 0, 0])
        self.assertLess(float(np.sqrt(np.mean(log[-2000:, 8] ** 2))), 0.6)


class SafetyLayer(unittest.TestCase):
    def test_barrier_keeps_the_cart_on_the_track(self):
        p = Plant()
        saf = Safety(p, SafetyConfig())
        P = dyn.PlantArrays(p)
        s = np.array([0.0, -math.pi, 0.0, 0.0])
        xmax = 0
        for _ in range(3000):                     # a controller that always wants +10 V
            v, f = saf(s, 10.0)
            self.assertIsNone(f)
            s = dyn.rk4(s, v, P)
            xmax = max(xmax, s[0])
        self.assertLess(xmax, SafetyConfig().x_soft + 0.01)

    def test_fault_beyond_hard_limit_and_voltage_cap(self):
        saf = Safety(Plant(), SafetyConfig(v_cap=4.0, cbf=False))
        v, f = saf(np.array([0.195, 0, 0, 0]), 5.0)
        self.assertEqual(v, 0.0)
        self.assertIsNotNone(f)
        v, f = saf(np.array([0.0, 0, 0, 0]), 9.0)
        self.assertEqual(v, 4.0)


if __name__ == "__main__":
    unittest.main()
