"""Course regression and deployment preparation, independent of real hardware."""
import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np

from cartpole.friction import estimate
from cartpole.params import Plant


def friction_record(B_f=3.3, F_coul=0.):
    p = Plant(); t = np.arange(0, 20, .002)
    x = .04*np.sin(2.4*t) + .015*np.sin(6.8*t)
    xd = .096*np.cos(2.4*t) + .102*np.cos(6.8*t)
    xdd = -.2304*np.sin(2.4*t) - .6936*np.sin(6.8*t)
    a = .3*np.sin(3.3*t); ad = .99*np.cos(3.3*t); add = -3.267*np.sin(3.3*t)
    force = p.M_tot*xdd - p.ml*np.cos(a)*add + p.ml*np.sin(a)*ad**2 + B_f*xd + F_coul*np.tanh(xd/.01)
    return {'t': t, 'x': x, 'alpha': a, 'force': force}


class Friction(unittest.TestCase):
    def test_handout_scalar_fit_recovers_viscous_friction(self):
        r = estimate(friction_record())
        self.assertEqual(r['exercise_dimension'], 1)
        self.assertTrue(r['accepted'])
        self.assertAlmostEqual(r['models']['viscous']['B_f'], 3.3, delta=.04)
        self.assertGreater(r['models']['viscous']['validation_r2'], .99)

    def test_coulomb_extension_selected_only_when_it_predicts_better(self):
        r = estimate(friction_record(F_coul=.25))
        self.assertTrue(r['accepted'])
        self.assertEqual(r['selected'], 'viscous_coulomb')
        self.assertAlmostEqual(r['params']['B_f'], 3.3, delta=.15)
        self.assertAlmostEqual(r['params']['F_coul'], .25, delta=.02)

    def test_validation_detects_changed_friction(self):
        data = friction_record(); changed = friction_record(B_f=20)
        data['force'][7000:] = changed['force'][7000:]
        self.assertFalse(estimate(data)['accepted'])

    def test_rejects_bad_timestamps_and_insufficient_motion(self):
        data = friction_record(); data['t'][500] += .01
        with self.assertRaisesRegex(ValueError, 'uniformly'):
            estimate(data)
        data = friction_record(); data['x'] *= 0
        with self.assertRaisesRegex(ValueError, 'moving samples'):
            estimate(data)


class Controllers(unittest.TestCase):
    def test_estimator_tuning_preserves_the_ppo_contract(self):
        from cartpole.runtime import Runtime
        from cartpole.backends.sim import SimBackend
        rt=Runtime(SimBackend(realtime=False),log=lambda _:None)
        rt.set_estimator('derivative',80)
        self.assertEqual(rt.est.w,80)
        self.assertIn('Kalman',rt.set_mode('ppo'))
        rt.set_mode('pid'); self.assertIsNone(rt.arm())
        with self.assertRaisesRegex(ValueError,'Stop'):
            rt.set_estimator('kalman')
        rt.disarm(); rt.set_estimator('kalman')
        self.assertIsNone(rt.set_mode('ppo'))

    def test_pid_freezes_integral_when_voltage_is_clamped(self):
        from cartpole.controllers import make
        from cartpole.controllers.base import Ref
        c=make('pid',Plant()); c.set_params({'ki_a':1.})
        for _ in range(100):
            v=c.step(np.array([0., .04, 0., 0.]),0.,Ref())
            c.observe_voltage(v,0.)
        self.assertEqual(c.ia,0.)
        c.step(np.array([0., 1., 0., 0.]),0.,Ref())
        c.observe_voltage(0.,1.)
        self.assertEqual(c.ia,0.)

    def test_manual_input_hands_over_to_lqr(self):
        from cartpole.controllers import make
        from cartpole.controllers.base import Ref
        c=make('manual_swing',Plant()); c.reset(None)
        self.assertEqual(c.step(np.array([0., np.pi, 0., 0.]),0.,Ref(manual=.5)),2.)
        v=c.step(np.array([0., .02, 0., 0.]),.002,Ref(manual=1.))
        self.assertEqual(c.info['phase'],'balancing')
        self.assertAlmostEqual(v,float(-(c.Kc@np.array([0.,.02,0.,0.]))[0]))

    def test_vectorized_search_matches_deployment_simulation(self):
        from cartpole.energy_search import batch
        from cartpole.robustness import scenarios,evaluate
        p=Plant(); cases=scenarios(p,1,17); params={'adapt':.3,'e_over':.2}
        fast=batch(p,[params],cases,4)[0]
        scalar=evaluate('energy',params,p,cases,10,4,lambda _:None)
        for a,b in zip(fast['cases'],scalar['cases']):
            self.assertEqual(a['balanced_at_end'],b['balanced_at_end'])
            self.assertEqual(a['t_upright'],b['t_upright'])
            self.assertAlmostEqual(a['max_abs_x_cm'],b['max_abs_x_cm'],places=7)
            self.assertAlmostEqual(a['voltage_rms'],b['voltage_rms'],places=5)


class RecordingImport(unittest.TestCase):
    def test_default_is_simulated_and_calibrated_counts_round_trip(self):
        from cartpole import import_run
        from cartpole.params import Hardware
        from cartpole.sysid import load_runs
        import cartpole.sysid as sysid
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); source=root/'recording.csv'; data=friction_record()
            rows=np.column_stack([data['t'],data['x'],data['alpha'],np.sin(data['t'])])
            np.savetxt(source,rows,delimiter=',',header='t,x,alpha,v',comments='')
            with patch.object(import_run,'RUNS',root/'runs'),patch.object(sysid,'RUNS',root/'runs'):
                meta=import_run.import_csv(source)
                self.assertTrue(meta['sim']); self.assertEqual(load_runs(),[])
                arr=np.load(root/'runs'/f"{meta['id']}.npy"); hw=Hardware()
                np.testing.assert_allclose(arr[:,7]*hw.cart_m_per_count,data['x'],atol=abs(hw.cart_m_per_count)/2)
                self.assertEqual(len(meta['source_sha256']),64)
                real=import_run.import_csv(source,hardware=True)
                self.assertFalse(real['sim']); self.assertEqual(len(load_runs()),1)
                saved=json.loads((root/'runs'/f"{real['id']}.json").read_text())
                self.assertEqual(saved['voltage_convention'],'logical_command_before_volts_sign')

    def test_rejects_slow_recording_instead_of_assuming_500_hz(self):
        from cartpole.import_run import import_csv
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'slow.csv'; t=np.arange(1200)*.01
            np.savetxt(source,np.column_stack([t,t*0,t*0,t*0]),delimiter=',',header='t,x,alpha,v',comments='')
            with self.assertRaisesRegex(ValueError,'500 Hz'):
                import_csv(source)
