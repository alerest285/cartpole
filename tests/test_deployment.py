"""Regression tests for first-session hardware and data failure modes."""
import json
import tempfile
import unittest
from dataclasses import asdict, replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from cartpole import sysid
from cartpole.backends.quanser import FakeHIL, FakeQuanserBackend, QuanserBackend
from cartpole.params import Hardware, Plant
from cartpole.runtime import Runtime
from cartpole.safety import Safety, SafetyConfig


class Deployment(unittest.TestCase):
    def test_demo_never_arms_hardware_and_resets_simulation(self):
        from cartpole.backends.sim import SimBackend
        be=FakeQuanserBackend(log=lambda _:None)
        rt=Runtime(be,log=lambda _:None)
        self.assertIn("only in the simulator",rt.start_demo())
        self.assertFalse(rt.armed)
        rt=Runtime(SimBackend(realtime=False),log=lambda _:None)
        rt.backend.sim.reset([0.1,0.2,1.0,1.0])
        rt.set_ref(x_ref=0.1)
        self.assertIsNone(rt.start_demo())
        self.assertTrue(rt.armed)
        self.assertEqual(rt.mode,"energy")
        self.assertEqual(rt.ref.x_ref,0)
        np.testing.assert_allclose(rt.backend.sim.s,[0,-np.pi,0,0])

    def test_infeasible_braking_never_exceeds_voltage_cap(self):
        safe = Safety(Plant(), SafetyConfig(v_cap=4))
        for x, xd in ((0.18, 3.0), (-0.18, -3.0)):
            v, _ = safe(np.array([x, 0, xd, 0]), 10)
            self.assertLessEqual(abs(v), 4)
            self.assertLess(v*xd, 0)

    def test_watchdog_start_failure_closes_without_enabling(self):
        class BrokenHIL(FakeHIL):
            def watchdog_start(self, timeout):
                raise RuntimeError("not supported")
        card = BrokenHIL()
        be = QuanserBackend(hil_factory=lambda: card, log=lambda _: None)
        with self.assertRaisesRegex(RuntimeError, "start refused"):
            be.open()
        self.assertIsNone(card.opened)
        self.assertFalse(card.do.any())
        self.assertEqual(card.ao, 0)

    def test_expired_watchdog_aborts_nonzero_write(self):
        be = FakeQuanserBackend(realtime=False, log=lambda _: None)
        be.open()
        be.fake.wd_last -= 1
        with self.assertRaisesRegex(RuntimeError, "expired"):
            be.write_volts(5)
        self.assertEqual(be.fake.ao, 0)
        be.close()

    def test_fake_runs_cannot_enter_hardware_sysid(self):
        self.assertTrue(FakeQuanserBackend().is_sim)
        with tempfile.TemporaryDirectory() as folder, patch.object(sysid, "RUNS", Path(folder)):
            root = Path(folder)
            (root / "legacy.json").write_text(json.dumps({"id":"legacy", "sim":False, "backend":"fake Q2-USB (simulated physics)"}))
            np.save(root / "legacy.npy", np.zeros((200,12)))
            self.assertEqual(sysid.load_runs(), [])
            self.assertEqual(len(sysid.load_runs(include_sim=True)), 1)

    def test_sysid_uses_recorded_encoder_map_and_logical_volts(self):
        arr = np.zeros((600,12)); t = np.arange(len(arr))*0.002
        arr[:,7] = np.rint(1000*np.sin(3*t))
        arr[:,8] = np.rint(100*np.sin(5*t))
        arr[:,6] = np.sin(7*t)
        hw = Hardware()
        expected = sysid.build([({"sim":False},arr)],hw,0)
        changed = arr.copy(); changed[:,7:9] *= -1
        calibrated = replace(hw, cart_m_per_count=-hw.cart_m_per_count,
                             pole_rad_per_count=-hw.pole_rad_per_count, volts_sign=-1)
        actual = sysid.build([({"sim":False,"hardware":asdict(calibrated)},changed)],hw,0)
        for a,b in zip(expected,actual):
            np.testing.assert_allclose(a,b)

    def test_repeated_arm_preserves_run_and_real_limits(self):
        be=FakeQuanserBackend(log=lambda _:None); be.is_sim=False
        rt=Runtime(be, log=lambda _:None)
        rt.arm(); meta=rt.run_meta; rt.arm()
        self.assertIs(meta,rt.run_meta)
        rt.set_safety({"cbf":False,"v_cap":100,"x_hard":1,"x_soft":0.9})
        self.assertTrue(rt.safety.cfg.cbf)
        self.assertLessEqual(rt.safety.cfg.x_hard,0.19)
        self.assertLessEqual(rt.safety.cfg.v_cap,10)
        with self.assertRaises(ValueError):
            rt.set_safety({"v_cap":float('nan')})

    def test_ppo_requires_current_policy_and_contiguous_transitions(self):
        from cartpole.rl import improve as imp
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for rid,version,ticks,sim in [('good','v002',[0,10,20],False),
                                          ('old','v001',[0,10,20],False),
                                          ('gap','v002',[0,10,50],False),
                                          ('fake','v002',[0,10,20],True)]:
                (root/f'{rid}.json').write_text(json.dumps({'id':rid,'policy':version,'sim':sim}))
                np.savez(root/f'{rid}.ppo.npz',tick=ticks)
            with patch.object(imp,'RUNS',root), patch.object(imp,'CONSUMED',root/'used.json'):
                self.assertEqual([r[0] for r in imp.hardware_episodes(False,'v002')],['good'])


if __name__ == '__main__':
    unittest.main()
