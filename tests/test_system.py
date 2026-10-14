"""The hardware code path (fake board), the runtime, the server, sysid, PPO.

Anything that writes files runs in a subprocess with CARTPOLE_DATA/RUNS
pointing at a temp folder, so the real run history is never touched.
"""
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
from pathlib import Path

import numpy as np

from cartpole.backends.quanser import FakeHIL, FakeQuanserBackend
from cartpole.params import Hardware, Plant
from cartpole.rl.policy import MLP, Policy

ROOT = Path(__file__).resolve().parent.parent


def run_py(code: str, timeout=600) -> str:
    d = tempfile.mkdtemp(prefix="cartpole-test-")
    env = dict(os.environ, CARTPOLE_DATA=f"{d}/data", CARTPOLE_RUNS=f"{d}/runs", OPENBLAS_NUM_THREADS="1")
    r = subprocess.run([sys.executable, "-c", textwrap.dedent(code)], cwd=ROOT, env=env,
                       capture_output=True, text=True, timeout=timeout)
    if r.returncode:
        raise AssertionError(r.stderr[-3000:])
    return r.stdout


class HardwarePath(unittest.TestCase):
    def test_counts_signs_and_volts(self):
        be = FakeQuanserBackend(realtime=False, log=lambda m: None)
        be.open()
        c0, p0 = be.read_counts()
        self.assertEqual((c0, p0), (0, 0))
        x, a = be.counts_to_xa(c0, p0)
        self.assertAlmostEqual(a, -math.pi)          # hanging reads -pi, like the lab models
        for _ in range(100):
            be.write_volts(2.0)
            be.wait_tick()
        x, _ = be.counts_to_xa(*be.read_counts())
        self.assertGreater(x, 0.002)                 # +V moves the cart toward +x
        self.assertTrue(be.fake.do[:4].all())        # amplifier enabled
        be.close()
        self.assertEqual(be.fake.ao, 0.0)
        self.assertFalse(be.fake.do[:4].any())

    def test_board_watchdog_cuts_the_motor(self):
        be = FakeQuanserBackend(realtime=False, log=lambda m: None)
        be.open()
        be.write_volts(5.0)
        time.sleep(be.hw.hil_watchdog_s * 1.5)       # the loop "hangs"
        be.wait_tick()
        self.assertTrue(be.fake.wd_expired)
        be.close()

    def test_runtime_swings_up_through_the_fake_board(self):
        out = run_py("""
            import time
            from cartpole.backends.quanser import FakeQuanserBackend
            from cartpole.runtime import Runtime
            rt = Runtime(FakeQuanserBackend(realtime=False, log=lambda m: None), log=lambda m: None)
            rt.deadman_s = None
            rt.start(); rt.set_mode("energy"); assert rt.arm() is None
            t0 = rt.t
            while rt.t < t0 + 10: time.sleep(0.01)
            print(rt.armed, round(abs(rt.s[1]), 3)); rt.disarm(); time.sleep(0.3); rt.stop()
        """)
        armed, a = out.split()
        self.assertEqual(armed, "True")
        self.assertLess(float(a), 0.1)

    def test_deadman_disarms_on_hardware(self):
        out = run_py("""
            import time
            from cartpole.backends.quanser import FakeQuanserBackend
            from cartpole.runtime import Runtime
            be = FakeQuanserBackend(realtime=True, log=lambda m: None); be.is_sim = False
            rt = Runtime(be, log=lambda m: None, deadman_s=0.3)
            rt.start(); rt.set_mode("manual"); rt.arm(); time.sleep(0.8)
            print(rt.armed, rt.fault); rt.stop()
        """)
        self.assertTrue(out.startswith("False"), out)
        self.assertIn("dead-man", out)


class Sysid(unittest.TestCase):
    def test_recovers_a_hidden_plant(self):
        out = run_py("""
            import time, json
            from dataclasses import replace
            from cartpole.params import Plant
            from cartpole.backends.sim import SimBackend
            from cartpole.runtime import Runtime
            from cartpole import sysid
            true = replace(Plant(), B_f=3.3, k_amp=0.93, B_p=0.004, F_coul=0.15, delay_steps=1, m_c=0.6)
            rt = Runtime(SimBackend(true, realtime=False), plant=Plant(), log=lambda m: None)
            rt.start()
            for mode, secs in (("excite", 30), ("energy", 8)):
                rt.set_mode(mode); rt.arm(); t0 = rt.t
                while rt.t < t0 + secs: time.sleep(0.01)
                rt.disarm()
            time.sleep(0.5); rt.stop(); time.sleep(0.3)
            r = sysid.fit(include_sim=True, log=lambda m: None)
            print(json.dumps(r["params"]))
        """)
        p = json.loads(out.strip().splitlines()[-1])
        self.assertAlmostEqual(p["k_amp"], 0.93, delta=0.05)
        self.assertAlmostEqual(p["B_f"], 3.3, delta=0.6)
        self.assertAlmostEqual(p["m_c"], 0.6, delta=0.05)
        self.assertAlmostEqual(p["B_p"], 0.004, delta=0.0008)
        self.assertEqual(p["delay_steps"], 1)


class RL(unittest.TestCase):
    def test_mlp_gradient(self):
        rng = np.random.default_rng(0)
        net = MLP([6, 8, 8, 1], rng, out_scale=1.0)
        x = rng.normal(size=(5, 6))
        y, hs = net.forward(x)
        g = net.backward(hs, np.ones_like(y))
        W = net.W[0]
        i, j, eps = 2, 3, 1e-6
        W[i, j] += eps; yp, _ = net.forward(x); W[i, j] -= 2 * eps; ym, _ = net.forward(x); W[i, j] += eps
        self.assertAlmostEqual(g[0][i, j], float((yp.sum() - ym.sum()) / (2 * eps)), places=5)

    def test_policy_roundtrip(self):
        p = Policy(seed=3)
        f = Path(tempfile.mkdtemp()) / "p.npz"
        p.save(f, {"v_max": 8.0})
        q = Policy.load(f)
        o = np.random.default_rng(0).normal(size=(4, 6))
        np.testing.assert_allclose(p.mean(o), q.mean(o))
        self.assertEqual(q.meta["v_max"], 8.0)

    def test_improve_end_to_end(self):
        """Explore episodes recorded through the runtime -> improve (sysid + real
        update + fine-tune) -> a new registered version."""
        out = run_py("""
            import time, json
            from dataclasses import replace
            from cartpole.params import Plant
            from cartpole.rl.train import run_training
            from cartpole.backends.sim import SimBackend
            from cartpole.runtime import Runtime
            from cartpole.rl.improve import improve
            from cartpole.controllers.learned import registry
            v, _ = run_training(iters=4, log=lambda m: None)
            rt = Runtime(SimBackend(replace(Plant(), B_f=3.0), realtime=False), log=lambda m: None)
            rt.start()
            rt.set_mode("excite"); rt.arm(); t0 = rt.t
            while rt.t < t0 + 15: time.sleep(0.01)
            rt.disarm()
            rt.set_mode("ppo"); rt.set_params("ppo", {"explore": True})
            for _ in range(2):
                rt.arm(); t0 = rt.t
                while rt.t < t0 + 3 and rt.armed: time.sleep(0.01)
                rt.disarm(); time.sleep(0.2)
            time.sleep(0.5); rt.stop(); time.sleep(0.3)
            rep = improve(iters=3, include_sim=True, log=lambda m: None)
            reg = registry()
            print(json.dumps({"versions": sorted(reg["versions"]), "real": rep.get("real_update", {}).get("episodes"),
                              "sysid": rep.get("sysid", {}).get("saved"), "hw": list(rep["hardware"])}))
        """)
        r = json.loads(out.strip().splitlines()[-1])
        self.assertEqual(r["versions"], ["v001", "v002"])
        self.assertGreaterEqual(r["real"], 1)
        self.assertTrue(r["sysid"])


class Server(unittest.TestCase):
    def test_websocket_protocol_and_csv(self):
        out = run_py("""
            import threading, time, socket, base64, os, json, struct, urllib.request
            from cartpole.backends.sim import SimBackend
            from cartpole.runtime import Runtime
            from cartpole import server
            s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
            rt = Runtime(SimBackend(realtime=True), log=lambda m: None)
            threading.Thread(target=server.serve, args=(rt, "127.0.0.1", port), daemon=True).start()
            time.sleep(1.0)
            k = socket.create_connection(("127.0.0.1", port)); key = base64.b64encode(os.urandom(16)).decode()
            k.sendall(f"GET /ws HTTP/1.1\\r\\nHost: x\\r\\nUpgrade: websocket\\r\\nConnection: Upgrade\\r\\nSec-WebSocket-Key: {key}\\r\\nSec-WebSocket-Version: 13\\r\\n\\r\\n".encode())
            h = b""
            while b"\\r\\n\\r\\n" not in h: h += k.recv(1)
            def rx(n):
                b = b""
                while len(b) < n: b += k.recv(n - len(b))
                return b
            def recv():
                b1, b2 = rx(2); n = b2 & 127
                if n == 126: n = struct.unpack("!H", rx(2))[0]
                elif n == 127: n = struct.unpack("!Q", rx(8))[0]
                return json.loads(rx(n))
            def send(o):
                d = json.dumps(o).encode(); m = os.urandom(4)
                k.sendall(struct.pack("!BB", 0x81, 0x80 | len(d)) + m + bytes(b ^ m[i % 4] for i, b in enumerate(d)))
            hello = recv()
            send({"type": "mode", "key": "energy"}); send({"type": "arm"})
            t0 = time.time(); st = None
            while time.time() - t0 < 8:
                send({"type": "beat"}); m = recv()
                if m["type"] == "status": st = m["status"]
            send({"type": "stop"}); time.sleep(1.0)
            runs = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/api/runs").read())
            csv = urllib.request.urlopen(f"http://127.0.0.1:{port}/api/runs/{runs[0]['id']}.csv").read().decode()
            print(json.dumps({"modes": hello["modes"], "armed": st["armed"], "alpha": st["s"][1],
                              "runs": len(runs), "csv_head": csv.splitlines()[0], "csv_rows": len(csv.splitlines())}))
        """)
        r = json.loads(out.strip().splitlines()[-1])
        self.assertIn("ppo", r["modes"])
        self.assertTrue(r["armed"])
        self.assertLess(abs(r["alpha"]), 0.1)
        self.assertEqual(r["runs"], 1)
        self.assertTrue(r["csv_head"].startswith("t,x,alpha"))
        self.assertGreater(r["csv_rows"], 3000)


if __name__ == "__main__":
    unittest.main()
