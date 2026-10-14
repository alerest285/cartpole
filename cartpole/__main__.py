"""python -m cartpole <command>

  serve     web UI + 500 Hz loop   --backend sim|fake|quanser  --port 8000
  probe     step-by-step check of the Quanser board (lab PC)
  bench     every controller on a set of mismatched simulated plants
  train     PPO from scratch or warm   --iters 600 --warm current|vNNN|checkpoint
  improve   sysid + real episodes + sim fine-tune + promote
  sysid     fit the plant model from logged runs
  selftest  the test suite (no hardware)
  doctor    read-only environment/policy preflight (never opens the board)
  friction  handout B_f regression from CSV, with held-out validation
"""
from __future__ import annotations

import argparse
import json
import sys
import time


def _progress_writer(path):
    if not path:
        return lambda d: None
    f = open(path, "a", buffering=1)

    def w(d):
        f.write(json.dumps(d, default=float) + "\n")
    return w


def main(argv=None):
    ap = argparse.ArgumentParser(prog="cartpole", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--backend", default="sim", choices=["sim", "fake", "quanser"])
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=8000)
    s.add_argument("--timing", default="sw", choices=["sw", "hw"], help="quanser: software or hardware-clocked loop")
    s.add_argument("--nominal", action="store_true", help="ignore the sysid fit")
    s.add_argument("--no-browser", action="store_true")
    s.add_argument("--demo", action="store_true", help="sim only: swing up when the first browser connects")
    p = sub.add_parser("probe")
    p.add_argument("--fake", action="store_true", help="run the probe against the simulated board")
    p.add_argument("--move", action="store_true", help="also apply small test voltages (cart moves a few mm)")
    b = sub.add_parser("bench")
    b.add_argument("--T", type=float, default=12.0)
    f = sub.add_parser("friction")
    f.add_argument("csv")
    f.add_argument("--output")
    f.add_argument("--delay-steps", type=int, default=0, choices=range(6))
    i = sub.add_parser("import-run", help="import normalized MATLAB CSV; simulated unless --hardware is explicit")
    i.add_argument("csv")
    i.add_argument("--hardware", action="store_true")
    tu = sub.add_parser("tune", help="offline uncertainty sweep; does not change controller settings")
    tu.add_argument("--output", default="data/tuning_report.json")
    tu.add_argument("--cases", type=int, default=6)
    tu.add_argument("--T", type=float, default=12.)
    pg = sub.add_parser("playground", help="offline show, ILC and sampling-MPC demonstrations")
    pg.add_argument("--output", default="docs/evidence/playground")
    for name in ("train", "improve", "sysid"):
        q = sub.add_parser(name)
        q.add_argument("--progress", default=None)
        q.add_argument("--include-sim", action="store_true")
        if name in ("train", "improve"):
            q.add_argument("--iters", type=int, default=600 if name == "train" else 300)
        if name == "train":
            q.add_argument("--warm", default=None)
            q.add_argument("--rand-scale", type=float, default=1.0)
            q.add_argument("--seed", type=int, default=0)
        if name == "improve":
            q.add_argument("--no-real-update", action="store_true")
    sub.add_parser("selftest")
    d = sub.add_parser("doctor")
    d.add_argument("--hardware", action="store_true", help="also require the Quanser API")
    a = ap.parse_args(argv)

    if a.cmd == "playground":
        from .playground import run
        run(a.output)
    elif a.cmd == "tune":
        if not 1 <= a.cases <= 100 or not 8 <= a.T <= 60:
            ap.error("--cases must be 1–100 and --T 8–60 seconds")
        from .robustness import tune
        tune(a.output, a.cases, a.T)
    elif a.cmd == "import-run":
        from .import_run import import_csv
        try:
            print(json.dumps(import_csv(a.csv, a.hardware), indent=2))
        except (ValueError, OSError) as error:
            ap.error(str(error))
    elif a.cmd == "friction":
        from .friction import main as friction_main
        try:
            sys.exit(friction_main(a.csv, a.output, a.delay_steps))
        except (ValueError, OSError) as error:
            ap.error(str(error))
    elif a.cmd == "doctor":
        from .doctor import doctor
        sys.exit(doctor(a.hardware))
    elif a.cmd == "serve":
        if a.demo and a.backend != "sim":
            ap.error("--demo requires --backend sim")
        from .params import load_hardware, load_plant
        from .runtime import Runtime
        from .server import serve
        if a.backend == "sim":
            from .backends.sim import SimBackend
            be = SimBackend(load_plant())
        elif a.backend == "fake":
            from .backends.quanser import FakeQuanserBackend
            be = FakeQuanserBackend(plant=load_plant())
        else:
            from .backends.quanser import QuanserBackend
            be = QuanserBackend(load_hardware(), timing=a.timing)
        rt = Runtime(be, plant=load_plant(use_fitted=not a.nominal))
        if not a.no_browser:
            import threading
            import webbrowser
            threading.Timer(1.0, lambda: webbrowser.open(f"http://{a.host}:{a.port}")).start()
        serve(rt, a.host, a.port, demo=a.demo)
    elif a.cmd == "probe":
        from .probe import probe
        sys.exit(probe(fake=a.fake, move=a.move))
    elif a.cmd == "bench":
        from .bench import bench
        bench(a.T)
    elif a.cmd == "train":
        from .rl.train import run_training
        prog = _progress_writer(a.progress)
        def pr(d):  # noqa: E306
            prog(d)
            if d.get("phase") == "train" and d["iter"] % 25 == 0:
                print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()
                       if k in ("iter", "steps", "sec", "ep_return", "fail_rate", "upright_end", "log_std")}, flush=True)
        v, promoted = run_training(iters=a.iters, warm=a.warm, rand_scale=a.rand_scale, seed=a.seed,
                                   progress=pr, log=lambda m: (print(m, flush=True), prog({"phase": "log", "msg": m})))
        prog({"phase": "done", "version": v, "promoted": promoted})
    elif a.cmd == "improve":
        from .rl.improve import improve
        prog = _progress_writer(a.progress)
        rep = improve(iters=a.iters, include_sim=a.include_sim, real_update=not a.no_real_update, progress=prog,
                      log=lambda m: (print(m, flush=True), prog({"phase": "log", "msg": m})))
        print(json.dumps({k: v for k, v in rep.items() if k != "hardware"}, indent=1, default=float))
    elif a.cmd == "sysid":
        from . import sysid
        prog = _progress_writer(a.progress)
        log = lambda m: (print(m, flush=True), prog({"phase": "log", "msg": m}))  # noqa: E731
        res = sysid.fit(include_sim=a.include_sim, log=log)
        accepted = bool(res and sysid.save(res, log=log))
        prog({"phase": "done", "accepted": accepted})
        if not accepted:
            sys.exit(2)
    elif a.cmd == "selftest":
        import unittest
        from pathlib import Path
        suite = unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parent.parent / "tests"))
        r = unittest.TextTestRunner(verbosity=2).run(suite)
        sys.exit(0 if r.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
