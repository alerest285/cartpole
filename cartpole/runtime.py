"""The 500 Hz loop: read -> estimate -> control -> safety -> write -> log.

Runs in its own thread. The web server only calls the small command methods
(all lock-protected) and pulls decimated samples; it never touches the loop's
timing. Every armed stretch is saved to runs/ as one run (npz + json), which is
what system identification and PPO learn from.
"""
from __future__ import annotations

import json
import threading
import time
import traceback
from collections import deque
from dataclasses import asdict
from datetime import datetime

import numpy as np

from .controllers import REGISTRY, make
from .controllers.base import Ref
from .estimator import Estimator
from .params import H, RUNS, Plant, load_plant
from .safety import Safety, SafetyConfig

COLS = ["t", "x", "alpha", "xd", "ad", "v_des", "v", "cart_counts", "pole_counts", "mode", "phase", "extra"]
PHASES = {"": 0, "waiting (lift the pole by hand)": 1, "swinging": 2, "balancing": 3, "resetting": 4, "exploring": 5, "exciting": 6}


class Runtime:
    def __init__(self, backend, plant: Plant | None = None, log=print, deadman_s: float = 0.5):
        self.backend = backend
        self.plant = plant or load_plant()
        self.log = log
        self.lock = threading.RLock()
        self.ctrls = {k: make(k, self.plant) for k in REGISTRY}
        self.mode = "idle"
        self.est = Estimator()
        self.safety = Safety(self.plant, SafetyConfig(v_cap=10. if backend.is_sim else 4.))
        if not backend.is_sim:
            self.ctrls['excite'].set_params({'amp':1.5,'noise':0.5})
        self.ref = Ref()
        self.armed = False
        self.fault: str | None = None
        self.deadman_s = deadman_s if not backend.is_sim else None
        self.heartbeat = time.monotonic()
        self.t = 0.0
        self.k = 0
        self.s = np.zeros(4)
        self.counts = (0, 0)
        self.v = 0.0
        self.v_des = 0.0
        self.ring = deque(maxlen=5000)       # decimated samples for the UI
        self.ring_seq = 0
        self.run_rows: list | None = None
        self.run_meta: dict | None = None
        self.saved_runs: list[str] = []
        self.timing = {"period_ms_max": 0.0, "period_ms_mean": 2.0, "compute_ms_max": 0.0, "overruns": 0}
        self._stop = threading.Event()
        self.thread: threading.Thread | None = None
        self.hooks = []                      # callables(runtime, s, v) run each tick (RL episodes)
        self.messages = deque(maxlen=50)
        self._writers = []

    # ------------------------------------------------------------------ loop
    def start(self):
        self.backend.open()
        self.est.started = False
        self._stop.clear()
        self.thread = threading.Thread(target=self._loop, name="control-500Hz", daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=2)
        with self.lock:
            self._end_run("server stopped")
        self.backend.close()
        for writer in self._writers:
            writer.join(timeout=5)

    def note(self, msg: str):
        self.messages.append({"t": time.time(), "msg": msg})
        self.log(msg)

    def _loop(self):
        last = time.perf_counter()
        per = deque(maxlen=500)
        try:
            while not self._stop.is_set():
                self.backend.wait_tick()
                now = time.perf_counter()
                per.append(now - last)
                last = now
                c0 = time.perf_counter()
                with self.lock:
                    self._tick()
                comp = time.perf_counter() - c0
                if self.k % 250 == 0 and per:
                    pm = np.array(per) * 1000
                    self.timing.update(period_ms_max=float(pm.max()), period_ms_mean=float(pm.mean()),
                                       overruns=int(getattr(self.backend, "pacer", None).overruns if hasattr(self.backend, "pacer") else 0))
                self.timing["compute_ms_max"] = max(self.timing["compute_ms_max"] * 0.999, comp * 1000)
        except Exception as e:  # noqa: BLE001
            self.fault = f"control loop crashed: {e}"
            with self.lock:
                self._disarm(self.fault)
            self.note(traceback.format_exc())
            try:
                self.backend.write_volts(0.0)
            except Exception:  # noqa: BLE001
                pass
            self.backend.close()

    def _tick(self):
        cart, pole = self.backend.read_counts()
        self.counts = (cart, pole)
        x, au = self.backend.counts_to_xa(cart, pole)
        s = self.est(x, au)
        self.s = s
        ctrl = self.ctrls[self.mode]
        v_des = 0.0
        if self.armed:
            if self.deadman_s and time.monotonic() - self.heartbeat > self.deadman_s:
                self._disarm("browser connection lost (dead-man)")
            else:
                try:
                    v_des = float(ctrl.step(s, self.t, self.ref))
                except Exception as e:  # noqa: BLE001
                    self._disarm(f"controller error: {e}")
                    v_des = 0.0
        v, f = self.safety(s, v_des) if self.armed else (0.0, None)
        if f:
            self._disarm(f)
            v = 0.0
        for hook in list(self.hooks):
            v = hook(self, s, v)
        self.backend.write_volts(v)
        ctrl.observe_voltage(v_des, v)
        self.v, self.v_des = v, v_des
        phase = PHASES.get(ctrl.info.get("phase", ""), 0) if self.armed else 0
        row = (self.t, s[0], s[1], s[2], s[3], v_des, v, cart, pole,
               list(REGISTRY).index(self.mode), phase, float(ctrl.info.get("extra", 0.0)))
        if self.run_rows is not None:
            self.run_rows.append(row)
        if self.k % 5 == 0:                      # 100 Hz to the UI
            self.ring.append(row)
            self.ring_seq += 1
        self.t += H
        self.k += 1

    # ------------------------------------------------------------- commands
    def start_demo(self) -> str | None:
        """One-click simulation preset; never available to a hardware backend."""
        from .backends.sim import SimBackend
        with self.lock:
            if not isinstance(self.backend, SimBackend):
                return "Demo is available only in the simulator"
            if self.fault and "crashed" in self.fault:
                return self.fault
            self._disarm("demo restarted", operator=True)
            self.backend.write_volts(0.0)
            self.backend.sim.reset()
            self.est = Estimator()
            self.safety = Safety(self.plant, SafetyConfig())
            self.ref = Ref()
            self.ctrls["energy"] = make("energy", self.plant)
            self.mode = "energy"
            self.s = np.array([0.0, -np.pi, 0.0, 0.0])
            self.counts = self.backend.read_counts()
            return self.arm()

    def arm(self) -> str | None:
        with self.lock:
            if self.mode == "ppo" and self.est.kind != "kalman":
                return "The current PPO policy requires its trained Kalman estimator"
            if self.armed:
                return None
            if self.fault and "crashed" in self.fault:
                return self.fault
            f = self.safety.fault(self.s)
            if f:
                return f"cannot arm: {f}"
            self.fault = None
            self.heartbeat = time.monotonic()
            ctrl = self.ctrls[self.mode]
            ctrl.reset(self.s)
            self.armed = True
            self._begin_run()
            self.note(f"armed: {ctrl.label}")
            return None

    def disarm(self, why: str = "operator"):
        with self.lock:
            self._disarm(why, operator=True)

    def _disarm(self, why: str, operator: bool = False):
        if not self.armed:
            return
        self.armed = False
        if not operator:
            self.fault = why
        self.note(f"disarmed: {why}")
        self._end_run(why)

    def set_mode(self, key: str):
        with self.lock:
            if key not in self.ctrls:
                return "Unknown controller"
            if key == "ppo" and self.est.kind != "kalman":
                return "The current PPO policy requires its trained Kalman estimator"
            if self.armed:
                self._end_run(f"mode -> {key}")
            self.mode = key
            self.ref.manual = 0.0
            self.ctrls[key].reset(self.s)
            if self.armed:
                self._begin_run()

    def set_params(self, key: str, values: dict):
        with self.lock:
            if key in self.ctrls:
                self.ctrls[key].set_params(values)
                if self.run_meta is not None and key == self.mode:
                    self.run_meta.setdefault("param_changes", []).append({"t": self.t, key: values})

    def set_ref(self, x_ref=None, manual=None):
        with self.lock:
            if x_ref is not None:
                self.ref.x_ref = float(np.clip(x_ref, -0.12, 0.12))
            if manual is not None:
                self.ref.manual = float(np.clip(manual, -1, 1))

    def set_safety(self, values: dict):
        with self.lock:
            cfg = self.safety.cfg
            candidate = asdict(cfg)
            for k, v in values.items():
                if hasattr(cfg, k):
                    cur = getattr(cfg, k)
                    candidate[k] = bool(v) if isinstance(cur, bool) else float(v)
            numeric = [v for k, v in candidate.items() if k != "cbf"]
            if not all(np.isfinite(v) and v > 0 for v in numeric):
                raise ValueError("Safety limits must be finite and positive")
            candidate["v_cap"] = min(candidate["v_cap"], self.backend.hw.v_limit)
            candidate["x_hard"] = min(candidate["x_hard"], 0.19, self.backend.hw.x_watchdog - 0.005)
            candidate["x_soft"] = min(candidate["x_soft"], 0.17)
            if candidate["x_soft"] >= candidate["x_hard"]:
                raise ValueError("Soft track limit must be inside the hard limit")
            if not self.backend.is_sim:
                candidate["cbf"] = True
            self.safety.cfg = SafetyConfig(**candidate)

    def set_estimator(self, kind: str, w=None):
        with self.lock:
            if self.armed:
                raise ValueError("Stop before changing the estimator")
            if kind not in Estimator.KINDS:
                raise ValueError("Unknown estimator")
            if w is not None and (not np.isfinite(float(w)) or not 10 <= float(w) <= 200):
                raise ValueError("Derivative cutoff must be 10–200 rad/s")
            self.est.configure(kind=kind, w=None if w is None else float(w))

    def set_plant(self, plant: Plant):
        with self.lock:
            self.plant = plant
            for c in self.ctrls.values():
                c.set_plant(plant)
            self.safety.set_plant(plant)

    def zero(self) -> str | None:
        with self.lock:
            if self.armed:
                return "disarm before zeroing"
            self.backend.zero()
            self.est.started = False
            self.note("encoders zeroed: cart = centre, pole = hanging")
            return None

    def beat(self):
        self.heartbeat = time.monotonic()

    def clear_fault(self):
        with self.lock:
            if self.fault and "crashed" not in self.fault:
                self.fault = None

    # ---------------------------------------------------------------- runs
    def _begin_run(self):
        ctrl = self.ctrls[self.mode]
        self.run_rows = []
        self.run_meta = {
            "id": datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3],
            "backend": self.backend.info().get("name"),
            "sim": self.backend.is_sim,
            "mode": self.mode,
            "params": dict(ctrl.p),
            "plant": self.plant.to_dict(),
            "hardware": asdict(self.backend.hw),
            "voltage_convention": "logical_command_before_volts_sign",
            "safety": asdict(self.safety.cfg),
            "estimator": self.est.kind,
            "estimator_settings": {"w": self.est.w, "q_x": self.est.q_x, "q_a": self.est.q_a},
            "policy": getattr(ctrl, "version", None),
            "h": H,
            "t0": self.t,
            "cols": COLS,
            "modes": list(REGISTRY),
        }

    def _end_run(self, why: str):
        rows, meta = self.run_rows, self.run_meta
        self.run_rows = self.run_meta = None
        ep = None
        if meta is not None:
            flush = getattr(self.ctrls.get(meta["mode"]), "flush_episode", None)
            ep = flush() if flush else None
        if not rows or len(rows) < 100:
            return
        meta["end"] = why
        meta["timing_at_end"] = dict(self.timing)
        meta["n"] = len(rows)
        meta["duration_s"] = len(rows) * H
        RUNS.mkdir(parents=True, exist_ok=True)
        rid = meta["id"]
        arr = np.asarray(rows, dtype=np.float64)
        if ep is not None:
            meta["ppo_episode"] = True
            meta["policy"] = str(ep["version"])
        self._writers = [w for w in self._writers if w.is_alive()]
        writer = threading.Thread(target=self._save, args=(rid, arr, meta, ep), daemon=True)
        self._writers.append(writer)
        writer.start()

    def _save(self, rid, arr, meta, ep=None):
        np.save(RUNS / f"{rid}.npy", arr)
        np.savetxt(RUNS / f"{rid}.csv", arr, delimiter=",", header=",".join(COLS), comments="")
        if ep is not None:
            np.savez(RUNS / f"{rid}.ppo.npz", **ep)
        (RUNS / f"{rid}.json").write_text(json.dumps(meta, indent=1, default=str))
        self.saved_runs.append(rid)
        self.note(f"saved run {rid} ({meta['duration_s']:.1f} s, {meta['mode']}, end: {meta['end']})")

    # ------------------------------------------------------------ snapshots
    def samples_since(self, seq: int):
        with self.lock:
            n_new = self.ring_seq - seq
            if n_new <= 0:
                return [], self.ring_seq
            n_new = min(n_new, len(self.ring))
            return list(self.ring)[-n_new:], self.ring_seq

    def status(self) -> dict:
        with self.lock:
            ctrl = self.ctrls[self.mode]
            return {
                "armed": self.armed, "fault": self.fault, "mode": self.mode,
                "ctrl": ctrl.state(), "ref": {"x_ref": self.ref.x_ref, "manual": self.ref.manual},
                "safety": asdict(self.safety.cfg), "estimator": self.est.kind, "estimator_w": self.est.w,
                "backend": self.backend.info(), "timing": self.timing,
                "s": [float(v) for v in self.s], "v": self.v, "counts": self.counts,
                "plant": self.plant.to_dict(), "t": self.t,
            }
