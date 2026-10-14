"""Web server: static UI + a WebSocket for live data and commands.

Standard library only (http.server + a minimal RFC 6455 WebSocket), so the lab
PC needs nothing beyond numpy. Binds to 127.0.0.1 by default; the rig should
only ever be driven from the PC it is plugged into.

Training and the improve loop run as subprocesses (python -m cartpole ...),
never in the server process, so they cannot steal time from the 500 Hz loop
through the GIL. They report through a JSONL progress file the server tails.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np

from .controllers import schemas
from .controllers.learned import registry
from .params import DATA, FITTED, ROOT, RUNS, load_plant
from .runtime import COLS, Runtime

WEB = ROOT / "web"
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".json": "application/json"}


# ---------------------------------------------------------------- websocket
class WS:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.lock = threading.Lock()
        self.open = True

    def send(self, obj):
        data = json.dumps(obj, separators=(",", ":"), default=_jsonable).encode()
        n = len(data)
        if n < 126:
            hdr = struct.pack("!BB", 0x81, n)
        elif n < 65536:
            hdr = struct.pack("!BBH", 0x81, 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x81, 127, n)
        with self.lock:
            try:
                self.sock.sendall(hdr + data)
            except OSError:
                self.open = False

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError
            buf += chunk
        return buf

    def recv(self):
        """Next text message as a string; None on close."""
        while True:
            b1, b2 = self._recv_exact(2)
            op, masked, n = b1 & 0x0F, b2 & 0x80, b2 & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._recv_exact(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._recv_exact(8))[0]
            mask = self._recv_exact(4) if masked else b"\0\0\0\0"
            payload = bytearray(self._recv_exact(n))
            for i in range(n):
                payload[i] ^= mask[i % 4]
            if op == 0x8:
                return None
            if op == 0x9:  # ping -> pong
                with self.lock:
                    self.sock.sendall(struct.pack("!BB", 0x8A, len(payload)) + bytes(payload))
                continue
            if op in (0x1, 0x0):
                return payload.decode("utf-8", "replace")


def _jsonable(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


# --------------------------------------------------------------------- jobs
class Jobs:
    """One background job at a time (train / improve / sysid), as a subprocess."""

    def __init__(self, note):
        self.proc = None
        self.kind = None
        self.progress_file = DATA / "job_progress.jsonl"
        self.pos = 0
        self.last: list[dict] = []
        self.note = note

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, kind: str, args: list[str], command=None):
        if self.running():
            return f"a {self.kind} job is already running"
        DATA.mkdir(parents=True, exist_ok=True)
        self.progress_file.write_text("")
        self.pos = 0
        self.last = []
        cmd = command or [sys.executable, "-m", "cartpole", kind, "--progress", str(self.progress_file), *args]
        env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        flags = {}
        if os.name == "nt":  # keep the job below the control loop on Windows
            flags["creationflags"] = getattr(subprocess, "BELOW_NORMAL_PRIORITY_CLASS", 0)
        with open(DATA / "job_stderr.log", "w") as err, open(DATA / "job_stdout.log", "w") as out:
            self.proc = subprocess.Popen(cmd, cwd=str(ROOT), env=env, stdout=out, stderr=err, **flags)
        self.kind = kind
        self.note(f"started {kind} job: {' '.join(args)}")
        return None

    def cancel(self):
        if self.running():
            self.proc.terminate()
            self.note(f"cancelled {self.kind} job")

    def poll(self) -> list[dict]:
        out = []
        if self.progress_file.exists():
            with open(self.progress_file) as f:
                f.seek(self.pos)
                for line in f:
                    if line.endswith("\n"):
                        try:
                            out.append(json.loads(line))
                        except ValueError:
                            pass
                        self.pos += len(line.encode())
        if self.kind and not self.running() and self.proc is not None:
            rc = self.proc.returncode
            err = ""
            if rc:
                try:
                    err = (DATA / "job_stderr.log").read_text()[-1500:]
                except OSError:
                    pass
            stdout = (DATA / "job_stdout.log").read_text(errors="replace")[-5000:]
            out.append({"phase": "finished", "kind": self.kind, "rc": rc, "stderr": err, "stdout": stdout})
            self.kind = None
            self.proc = None
        self.last = (self.last + out)[-400:]
        return out


# ------------------------------------------------------------------- server
class App:
    def __init__(self, runtime: Runtime, demo: bool = False):
        self.rt = runtime
        self.demo_pending = demo
        self.clients: list[WS] = []
        self.jobs = Jobs(runtime.note)
        self.msg_seq = 0

    def hello(self) -> dict:
        return {"type": "hello", "schemas": schemas(), "status": self.rt.status(), "cols": COLS,
                "modes": list(self.rt.ctrls), "params": {k: c.p for k, c in self.rt.ctrls.items()},
                "infos": {k: c.info for k, c in self.rt.ctrls.items()},
                "policies": registry(), "runs": list_runs()[:200], "fit": read_fit(),
                "job": {"running": self.jobs.running(), "kind": self.jobs.kind, "log": self.jobs.last[-100:]}}

    def handle(self, ws: WS, m: dict):
        rt = self.rt
        t = m.get("type")
        rt.beat()
        err = None
        if t == "beat":
            return
        if t == "demo":
            err = "Wait for the offline job to finish or cancel it first" if self.jobs.running() else rt.start_demo()
            ws.send(self.hello())
        elif t == "arm":
            err = "Wait for the offline job to finish or cancel it first" if self.jobs.running() else rt.arm()
        elif t in ("disarm", "stop"):
            rt.disarm("operator stop" if t == "stop" else "operator")
        elif t == "mode":
            err = rt.set_mode(m["key"])
        elif t == "params":
            rt.set_params(m["key"], m.get("values", {}))
            ws.send({"type": "info", "key": m["key"], "info": rt.ctrls[m["key"]].info, "params": rt.ctrls[m["key"]].p})
        elif t == "ref":
            rt.set_ref(m.get("x_ref"), m.get("manual"))
        elif t == "safety":
            rt.set_safety(m.get("values", {}))
        elif t == "estimator":
            rt.set_estimator(m.get("kind", "kalman"), m.get("w"))
        elif t == "zero":
            err = rt.zero()
        elif t == "clear_fault":
            rt.clear_fault()
        elif t == "poke" and rt.backend.is_sim:
            sim = getattr(rt.backend, "sim", None) or getattr(getattr(rt.backend, "fake", None), "sim", None)
            if sim is not None:
                with rt.lock:
                    sim.poke(float(m.get("dv", 1.5)))
        elif t == "reset_sim" and rt.backend.is_sim:
            sim = getattr(rt.backend, "sim", None) or getattr(getattr(rt.backend, "fake", None), "sim", None)
            if sim is not None:
                with rt.lock:
                    rt.disarm("simulator reset")
                    up = m.get("upright")
                    sim.reset([0.0, 0.05, 0.0, 0.0] if up else None)
                    rt.est.started = False
        elif t == "train":
            args = ["--iters", str(int(m.get("iters", 600))), "--rand-scale", str(float(m.get("rand_scale", 1.0)))]
            if m.get("warm"):
                args += ["--warm", str(m["warm"])]
            err = self.jobs.start("train", args)
        elif t == "improve":
            args = ["--iters", str(int(m.get("iters", 300)))]
            if m.get("include_sim"):
                args.append("--include-sim")
            if m.get("no_real_update"):
                args.append("--no-real-update")
            err = self.jobs.start("improve", args)
        elif t == "sysid":
            args = ["--include-sim"] if m.get("include_sim") else []
            err = self.jobs.start("sysid", args)
        elif t == "cancel_job":
            self.jobs.cancel()
        elif t == "lab_job":
            task = m.get("task")
            commands = {
                "tune": [sys.executable, "-m", "cartpole", "tune"],
                "playground": [sys.executable, "-m", "cartpole", "playground"],
                "rehearsal": [sys.executable, "tools/rehearse_lab.py"],
                "modeling": [sys.executable, "tools/prepare_course.py"],
                "export_matlab": [sys.executable, "tools/export_matlab.py"],
                "bench": [sys.executable, "-m", "cartpole", "bench"],
            }
            if task not in commands:
                err = "Unknown lab task"
            elif rt.armed:
                err = "Stop the controller before starting an offline lab task"
            else:
                err = self.jobs.start(task, [], command=commands[task])
                if not err:
                    ws.send({"type": "job", "events": [{"phase": "started", "kind": task}],
                             "running": True, "kind": task})
        elif t == "reload_plant":
            rt.set_plant(load_plant(use_fitted=m.get("fitted", True)))
            rt.note("plant model reloaded" + (" from the sysid fit" if m.get("fitted", True) else " (nominal)"))
            ws.send({"type": "hello_partial", "status": rt.status(), "fit": read_fit()})
        elif t == "refresh":
            ws.send(self.hello())
        if err:
            ws.send({"type": "error", "msg": err})

    def pump(self):
        """Fan-out thread: samples at ~30 Hz, status at 5 Hz, job progress."""
        seqs: dict[int, int] = {}
        k = 0
        last_msgs = 0
        while True:
            time.sleep(1 / 30)
            k += 1
            live = [c for c in self.clients if c.open]
            self.clients = live
            for c in live:
                rows, seqs[id(c)] = self.rt.samples_since(seqs.get(id(c), self.rt.ring_seq - 200))
                if rows:
                    c.send({"type": "samples", "rows": [list(map(lambda v: round(float(v), 5), r)) for r in rows]})
            if k % 6 == 0:
                st = self.rt.status()
                msgs = list(self.rt.messages)
                new_msgs = [m for m in msgs if m["t"] > last_msgs]
                if new_msgs:
                    last_msgs = new_msgs[-1]["t"]
                for c in live:
                    c.send({"type": "status", "status": st, "msgs": new_msgs,
                            "info": self.rt.ctrls[self.rt.mode].info})
            if k % 10 == 0:
                prog = self.jobs.poll()
                if prog:
                    fin = any(p.get("phase") == "finished" for p in prog)
                    refit = any(p.get("phase") == "finished" and p.get("rc") == 0 and p.get("kind") in ("sysid", "improve", "train") for p in prog)
                    for c in live:
                        c.send({"type": "job", "events": prog, "running": self.jobs.running(), "kind": self.jobs.kind})
                        if fin:
                            c.send({"type": "hello_partial", "policies": registry(), "runs": list_runs()[:200],
                                    "fit": read_fit(), "schemas": schemas()})
                    if refit:
                        # a finished improve/sysid may have refit the plant: pick it up when disarmed
                        if not self.rt.armed:
                            self.rt.set_plant(load_plant())
                            self.rt.ctrls["ppo"].set_params({})
            if k % 90 == 0:
                for c in live:
                    c.send({"type": "runs", "runs": list_runs()[:200]})


def list_runs() -> list[dict]:
    out = []
    if not RUNS.exists():
        return out
    for j in sorted(RUNS.glob("*.json"), reverse=True):
        try:
            m = json.loads(j.read_text())
        except (OSError, ValueError):
            continue
        out.append({k: m.get(k) for k in ("id", "backend", "sim", "mode", "duration_s", "end", "policy", "ppo_episode")})
    return out


def read_fit():
    try:
        return json.loads(FITTED.read_text())
    except (OSError, ValueError):
        return None


def make_handler(app: App):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="text/plain", extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
                return self._ws()
            if path == "/":
                path = "/index.html"
            if path == "/course/handout.pdf":
                pdf = ROOT / "course_original" / "PS_Mission_Impossible_CartPole4_0.pdf"
                if pdf.is_file():
                    return self._send(200, pdf.read_bytes(), "application/pdf")
                return self._send(404, b"Course handout is not included in this installation")
            if path.startswith("/api/"):
                return self._api(path)
            if path.startswith("/materials/"):
                from .dashboard import MATERIALS
                resource = MATERIALS.get(path[len("/materials/"):])
                if resource and resource[1].is_file():
                    f = resource[1]
                    ctype = {".pdf": "application/pdf", ".md": "text/plain; charset=utf-8", ".zip": "application/zip"}[f.suffix]
                    extra = {"Content-Disposition": f'attachment; filename="{f.name}"'} if f.suffix == ".zip" else None
                    return self._send(200, f.read_bytes(), ctype, extra)
                return self._send(404, b"Material not included in this installation")
            f = (WEB / path.lstrip("/")).resolve()
            if WEB.resolve() in f.parents and f.is_file():
                return self._send(200, f.read_bytes(), MIME.get(f.suffix, "application/octet-stream"))
            self._send(404, b"not found")

        def _api(self, path):
            if path == "/api/dashboard":
                from .dashboard import snapshot
                return self._send(200, json.dumps(snapshot(), default=_jsonable).encode(), "application/json")
            if path.startswith("/api/evidence/"):
                from .dashboard import REPORTS
                report = REPORTS.get(path[len("/api/evidence/"):])
                if report and report.is_file():
                    return self._send(200, report.read_bytes(), "application/json")
                return self._send(404, b"Report not available")
            if path == "/api/runs":
                return self._send(200, json.dumps(list_runs()).encode(), "application/json")
            if path == "/api/policies":
                return self._send(200, json.dumps(registry(), default=float).encode(), "application/json")
            if path.startswith("/api/runs/") and path.endswith(".csv"):
                rid = path[len("/api/runs/"):-4]
                f = RUNS / f"{rid}.npy"
                if f.parent.resolve() != RUNS.resolve() or not f.exists():
                    return self._send(404, b"no such run")
                arr = np.load(f)
                lines = [",".join(COLS)] + [",".join(f"{v:.6g}" for v in row) for row in arr]
                return self._send(200, ("\n".join(lines) + "\n").encode(), "text/csv",
                                  {"Content-Disposition": f'attachment; filename="{rid}.csv"'})
            if path.startswith("/api/runs/") and path.endswith(".json"):
                rid = path[len("/api/runs/"):-5]
                f = RUNS / f"{rid}.json"
                if f.parent.resolve() == RUNS.resolve() and f.is_file():
                    return self._send(200, f.read_bytes(), "application/json")
            return self._send(404, b"unknown api")

        def do_POST(self):
            if self.path not in ("/api/friction", "/api/import"):
                self.close_connection = True
                return self._send(404, b"unknown api")
            # Browsers must originate from this local console, not another website.
            origin = self.headers.get("Origin")
            if (origin and urlparse(origin).netloc != self.headers.get("Host")) or self.headers.get("Sec-Fetch-Site") == "cross-site":
                self.close_connection = True
                return self._send(403, b"Same-origin requests only")
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8 * 1024 * 1024:
                    raise ValueError("Upload must be between 1 byte and 8 MB")
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise ValueError("Expected application/json")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("Expected a JSON object")
                with app.rt.lock:
                    if app.rt.armed or app.jobs.running():
                        return self._send(409, b'{"error":"Stop the controller and wait for the current job before analyzing or importing"}', "application/json")
                    from .dashboard import analyze, import_recording
                    result = analyze(payload, app.rt.plant) if self.path == "/api/friction" else import_recording(payload)
                return self._send(200, json.dumps(result, default=_jsonable).encode(), "application/json")
            except (ValueError, OSError, KeyError, TypeError) as error:
                self.close_connection = True
                return self._send(400, json.dumps({"error": str(error)}).encode(), "application/json")

        def _ws(self):
            key = self.headers.get("Sec-WebSocket-Key", "")
            acc = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
            self.send_response(101, "Switching Protocols")
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", acc)
            self.end_headers()
            self.wfile.flush()
            ws = WS(self.connection)
            with app.rt.lock:
                if app.demo_pending:
                    app.demo_pending = False
                    error = app.rt.start_demo()
                    if error:
                        app.rt.note(error)
            ws.send(app.hello())
            app.clients.append(ws)
            try:
                while ws.open:
                    msg = ws.recv()
                    if msg is None:
                        break
                    try:
                        app.handle(ws, json.loads(msg))
                    except Exception as e:  # noqa: BLE001
                        ws.send({"type": "error", "msg": f"{type(e).__name__}: {e}"})
            except (ConnectionError, OSError):
                pass
            finally:
                ws.open = False
                if not app.clients or all(not c.open for c in app.clients):
                    app.rt.disarm("last browser tab closed")
            self.close_connection = True

    return H


def serve(runtime: Runtime, host: str = "127.0.0.1", port: int = 8000, demo: bool = False):
    app = App(runtime, demo=demo)
    # bind the port BEFORE touching the hardware: if the port is taken, the board
    # is never opened and the amplifier never enabled
    httpd = ThreadingHTTPServer((host, port), make_handler(app))
    httpd.daemon_threads = True
    started = False
    try:
        runtime.start()
        started = True
        threading.Thread(target=app.pump, daemon=True, name="pump").start()
        runtime.note(f"cart-pole UI on http://{host}:{port}  (backend: {runtime.backend.info()['name']})")
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if started:
            runtime.disarm("server shutdown")
        runtime.stop() if started else runtime.backend.close()
        httpd.server_close()
