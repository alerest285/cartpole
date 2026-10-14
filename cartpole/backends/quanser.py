"""The real rig: Quanser Q2-USB through Quanser's Python HIL API.

The lab PC runs QUARC, which talks to the board through the HIL layer. The same
layer is exposed to Python as `quanser.hardware.HIL` (the `quanser_api` wheel
that ships with QUARC and with the free Quanser SDK). See docs/lab-runbook.md
for installation and `python -m cartpole probe` for a step-by-step check.

Channel map (read from the course Simulink models, 2026-10-01):
    encoder 0 = cart, encoder 1 = pole, analog out 0 = amplifier,
    digital out 0..3 = 1 enables the VoltPAQ (swingup_mouse.slx).

Only the basic, long-stable HIL calls are relied on: open, read_encoder,
write_analog, write_digital, set_encoder_counts, close. The board watchdog
is required. A failed or expired watchdog aborts the connection. The optional
hardware-clocked reader can fall back to software timing.

IMPORTANT: a running Simulink/QUARC model holds the board. Stop every QUARC
model before starting this backend, and vice versa.
"""
from __future__ import annotations

import time

import numpy as np

from ..params import Hardware
from .base import Backend, Pacer


def load_hil():
    """Import the Quanser HIL class. Raises ImportError with instructions."""
    try:
        from quanser.hardware import HIL  # type: ignore
        return HIL
    except ImportError as e:  # pragma: no cover - depends on the lab PC
        raise ImportError(
            "Quanser's Python API is not installed. On the lab PC run:\n"
            "  python -m pip install --upgrade --find-links \"%QSDK_DIR%python\" quanser-api\n"
            "or, if only QUARC is installed, the wheel in \"%QUARC_DIR%\\python\".\n"
            "Details: docs/lab-runbook.md") from e


class QuanserBackend(Backend):
    name = "quanser"
    is_sim = False

    def __init__(self, hw: Hardware | None = None, hil_factory=None, timing: str = "sw", log=print):
        super().__init__(hw)
        self.hil_factory = hil_factory
        self.timing = timing
        self.log = log
        self.card = None
        self.pacer = Pacer()
        self.task = None
        self.watchdog = False
        self.warnings: list[str] = []
        h = self.hw
        self.enc_ch = np.array(h.enc_channels, dtype=np.uint32)
        self.enc_buf = np.zeros(len(h.enc_channels), dtype=np.int32)
        self.ao_ch = np.array([h.ao_channel], dtype=np.uint32)
        self.ao_buf = np.zeros(1, dtype=np.float64)
        self.do_ch = np.array(h.do_channels, dtype=np.uint32)
        self.v_last = 0.0

    # ------------------------------------------------------------------
    def _warn(self, msg):
        self.warnings.append(msg)
        self.log(f"[quanser] {msg}")

    def open(self):
        HIL = self.hil_factory or load_hil()
        self.card = HIL()
        self.card.open(self.hw.board_type, self.hw.board_id)
        self.log(f"[quanser] opened {self.hw.board_type}-{self.hw.board_id}")
        try:
            self.write_volts(0.0)
            self.zero()
            self._start_watchdog()
            if self.hw.enable_amp and len(self.do_ch):
                self.card.write_digital(self.do_ch, len(self.do_ch), np.ones(len(self.do_ch), dtype=np.int8))
            if self.timing == "hw":
                self._start_task()
        except Exception:
            self.close()
            raise

    def _start_watchdog(self):
        try:
            self.card.watchdog_set_analog_expiration_state(self.ao_ch, 1, np.zeros(1, dtype=np.float64))
            self.card.watchdog_start(self.hw.hil_watchdog_s)
            self.watchdog = True
            self.log(f"[quanser] board watchdog armed: outputs go to 0 V if the loop stalls {self.hw.hil_watchdog_s*1000:.0f} ms")
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"board watchdog unavailable; hardware start refused: {e}") from e

    def _start_task(self):
        try:
            from quanser.hardware import Clock  # type: ignore
            self.task = self.card.task_create_encoder_reader(100, self.enc_ch, len(self.enc_ch))
            self.card.task_start(self.task, Clock.HARDWARE_CLOCK_0, 1.0 / 0.002, 2 ** 31 - 1)
            self.log("[quanser] hardware-clocked encoder task running at 500 Hz")
        except Exception as e:  # noqa: BLE001
            self.task = None
            self._warn(f"hardware-clocked task unavailable ({e}); using software timing")

    def close(self):
        if self.card is None:
            return
        try:
            # Do not reload an expired watchdog during cleanup.
            self.card.write_analog(self.ao_ch, 1, np.zeros(1, dtype=np.float64))
        except Exception:
            pass
        finally:
            for f in (self._stop_task, self._stop_watchdog, self._disable_amp):
                try:
                    f()
                except Exception:  # noqa: BLE001
                    pass
            try:
                self.card.close()
            except Exception:  # noqa: BLE001
                pass
            self.card = None
            self.log("[quanser] closed, output 0 V")

    def _stop_task(self):
        if self.task is not None:
            self.card.task_stop(self.task)
            self.card.task_delete(self.task)
            self.task = None

    def _stop_watchdog(self):
        if self.watchdog:
            self.card.watchdog_stop()
            self.watchdog = False

    def _disable_amp(self):
        if self.hw.enable_amp and len(self.do_ch):
            self.card.write_digital(self.do_ch, len(self.do_ch), np.zeros(len(self.do_ch), dtype=np.int8))

    # ------------------------------------------------------------------
    def read_counts(self):
        if self.task is not None:
            self.card.task_read_encoder(self.task, 1, self.enc_buf)
        else:
            self.card.read_encoder(self.enc_ch, len(self.enc_ch), self.enc_buf)
        return int(self.enc_buf[0]), int(self.enc_buf[1])

    def write_volts(self, v: float):
        if self.watchdog:
            try:
                if not self.card.watchdog_reload():
                    raise RuntimeError("board watchdog expired; restart the connection")
            except Exception:
                # Preserve the expired board's protective state; do not clear it.
                self.card.write_analog(self.ao_ch, 1, np.zeros(1, dtype=np.float64))
                raise
        v = float(np.clip(v * self.hw.volts_sign, -self.hw.v_limit, self.hw.v_limit))
        self.ao_buf[0] = v
        self.card.write_analog(self.ao_ch, 1, self.ao_buf)
        self.v_last = v

    def zero(self):
        self.card.set_encoder_counts(self.enc_ch, len(self.enc_ch), np.zeros(len(self.enc_ch), dtype=np.int32))

    def wait_tick(self):
        if self.task is None:      # with the task, read_counts already blocks on the clock
            self.pacer.wait()

    def info(self):
        return {"name": f"{self.hw.board_type}-{self.hw.board_id}", "sim": False,
                "timing": "hardware clock" if self.task is not None else "software",
                "watchdog": self.watchdog, "overruns": self.pacer.overruns,
                "warnings": self.warnings[-5:]}


class FakeHIL:
    """Stand-in for quanser.hardware.HIL with the same method signatures,
    backed by the simulator. Lets the hardware path run end to end on a Mac,
    including the board watchdog (outputs drop to 0 V when not reloaded)."""

    def __init__(self, sim=None):
        from .sim import SimPlant
        self.sim = sim or SimPlant()
        self.ao = 0.0
        self.do = np.zeros(8, dtype=np.int8)
        self.wd_timeout = None
        self.wd_last = None
        self.wd_expired = False
        self.opened = None
        self.calls: list[str] = []

    def open(self, board_type, board_id):
        self.opened = (board_type, board_id)
        self.calls.append("open")

    def close(self):
        self.calls.append("close")
        self.opened = None

    def read_encoder(self, channels, num_channels, buffer):
        c, p = self.sim.counts()
        vals = {0: c, 1: p}
        for i in range(num_channels):
            buffer[i] = vals[int(channels[i])]

    def set_encoder_counts(self, channels, num_channels, buffer):
        self.sim.zero()

    def write_analog(self, channels, num_channels, buffer):
        self.ao = float(buffer[0])

    def write_digital(self, channels, num_channels, buffer):
        for i in range(num_channels):
            self.do[int(channels[i])] = buffer[i]

    def watchdog_set_analog_expiration_state(self, channels, num_channels, buffer):
        self.calls.append("wd_state")

    def watchdog_start(self, timeout):
        self.wd_timeout = timeout
        self.wd_last = time.perf_counter()

    def watchdog_reload(self):
        if self.wd_timeout is not None and time.perf_counter() - self.wd_last > self.wd_timeout:
            self.wd_expired = True
        if self.wd_expired:
            self.ao = 0.0
            return False
        self.wd_last = time.perf_counter()
        return True

    def watchdog_stop(self):
        self.wd_timeout = None

    # the physics advances by one tick per call; the backend's pacer calls it
    def advance(self):
        if self.wd_timeout is not None and time.perf_counter() - self.wd_last > self.wd_timeout:
            self.wd_expired = True
        v = 0.0 if (self.wd_expired or not self.do[:4].all()) else self.ao
        self.sim.step(v)


class FakeQuanserBackend(QuanserBackend):
    """The Quanser backend wired to FakeHIL: hardware code path, simulated physics."""
    name = "fake-quanser"
    is_sim = True

    def __init__(self, hw=None, plant=None, realtime=True, log=print):
        from .sim import SimPlant
        self.fake = FakeHIL(SimPlant(plant, hw or Hardware()))
        super().__init__(hw, hil_factory=lambda: self.fake, timing="sw", log=log)
        self.realtime = realtime

    def wait_tick(self):
        self.fake.advance()
        if self.realtime:
            self.pacer.wait()

    def info(self):
        d = super().info()
        d["name"] = "fake Q2-USB (simulated physics)"
        d["sim"] = True
        return d
