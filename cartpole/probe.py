"""Step-by-step check of the link to the rig. Run on the lab PC:

    python -m cartpole probe          (reads only, nothing moves)
    python -m cartpole probe --move   (adds a +-1 V pulse; the cart moves a few mm)

Each step prints PASS / FAIL and, on failure, what to do. Nothing here needs
the web UI. `--fake` runs the same steps against the simulated board.
"""
from __future__ import annotations

import math
import time

import numpy as np

from .params import load_hardware, save_hardware_override

OK, BAD, INFO = "PASS", "FAIL", "    "


def _say(tag, msg):
    print(f"[{tag}] {msg}", flush=True)


def probe(fake: bool = False, move: bool = False) -> int:
    failures = []
    def say(tag, msg):
        if tag == BAD:
            failures.append(msg)
        _say(tag, msg)

    hw = load_hardware()
    print(f"Cart-pole probe. Board {hw.board_type}-{hw.board_id}, encoders {hw.enc_channels}, "
          f"AO {hw.ao_channel}, amp enable DO {hw.do_channels}\n")
    # 1. API
    if fake:
        from .backends.quanser import FakeHIL
        HIL = lambda: FakeHIL()  # noqa: E731
        say(OK, "using the simulated board (--fake)")
    else:
        try:
            from quanser.hardware import HIL  # type: ignore
            import quanser  # type: ignore
            say(OK, f"Quanser Python API found at {getattr(quanser, '__path__', ['?'])[0]}")
        except ImportError:
            say(BAD, "Quanser Python API not importable (`from quanser.hardware import HIL`).")
            say(INFO, "Fix: see docs/lab-runbook.md, section 'Install the Quanser Python API'.")
            say(INFO, 'Usually: python -m pip install --upgrade --find-links "%QSDK_DIR%python" quanser-api')
            return 1
    # 2. open
    card = HIL()
    try:
        card.open(hw.board_type, hw.board_id)
        say(OK, f"opened {hw.board_type}-{hw.board_id}")
    except Exception as e:  # noqa: BLE001
        say(BAD, f"could not open the board: {e}")
        say(INFO, "Likely: a QUARC/Simulink model is still running and holds the board. Stop it")
        say(INFO, "(Simulink: Stop; or QUARC tray icon -> Stop all), unplug/replug the USB, retry.")
        say(INFO, "If the board is not a Q2-USB, set board_type in data/hardware.json (e.g. q8_usb).")
        return 1
    enc_ch = np.array(hw.enc_channels, dtype=np.uint32)
    buf = np.zeros(len(enc_ch), dtype=np.int32)
    ao = np.array([hw.ao_channel], dtype=np.uint32)
    try:
        card.write_analog(ao, 1, np.zeros(1))
        say(OK, "analog output 0 V")
        try:
            card.watchdog_set_analog_expiration_state(ao, 1, np.zeros(1))
            card.watchdog_start(hw.hil_watchdog_s)
        except Exception as e:
            say(BAD, f"watchdog unavailable; probe stopped before enabling amplifier: {e}")
            return 1

        def renew():
            if not card.watchdog_reload():
                raise RuntimeError("board watchdog expired; probe aborted")

        try:
            card.write_digital(np.array(hw.do_channels, dtype=np.uint32), len(hw.do_channels),
                               np.ones(len(hw.do_channels), dtype=np.int8))
            say(OK, f"amplifier enable written on DO {list(hw.do_channels)}")
        except Exception as e:  # noqa: BLE001
            say(BAD, f"digital write failed ({e}); the amplifier may stay disabled")
        card.set_encoder_counts(enc_ch, len(enc_ch), np.zeros(len(enc_ch), dtype=np.int32))
        say(OK, "encoders zeroed: put the cart in the middle and let the pole hang BEFORE this step")
        # 3. live encoders
        say(INFO, "Live readout for 10 s. Move the cart by hand to the RIGHT (seen from the front):")
        say(INFO, "x must INCREASE. Tilt the pole counter-clockwise: alpha must INCREASE from -180.")
        t_end = time.time() + (0.5 if fake else 10.0)
        xs, as_ = [], []
        while time.time() < t_end:
            renew()
            card.write_analog(ao, 1, np.zeros(1))
            card.read_encoder(enc_ch, len(enc_ch), buf)
            x = buf[0] * hw.cart_m_per_count
            a = buf[1] * hw.pole_rad_per_count - math.pi
            xs.append(x); as_.append(a)
            print(f"\r      counts {buf[0]:7d} {buf[1]:6d}   x {x*100:+7.2f} cm   alpha {math.degrees(a):+8.2f} deg   ", end="", flush=True)
            if fake:
                card.advance() if hasattr(card, "advance") else None
            time.sleep(0.02)
        print()
        moved = (max(xs) - min(xs)) > 0.002 or (max(as_) - min(as_)) > 0.02
        say(OK if moved or fake else BAD, "encoders change when moved" if moved or fake else
            "no encoder change seen: check the encoder cables (cart on channel 0, pole on channel 1)")
        # 4. watchdog
        try:
            renew()
            say(OK, "board watchdog configured and reloading (physical output expiration still needs a rig check)")
        except Exception as e:  # noqa: BLE001
            say(BAD, f"watchdog failed: {e}")
            return 1
        # 5. direction
        if move:
            if failures:
                say(BAD, "motion skipped because earlier checks failed")
                return 1
            card.read_encoder(enc_ch, len(enc_ch), buf)
            if abs(buf[0] * hw.cart_m_per_count) > 0.02:
                say(BAD, "cart is not centred; return it to the zero position and rerun")
                return 1

            def pulse(v, duration):
                until = time.monotonic() + duration
                while time.monotonic() < until:
                    renew()
                    card.read_encoder(enc_ch, len(enc_ch), buf)
                    if abs(buf[0] * hw.cart_m_per_count) > 0.04:
                        raise RuntimeError("probe motion exceeded 4 cm; output stopped")
                    card.write_analog(ao, 1, np.array([v * hw.volts_sign]))
                    if fake:
                        card.advance()
                    time.sleep(0.002)

            say(INFO, "Applying +1 V for 150 ms, then -1 V for 150 ms. Keep hands clear.")
            card.read_encoder(enc_ch, len(enc_ch), buf); c0 = int(buf[0])
            for v, dur in ((1.0, 0.15), (0.0, 0.05)):
                pulse(v, dur)
            card.read_encoder(enc_ch, len(enc_ch), buf); c1 = int(buf[0])
            pulse(-1.0, 0.15)
            card.write_analog(ao, 1, np.zeros(1))
            dx = (c1 - c0) * hw.cart_m_per_count
            if abs(dx) < 2e-4:
                say(BAD, f"cart barely moved ({dx*1000:.2f} mm): amplifier off? enable switch, power, DO channels")
            elif dx > 0:
                say(OK, f"+1 V moved the cart +{dx*1000:.1f} mm: voltage sign matches the model")
            else:
                say(BAD, f"+1 V moved the cart {dx*1000:.1f} mm (the wrong way)")
                if not fake:
                    save_hardware_override(volts_sign=-hw.volts_sign)
                    say(INFO, f"wrote volts_sign = {-hw.volts_sign:+.0f} to data/hardware.json; run the probe again")
        # 6. timing
        n, per = 1000, []
        t_prev = time.perf_counter(); nxt = t_prev
        for _ in range(n):
            nxt += 0.002
            while time.perf_counter() < nxt:
                pass
            renew()
            card.read_encoder(enc_ch, len(enc_ch), buf)
            card.write_analog(ao, 1, np.zeros(1))
            now = time.perf_counter(); per.append(now - t_prev); t_prev = now
        per = np.array(per[1:]) * 1000
        good = per.max() < 4.0
        say(OK if good else BAD, f"500 Hz software loop: mean {per.mean():.3f} ms, max {per.max():.2f} ms"
            + ("" if good else " (try --timing hw, close other programs)"))
    except Exception as e:
        say(BAD, f"probe aborted: {e}")
    finally:
        try:
            card.write_analog(ao, 1, np.zeros(1))
            card.write_digital(np.array(hw.do_channels, dtype=np.uint32), len(hw.do_channels),
                               np.zeros(len(hw.do_channels), dtype=np.int8))
        except Exception:  # noqa: BLE001
            pass
        try:
            card.watchdog_stop()
        except Exception:
            pass
        card.close()
        say(INFO, "board closed, output 0 V")
    if failures:
        print("\nResolve the failed checks before starting a hardware controller.")
    else:
        print("\nNext: python -m cartpole serve --backend " + ("fake" if fake else "quanser"))
    return 1 if failures else 0
