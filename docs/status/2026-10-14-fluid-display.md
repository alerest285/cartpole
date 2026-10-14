# Status 2026-10-14 — fluid simulator display

The browser scene and plots now update smoothly from the existing 100 Hz telemetry,
using a shared buffered display clock. The isolated Chromium check measured **58 visible
pose and plot updates/s**, up from **4.67 pose updates/s** before the change. The first
three seconds include startup buffering. JavaScript drawing callback p95 was 0.20 ms;
that is not a measurement of compositor time or a guarantee for every device.

Changes: a bounded presentation timeline, shortest-arc angle interpolation, continuous
timestamp-based plot scrolling, gaps at angle wraps and resets, cached canvas dimensions
and theme colors, contiguous phase bands, 10 Hz readouts, and a larger scene. Measured
voltage remains stepped. Lost telemetry freezes at the final sample; returning from a
suspended tab catches up to the live stream. Control, safety, estimation, physics,
training, and recorded data are unchanged. Decision 09 explains the 70 ms buffer.

Verification completed:

- `node --test tests/test_timeline.cjs`: 6/6 tests, including jitter, ±π seam, reset,
  stale data, resume, clock restart and bounded history.
- `python -m cartpole selftest`: 31/31 tests, 99.080 s.
- `python -m cartpole bench`: 49/49 controller/plant cases; energy 2.55 s, sine 2.74 s,
  PPO 1.85 s mean swing-up time, unchanged.
- `tools/check_fluidity.py`: automatic demo, Setup restart, swing-up and balance,
  reset without invented swing, STOP/reload, 390/800/1440 px layout, theme changes,
  hidden-canvas resize, and lost-stream freeze. No JavaScript errors.

Evidence: `docs/evidence/fluidity-before.json`, `fluidity-after.json`, `fluidity-live.png`.
The open local simulator tab was refreshed and the live server verified running Energy
in its balancing phase (`fluidity-live-server.json`). The portable lab ZIP includes these
frontend changes and verification tools.
Run the browser check from a Python environment with Playwright/Chromium, optionally
setting `PLAYWRIGHT_CHROMIUM_EXECUTABLE` to the browser binary. No frontend build is needed.

Course sources confirmed and read: the original `course_original/PS_Mission_Impossible_CartPole4_0.pdf`
and Ale's solved derivation at `~/Desktop/Mission Impossible Cart Pole/2026 HS (Ale)/cartpole_model_solved.pdf`.

This snapshot supersedes display/startup details in `2026-10-14-startup-demo-fixed.md`.
The MATLAB licensing and physical rig limitations in `2026-10-14-sim-to-real-ready.md`
still apply: MATLAB source passes its analyzer but has not run under an activated license;
the real rig has not been tested. No commit or push.
