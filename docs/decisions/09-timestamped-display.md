# 09 — Draw from timestamped samples on a buffered display clock

**Date:** 2026-10-14 · **Status:** accepted

The control loop runs at 500 Hz, decimates telemetry to 100 Hz, and sends batches about
30 times per second. The browser previously drew its scene from the status snapshot,
which arrives only five times per second. A browser measurement during the first three
seconds of swing-up found 4.67 visible pose updates/s despite a 60 Hz animation callback.
The charts also shifted in sample batches, with copied arrays and per-sample phase fills.

`web/timeline.js` now owns a bounded 1,200-sample presentation buffer. A local display
clock targets 70 ms behind the most recently received sample at packet arrival, smoothing
the clock offset to absorb arrival jitter. The scene and ten-second plots use the same
clock. Position interpolates linearly; angle uses the shortest arc through ±π. Voltage
uses the last measured value, without smoothing. Plots preserve raw samples and break
at angle wraps, sample gaps, or abrupt reset jumps. No presentation state feeds back
into the controller, estimator, safety checks, or saved run data.

The display never extrapolates beyond received data. After a stream interruption or a
suspended tab, it resumes at the live edge. New hello messages discard older backfilled
samples, preventing a restarted demo from briefly showing its previous state. Ordinary
resets are discontinuities: differences above 0.1 m or π/2 between consecutive samples,
or gaps above 100 ms, break interpolation and plot lines. Such a threshold can also break
a truly extreme measured jump; it never modifies that measurement.

Canvas dimensions update through ResizeObserver and theme colors are cached until the
color scheme changes. Phase bands draw contiguous spans. The scene is taller and numeric
readouts refresh at 10 Hz for legibility. Control/status messages retain their existing
path, so the rendering buffer does not delay STOP or motor commands.

The isolated browser test measured **58 pose and plot updates/s** during startup, **0.20 ms
p95** for the JavaScript drawing callback, and **52.48 ms median lag** behind the latest
received sample at frame time. The callback cost excludes browser compositing; measured
rates apply to this local Chromium run, not every device. Startup buffering accounts
for part of the difference from 60 updates/s. Evidence is in `docs/evidence/fluidity-before.json`
and `fluidity-after.json`.

Six dependency-free Node tests cover angle seams, voltage steps, resets, packet jitter,
freeze/resume, server-clock changes and bounded history. `tools/check_fluidity.py` checks
the real browser path, automatic demo, balance, Setup restart, reset jumps, STOP/reload,
390/800/1440 px layouts, theme changes, hidden-canvas resize, and connection loss.

Revisit the 70 ms buffer if measurements on the lab network show repeated buffer
underruns. Keep any future latency adaptation confined to presentation.
