# 01 — Drive the rig from Python through Quanser's HIL API, not from Simulink

**Date:** 2026-10-01 · **Status:** accepted

## Context
The course runs controllers as QUARC Simulink models on the lab PC. A browser UI that switches
methods live, tunes gains live, logs every run and trains a policy needs a long-running process
that owns the loop. Simulink external mode can tune parameters but not swap controllers,
host a web server or run PPO.

## Decision
`cartpole/backends/quanser.py` opens the Q2-USB with `quanser.hardware.HIL` (the Python face of
the same HIL layer QUARC uses) and runs the 500 Hz loop in a Python thread: `read_encoder`
(channels 0, 1) → estimator → controller → safety → `write_analog` (channel 0), amplifier
enabled on digital 0–3, board watchdog 100 ms. Everything else (channels, scalings, signs,
watchdog limit) is read from the course's seven `.slx` files.

## Consequences
- Simulink and this program cannot hold the board at the same time (runbook §0).
- Python timing: software pacing measured 2.000 ms mean / 2.04 ms max on the Mac with the fake
  board; on Windows unknown until `probe` runs. `--timing hw` uses a hardware-clocked reader task.
- The whole hardware path is exercised on the Mac through `FakeHIL` (same method signatures,
  simulated physics, watchdog): tests `HardwarePath.*`.
- Unverified: the wheel's location on the lab PC and the HIL calls' exact behaviour on the real
  board. `python -m cartpole probe` is the first thing to run there.
