# cartpole — project

**Owner:** Ale · **Started:** 2026-10-01 · **Status:** Python simulation verified and running; MATLAB/Simulink source and lab bundle prepared
(2026-10-14). Browser display now follows timestamped telemetry smoothly. MATLAB activation
and first hardware session pending. Course task coverage, friction identification and
first-session preparation and integrated dashboard are documented in `docs/status/2026-10-14-dashboard.md`.

## Why
P&S *Mission Impossible: CartPole 4.0* (IfA, 2 KP, pass/fail): model the rig, stabilise the pole,
swing it up, on the real hardware. Ale's brief (01.10): models that fully balance the pendulum
with different methods, a web app to control it once on the lab PC, and PPO that "learns and
gets better with more deployments … fully end to end".

## Stack and why
- **Python + numpy only** (the lab PC may have no admin rights and no PyTorch): physics, LQR/
  pole placement/c2d (`lin.py`), Kalman, PPO with manual backprop.
- **Quanser's Python HIL API** for the Q2-USB (decision 01) — the same layer QUARC uses.
- **stdlib HTTP + WebSocket** server, static HTML/CSS/JS UI, no build step.
- **Subprocess jobs** for training so the 500 Hz loop never shares the GIL with PPO.

## Components
| File | Role |
|---|---|
| `params.py` | plant constants (handout + `.slx`), hardware map, sysid overrides |
| `dynamics.py` | nonlinear model (PDF eq. L1/L2 + motor), batched; linearisation |
| `lin.py` | expm, c2d, dare (doubling), dlqr, care, lqr, place (Ackermann), ctrb, obsv |
| `estimator.py` | lab derivative filter, CV Kalman (default, decision 02) |
| `controllers/` | idle, manual, pid, statefb, energy, sine, excite, ppo — each a schema the UI renders |
| `safety.py` | cap, track CBF, hard faults (decision 07) |
| `runtime.py` | the 500 Hz loop, arming, dead-man, run logging |
| `backends/` | sim; quanser (+ FakeHIL) |
| `server.py`, `web/` | the console; buffered presentation clock in `web/timeline.js` (decision 09) |
| `sysid.py` | least-squares plant fit from runs (decision 06) |
| `friction.py`, `import_run.py` | course B_f exercise, held-out validation, MATLAB CSV import |
| `robustness.py`, `energy_search.py` | offline uncertainty sweeps and scalar-checked batched search |
| `playground.py` | offline show, ILC and sampling MPC coursework |
| `rl/` | policy, env, ppo, train (versions, eval, promotion), improve (decision 05) |
| `probe.py`, `bench.py` | lab-PC link check; controller × plant table |
