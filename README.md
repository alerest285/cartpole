# cartpole

Balance and swing up the IfA cart-pole (P&S *Mission Impossible: CartPole 4.0*, HS2026)
from a browser, with classic and learned methods side by side, and a PPO training pipeline that
uses recorded rig sessions for system identification and policy updates.

## Start here (2026-10-14)

- **Integrated dashboard:** opens at http://127.0.0.1:8000 with the live plant, commissioning
  notes, friction analysis/upload, the active model, all six course sections, benchmark
  and uncertainty results, offline jobs, both PDFs and the portable bundle. **Control room**
  retains the full plots, controller tuning, PPO learning, run history and hardware setup.
  Stop the controller before importing/analyzing data or launching an offline job.
  CSV imports default to simulation; mark a file as physical only if it came from the rig.

- **Tomorrow's rig session:** [commissioning order](docs/tomorrow.md) and
  [every handout task](docs/course-tasks.md). The browser's **Course** tab links the original
  PDF and lists exercise commands. Physical experiments remain pending.

- **Mac simulator:** double-click `start_sim.command`, then open http://127.0.0.1:8000.
  The first browser connection starts energy swing-up automatically. **Run demo** restarts it
  from any tab. STOP stays stopped across reloads. LQR/PID/pole placement start upright.
  The scene and ten-second plots follow a shared display clock, with smooth motion between
  telemetry samples. Resets snap to the new state; interrupted telemetry freezes the display.
- **MATLAB:** open `matlab/` and run `demo_cartpole`. Run `build_cartpole_simulink`
  for a simulation model with scopes. MATLAB R2024b is installed here but needs license
  activation; these new MATLAB files pass its code analyzer but numerical execution is pending.
- **Lab PC:** extract `dist/cartpole-lab.zip`, run `setup_lab.cmd`, and follow
  [docs/lab-runbook.md](docs/lab-runbook.md). The bundle includes the original course templates.
  `matlab/build_course_lqr.m` prepares a disarmed LQR copy for the QUARC route; compilation
  and real-rig checks are pending.

Verification: **45/45 Python tests**, **49/49 benchmark cases**, fake-board probe and browser
checks (energy, PPO, STOP, run logging) passed. No controller has been tested on the
physical rig. Six presentation tests and the browser fluidity check also pass.
[Current status](docs/status/2026-10-14-dashboard.md).

```
python -m cartpole serve                     # simulator, opens http://127.0.0.1:8000
python -m cartpole serve --backend quanser   # the real rig, on the lab PC (docs/lab-runbook.md)
```

Needs Python 3.9+ and numpy. On the lab PC also Quanser's `quanser_api` wheel.

## What it does

| Method | Key | From | Simulated result on 7 mismatched plants |
|---|---|---|---|
| PID, two loops | `pid` | tilted, tapped | 7/7 |
| Pole placement | `statefb` place | tilted, tapped | 7/7 |
| LQR (discrete or continuous) | `statefb` lqr | tilted, tapped | 7/7 |
| Energy swing-up (Lyapunov) + LQR catch | `energy` | hanging | 7/7, upright in 2.55 s |
| Phase-locked sinusoid + LQR catch | `sine` | hanging | 7/7, upright in 2.74 s |
| PPO policy (learned) | `ppo` | hanging | 7/7, upright in 1.85 s |
| Excitation for system identification | `excite` | hanging | — |

"Mismatched": the controllers use the model; the simulated plant has friction −40 %/+60 %,
amplifier gain ±10 %, 2 ms extra delay or 0.3 N Coulomb friction (`python -m cartpole bench`).

Every armed stretch is logged to `runs/`. **Improve** (Learn tab) refits the plant from those
logs, runs a PPO update on the exploration episodes recorded on the rig, fine-tunes in the
refit simulator and promotes the new version only if it is not worse; the policy table shows
each version's simulated and real success.

## Safety, independent of the controller

Voltage cap · control-barrier filter keeping the cart inside ±17 cm · hard stop at ±19 cm
(the lab's watchdog is ±20 cm) · STOP button and space bar · dead-man: the browser's 10 Hz
heartbeat, 0.5 s · board watchdog: 0 V if Python stalls 100 ms · amplifier disabled on exit.

## Layout

```
cartpole/        params, dynamics, lin (lqr/place/c2d in numpy), estimator, safety, runtime (500 Hz),
                 server (stdlib HTTP + WebSocket), sysid, probe, bench
  controllers/   classic (idle, manual, pid, statefb, excite), swingup (energy, sine), learned (ppo)
  backends/      sim, quanser (Q2-USB via quanser.hardware.HIL; FakeHIL for tests)
  rl/            policy (numpy MLP), env (vectorised, randomised), ppo, train, improve
web/             simulator, timestamped plots, manual pad and Course guide (no build step)
matlab/          simulator, friction analysis, course design exercises and eight QUARC model builders
tests/           python -m cartpole selftest (41 tests, no hardware)
                 node --test tests/test_timeline.cjs (6 presentation tests)
docs/            lab-runbook, project, decisions, status, worklog
```
