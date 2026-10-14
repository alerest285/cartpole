# 05 — PPO in numpy, trained on a deployment-faithful simulator, improved per session

**Date:** 2026-10-01 · **Status:** accepted

## Context
Ale: "make it so that i can do ppo and it optimizes and learns and gets better with more
deployments. make this fully end to end". The lab PC may have no PyTorch and no admin rights.

## Decision
- Policy 2×64 tanh MLP, Gaussian, 50 Hz decisions held for 10 ticks; manual backprop + Adam
  (gradient-checked in tests). 42k policy steps/s on the M4 with 64 vectorised envs.
- The env reproduces deployment: same dynamics, encoder quantisation, the Kalman filter's
  steady-state gains, the barrier filter, delay; randomised c_v ×0.8–1.25, k_amp ×0.9–1.1,
  M ×0.95–1.05, B_p ×0.5–2.5, Coulomb 0–0.3 N, delay 0–1 tick.
- Evaluation always through the real path (`PPOController` → `simulate.closed_loop`) on the
  7-plant set, from hanging and tilted. Promotion only if score ≥ current (re-scored on the
  refit model first, so the comparison is fair).
- Improve = sysid on all hardware runs → randomisation width 1/√(1+minutes of data), floor 0.35
  → one PPO update on hardware exploration episodes (rewards recomputed from logged states with
  the same reward function, importance ratios from the logged log-probs) → warm-started sim
  fine-tune → evaluate → promote → per-version rig scoreboard.

## Consequences
v001 (1500 iters, 19.2 M steps, 8 min): 14/14 eval cases, upright in 1.0 s mean (2.0 s from
hanging). v002 (v001 fine-tuned 400 iters on the new estimator): 14/14, swing-up 1.85 s on the
bench, 0.4 V RMS balancing. The loop itself, on a rig outside the training randomisation and with
one session's data per round (30 s Identify + 8 explore episodes): v002 1/5 → v003 5/5 → v004 5/5,
sysid within 4 % (`tools/deployment_loop_demo.py`, docs/evidence/deployment_loop.json).
