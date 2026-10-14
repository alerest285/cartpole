# 02 — Velocities from a tuned Kalman filter, not the lab's 60 rad/s derivative filter

**Date:** 2026-10-01 · **Status:** accepted

## Context
The lab models differentiate x and alpha through w²s/(s²+√2ws+w²), w = 60 rad/s. In closed loop:
- the energy swing-up failed on the nominal plant with that filter and worked with exact states:
  ~20 ms lag on x_dot, multiplied by ~10 N s/m of back-EMF in the model inversion;
- a constant-velocity Kalman filter with q_x = 50, q_a = 2000 fixed the swing-up but put
  **3.0 V RMS** of noise on the motor while balancing (the derivative filter: 0.09 V).

## Decision
Kalman per axis (`estimator.py`), q_x = 2, q_a = 0.3, measurement variance = quantisation²/12.
Grid over q_x ∈ {0.5…50}, q_a ∈ {0.2…2000}: swing-ups 8/8 everywhere; balancing noise falls
monotonically with q_a (2000: 3.0 V, 20: 0.96 V, 0.2: 0.26 V). Chosen q_a = 0.3: **0.30 V RMS**,
all benchmarks 7/7. The lab filter stays selectable in Setup.

## Consequences
The RL env uses this filter's steady-state gains: changing q means fine-tuning the policy
(v002 is v001 fine-tuned after this change). Revisit with real encoder noise and backlash.
