# 07 — Safety is a layer every voltage passes, independent of the controller

**Date:** 2026-10-01 · **Status:** accepted

## Decision
1. Voltage cap (≤ 10 V, adjustable). 2. Control barrier function on the track, exponential CBF
h = x_soft − |x|, k0 = 144, k1 = 24, exact because x'' is affine in V: the QP is a clamp; with a
controller demanding +10 V forever the cart stops below 17.5 cm (test). 3. Hard fault and disarm
at |x| > 19 cm, pole speed > 60 rad/s, NaN. 4. Dead-man: on hardware the loop disarms if no
browser heartbeat for 0.5 s; closing the last tab disarms. 5. Board watchdog 100 ms → 0 V.
6. Amplifier disabled and 0 V on every exit path.

## Consequences
The RL env includes the same barrier, so the policy is trained with it in the loop. Swing-ups use
up to 17 cm; the barrier and the 19 cm hard stop sit inside the lab's 20 cm watchdog.
