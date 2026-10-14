# 03 — Energy swing-up with energy-neutral cart centring

**Date:** 2026-10-01 · **Status:** accepted

## Context
Track ±17 cm usable (lab watchdog ±20 cm). Åström–Furuta pumping x'' = k(E_t−E)·a'cos a plus a
spring-damper to the centre failed: the spring drains exactly the energy the pole needs (energy
reached 0.03 mgl, pole never closer than 37° to upright), and with the spring at √20 ≈ 4.5 rad/s
near the pole's 6.9 rad/s the cart resonated into the barrier. Random search with that design:
0 of 400 candidates passed 3 mismatched plants.

## Decision
Weight the centring by ε/(w²+ε), w = a'cos a: the cart is re-centred only when moving it cannot
change the pole's energy (top of a swing, crossing horizontal). Energy normalised by mgl; desired
x'' → volts by exact model inversion (x'' is affine in V). Catch: LQR (Q x 5, Q a 100, R 0.05)
inside 35°, give up beyond 45°. Defaults from a 160-candidate search over 7 plants:
k_e 5, a_max 5, k_x 2, k_v 2, ε 3.

## Consequences
7/7 on the bench plants, upright in 2.55 s, cart within 17 cm. Fails when pole friction is
2.5× the prior (B_p = 0.006): the pump's power on this track equals the bearing's dissipation,
the pole tops out at 74°. Per-swing target adaptation was tried and made other plants worse;
it stays as an option (`adapt`, default 0). Sysid measures B_p before this matters.
