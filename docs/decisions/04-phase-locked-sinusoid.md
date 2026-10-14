# 04 — The sinusoid swing-up locks its phase to the pendulum

**Date:** 2026-10-01 · **Status:** accepted

## Context
Handout step 5: u = F sin(wt). Fixed frequency: 0 of 160 candidates swung up (a pendulum slows as
its swing grows, the drive falls out of resonance; energy stalled at −0.9 mgl). Tracking the
amplitude-dependent frequency w0·π/(2K(sin(θ0/2))) open loop: still 0 of 160 (phase drifts).

## Decision
`how = locked`: the drive phase is the pendulum's own, ψ = atan2(−φ'/w, φ) with φ the angle from
hanging, u = A sin(ψ + lead). In time it is still a sinusoid of amplitude A. Lead −40°, A 8 V,
k_x 5, k_v 1 (6 of 7 plants in the search, 7/7 on the bench set, upright in 2.74 s).
`fixed` and `tracking` stay selectable to show the handout's point on the rig.
