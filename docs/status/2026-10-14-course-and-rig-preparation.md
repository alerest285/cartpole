# Status 2026-10-14 — course coverage and tomorrow's rig preparation

The software side of all numbered course exercises and optional playground topics is
implemented, with a complete [task map](../course-tasks.md) and [first-session procedure](../tomorrow.md).
Physical comparisons, balance/swing-up and TA sign-off are pending. MATLAB R2024b remains
unlicensed locally: source analysis succeeds, numerical execution and QUARC compilation
have not been performed.

Added the scalar friction exercise with Coulomb comparison and held-out validation,
MATLAB workspace/CSV conversion, explicit real/simulated recording import, joint-fit
holdout/conditioning checks, manual swing-up/LQR catch, pointer timeout, 5 V/10 Hz test,
PID anti-windup, stopped-only derivative cutoff tuning, and preservation of PPO's
estimator contract. Hardware starts disarmed at 4 V with reduced Identify excitation.

MATLAB now has course modeling/linear/Q experiments, friction analysis, a 10 cm barrier
QP, offline validation, and a builder for eight disarmed course-model copies with
protected I/O/watchdog logic retained. The full-course controller estimates velocities
from raw positions with angle unwrapping. Applied-voltage logs can enter the shared
Python identification pipeline. The original model-derivation PDF is also included in
`docs/course/` for portability.

The browser's new Course tab links the original handout and lists commands plus physical
work still pending. The live simulator was restarted with the updated server and the
existing browser tab refreshed. `docs/evidence/course-live-server.json` records Energy
running in its balancing phase.

Measured preparation results:

- 41/41 Python tests (54.600 s), 6/6 presentation tests, 49/49 standard benchmark cases.
- Browser: ~58 updates/s, no JavaScript errors; Course/PDF, manual timeout, derivative
  cutoff, swing-up/balance, STOP/reload, mobile/theme and disconnected freeze pass.
- All 27 MATLAB files pass the installed analyzer without diagnostics. Execution pending.
- Nominal 4 V LQR passes 13/13 combined-uncertainty balance cases.
- Broad swing-up candidate search reaches 18/25 fresh cases: **not qualified or installed**.
- Full synthetic lab rehearsal: nominal model fails on a deliberately mismatched plant;
  after 40 s of Identify data, accepted model gives 3.448 s swing-up and stable balance.
  Held-out cart R² = 0.9885; B_f = 3.240 versus truth 3.3 N s/m. All fits stayed in a
  temporary data directory; deployed defaults/policy were preserved.
- Optional offline show and MPC balance successfully. Best ILC tracking RMS improves
  from 0.727 to 0.441 cm; a subsequent worse trial is rolled back.

Evidence/scripts: `docs/evidence/course/`, `lab-rehearsal/`, `playground/`,
`overnight-tuning.json`, `energy-search-final-validation.json`, `fluidity-after.json`,
`course-live.png`; `tools/prepare_course.py`, `rehearse_lab.py`, `search_energy.py`.
The portable bundle is refreshed from these sources with its SHA-256 manifest checked.

Next at the rig: verify calibration/timing, measure friction/motor response, validate a
second recording, establish quiet 4 V LQR balance, then attempt energy swing-up with
appropriate voltage and all limits intact. Physical robustness is not established.
No commit, push, hardware connection or policy promotion.
