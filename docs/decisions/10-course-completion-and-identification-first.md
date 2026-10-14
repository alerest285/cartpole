# 10 — Complete the exercises and make identification the first rig step

**Date:** 2026-10-14 · **Status:** accepted

The standard benchmark perturbs one plant feature at a time. It passed 49/49 cases,
but this did not answer whether a controller tolerates simultaneous friction, gain,
mass, inertia, latency, encoder-zero and deadband errors. A broader study found that
the existing 4 V LQR passed 13/13 unseen balance cases while default energy swing-up
passed only 5/13. An adaptive candidate reached 9/13. A 131-candidate search selected
another setting which passed 18/25 fresh cases. Neither was installed.

We keep the known baseline, record the failures, and prioritize measurement. A synthetic
40-second first-session recording demonstrates the complete path: an unknown simulated
rig failed swing-up with the nominal model, then swung up in 3.448 s and balanced after
identification. Cart held-out R² was 0.9885; B_f was 3.240 versus truth 3.3 N s/m.
This is evidence that the pipeline functions, not evidence that the physical rig matches
those parameters. Even a matched-model stress study still had failures with additional
deadband and sensor errors. Raw Coulomb smoothing does not model all static friction.

The handout scalar fit holds mass/motor constants fixed, estimates B_f and compares a
Coulomb extension using guarded chronological holdout. The generalized fit now selects
delay on estimation data and rejects poor held-out prediction or poor parameter
separation. It no longer selects and reports performance only on the same samples.
MATLAB's original F_c/x_dot signals are converted back to commanded voltage before
consistent offline filtering. Normalized imports default to simulated, require explicit
`--hardware` for real recordings, preserve calibration/source hashes, and cannot become
PPO exploration episodes.

Hardware runtime startup now defaults to 4 V and reduced Identify excitation. PID has
feedback from the shared output limiter for conditional integration. Derivative cutoff
is adjustable only while stopped, and PPO retains its trained Kalman contract. No
Kalman defaults or policy weights changed. New registry entries are appended so old
numeric mode IDs are preserved.

Every handout exercise now has code and an explicit status in `docs/course-tasks.md`.
The MATLAB builder copies the original balance or swing-up plant as appropriate,
preserves its I/O/watchdog and balance-angle gate, adds telemetry branches, and uses a
fixed-size controller with an unwrapped-angle Kalman observer. It prepares Arm=0 models
and never compiles or runs them. All 27 MATLAB sources pass the installed analyzer;
numerical execution/code generation remain pending on licensed MATLAB/QUARC.

ILC and sampling MPC remain offline coursework demonstrations. ILC reduced its best
tracking RMS from 0.727 to 0.441 cm and rolls back a worsening trial; MPC recovered a
tap with max cart travel 4.51 cm. Neither has a tested hardware timing/control contract.

Validation: 41 Python tests, 6 presentation tests, standard 49-case benchmark, analytic
friction/rejection tests, scalar-versus-batched tuning parity, full simulated lab
rehearsal, and actual browser tests of Course/PDF, cutoff tuning, manual input timeout,
swing-up, STOP/reload, resize/theme and disconnect behavior. MATLAB/physical checks
are deliberately separate from these results.
