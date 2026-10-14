# Status 2026-10-14 — simulator running; lab pipeline prepared

No real rig session has occurred (confirmed by Ale today). MATLAB execution remains blocked by licensing.

## Verified on this Mac

- `python -m cartpole selftest`: **30/30 passed**, including the full exploration -> identification -> PPO update -> candidate registration workflow in isolated scratch directories. Final run: 89.152 seconds.
- `python -m cartpole bench`: **49/49** cases passed (seven controller configurations on seven plants). Energy mean upright time 2.55 s, sine 2.74 s, PPO 1.85 s. Current policy remains v002.
- `python -m cartpole probe --fake --move`: passed; positive pulse displacement 14.2 mm, software period mean 2.000 ms/max 2.02 ms in that check. These are simulated-board results, not physical measurements.
- Actual headless browser: energy swing-up, PPO swing-up, STOP at zero volts, saved-run display, no JavaScript errors. Evidence: `docs/evidence/browser-check.json`, `browser-energy.png`, `browser-ppo.png`, `browser-runs.png`.
- MATLAB R2024b's local `mlint -id matlab/*.m`: all 20 files, no diagnostics. Batch MATLAB startup fails with License Manager Error -1,359: license file cannot be found. Static analysis is not numerical validation.
- `doctor`: current policy loads; Quanser API is not installed on the Mac. It never opens hardware.

## Added

- Native MATLAB nonlinear simulation, encoder quantisation, Kalman filter, six controllers including exported PPO inference, animation, CSV analysis, and Python reference-vector checks. `matlab/demo_cartpole.m` is the entry point.
- `build_cartpole_simulink`: normal-mode discrete simulation model builder. `validate_simulink`: native-loop comparison. Builder/runtime execution pending MATLAB activation.
- `build_course_lqr`: fills a copy of the course's state-feedback QUARC template, with Arm=0, 4 V cap and logging. Protected I/O/safety subsystems stay intact. LQR hardware only; derivative-filter behavior must be separately validated. It never compiles or starts the board.
- `tools/export_matlab.py`: shared model, gains, calibration, policy and reference-vector export. No proprietary MATLAB engine needed to export.
- Windows setup and simulation/hardware launchers, Mac simulator launcher, read-only `doctor`, manifest-checked lab ZIP builder and original course material.

## Deployment fixes

- Watchdog is mandatory for hardware startup; failed or expired reloads abort. Probe retains watchdog protection during motion and returns failures accurately.
- Fake-board data is marked simulated and legacy fake runs are excluded from real-data fitting/scoring.
- Each run snapshots encoder calibration; sysid uses that snapshot and the logged logical command without applying volts_sign a second time.
- Infeasible barrier braking cannot exceed the voltage cap. Invalid safety limits are rejected; real-hardware barrier cannot be disabled through settings.
- Crashed loops disarm and close the board. Repeated Arm does not overwrite a run. Shutdown joins background log writers; new runs include CSV alongside NPY/JSON.
- PPO real-data updates filter for current-policy contiguous episodes. Poor/rank-deficient fits are rejected, and rejected fits do not narrow domain randomisation.

## Remaining real-world work

1. Activate MATLAB using Ale's ETH/MathWorks license; run `demo_cartpole`, `validate_simulink`, and toolbox gain design. MATLAB numerical parity and Simulink build/compile have NOT been verified.
2. On the lab PC install the matching Quanser API, run the probe, verify signs, actual watchdog output shutdown and 500 Hz timing.
3. Collect controlled Identify runs, review the fit, catch upright at low voltage, then attempt swing-up/PPO with the operator at the rig. Simulation success does not establish real-rig success.
4. Copy real runs back, train in the authoritative policy directory, re-export MATLAB contract and rebuild the lab bundle.

No Git commit or remote push was made. Policies v001/v002 and the earlier Desktop model files are preserved.
