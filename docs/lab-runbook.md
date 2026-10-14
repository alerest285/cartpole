# Lab runbook — simulation to the first real session

Updated 2026-10-14. No real rig session has occurred. The local simulator, fake-board path and training pipeline have automated checks; encoder signs, physical watchdog operation, loop timing and control performance on the real rig remain unverified.

## 1. Bring the bundle

Copy `dist/cartpole-lab.zip` to the lab PC and extract it. Open the inner `cartpole-lab` folder. It contains Python code, trained policies, MATLAB simulation, original course material and `MANIFEST.sha256.json`. Keep one copy per rig so calibrations and fitted parameters do not mix.

Double-click `setup_lab.cmd`. It creates `.venv`, installs numpy and runs a read-only preflight. Internet access is needed for numpy unless already cached. Python 3.9 or newer must already be installed. In a terminal, activate the environment:

```bat
.venv\Scripts\activate
python -m cartpole doctor
python -m cartpole selftest
python -m cartpole serve --backend sim
```

Or double-click `start_sim.cmd` to start an energy swing-up demo automatically when the browser opens. **Run demo** resets and starts it from any tab. To try PPO, choose it, reset hanging, then Run selected. For LQR/PID/pole placement, reset upright first. STOP or space disarms. Every armed stretch is recorded.

## 2. MATLAB rehearsal

Activate the MATLAB license, open `matlab/` as Current Folder and run `demo_cartpole`. See `matlab/README.md` for the Simulink builder and LQR course-template builder. MATLAB execution was blocked by licensing on the Mac; run its parity checks before relying on those files.

## 3. Quanser API on the Windows lab PC

The course models use Q2-USB board 0, encoder channels 0 and 1, AO 0 and digital outputs 0–3 for amplifier enable. Sample period is 0.002 s. These are template values and must be checked on the actual rig.

Install the Quanser Python API into this project's `.venv`, using the wheel supplied with the lab's SDK/QUARC installation. Inspect `%QSDK_DIR%` and `%QUARC_DIR%`; wheel locations vary by installation. For example, if the wheel is in `%QSDK_DIR%python`:

```bat
.venv\Scripts\python -m pip install --find-links "%QSDK_DIR%python" quanser-api
.venv\Scripts\python -m cartpole doctor --hardware
```

If missing, ask the TA for the SDK/Python API. `doctor` only checks the environment and policy; it never opens the board. A successful doctor is not a successful hardware test.

**One board owner:** stop every QUARC/Simulink model before Python opens the board. Stop Python before building/connecting a QUARC hardware model.

## 4. Probe with the operator at the rig

Centre the cart physically and let the pole hang motionless before starting. Opening the board zeroes its encoders at that pose. The default probe writes zero volts, enables the amplifier and reads encoders; it does not command motion.

```bat
python -m cartpole probe
```

Move the cart and pole by hand during the readout. Positive x is to the right; alpha increases counter-clockwise. Return the cart to the centre. If signs disagree, update `data/hardware.json` with the TA, then repeat the check. Defaults are cart metres/count = `-2.2749e-5` and pole radians/count = `2*pi/4096`.

Then, with hands clear:

```bat
python -m cartpole probe --move
```

This commands +1 V for 150 ms and -1 V for 150 ms. It refuses motion after an earlier failed check or if the cart is more than 2 cm from zero, and aborts past 4 cm. The board watchdog remains active throughout. A wrong motor direction writes a voltage-sign correction and returns failure; rerun the probe. Failed checks return a nonzero exit code. Resolve each failure before arming.

The backend refuses startup without a working watchdog API and aborts on reload failure/expiration. Physical output shutdown on the actual rig still needs verification. Loop timing must also be measured on the Windows PC; if software timing is poor, try `--timing hw`. Hardware-clocked task availability is not yet verified.

## 5. First real deployment

Re-centre and hang the pole before opening the console:

```bat
python -m cartpole serve --backend quanser
```

Or use `start_hardware.cmd`. The browser opens disarmed and shows LIVE HARDWARE. Keep the operator at the rig. Closing the last browser tab disarms; the heartbeat timeout is 0.5 s and the board watchdog timeout is 0.1 s. Hard cart limit is 19 cm, within the course's 20 cm limit; the model-based soft barrier is 17 cm. These layers do not establish control robustness on unknown physical friction/backlash.

See [the October 15 checklist](tomorrow.md) for the complete identification/validation path.
Hardware startup now defaults to a 4 V cap and reduced Identify amplitude/noise.

First-session order:

1. Setup: set 4 V cap, verify encoder readout at rest. Zero only while disarmed, cart centred, pole hanging.
2. Identify: start with reduced excitation amplitude/noise, observe the response, record about 30 s if motion remains controlled, then STOP.
3. Learn -> Sysid only (leave simulator runs unchecked). Review estimation and held-out fit quality and physical parameters. Runs -> Use sysid fit. Rank-deficient, nonfinite, implausible, poorly separated, cart R2 below 0.8 or held-out R2 below 0.75 fits are rejected. Validate against a second recording too.
4. State feedback: arm, lift the pole upright by hand, observe the catch and STOP. Compare PID and pole placement after the baseline is stable.
5. With the operator/TA satisfied with balance and track limits, increase the cap as needed for energy and sine swing-up from hanging. Their simulated benchmark used a 10 V cap; a 4 V failure is not evidence that they match the rig.
6. Try PPO with LQR catch first. Exploration for PPO learning should use catch OFF so its logged 50 Hz transitions remain contiguous. Start exploration only after deterministic runs behave acceptably.
7. STOP before any identification/training/model reload. Record several short exploration episodes, then Learn -> Improve. Inspect the candidate evaluation before deploying it. Promotion is based on simulated evaluation, not proof of better real performance.

## 6. Return data and refresh MATLAB

Preserve every file in `runs/`, plus `data/hardware.json`, `data/params_fitted.json`, policy versions/registry and consumed-episode history. Use a single authoritative training copy; do not merge independently trained registries by hand.

The runtime saves logical voltage commands before the motor-direction sign and snapshots the hardware calibration per run. CSV is directly readable by MATLAB; NPY/JSON and exploration NPZ remain the training source. Fake-board runs are marked simulated and excluded from hardware fitting/scoring.

```bat
python -m cartpole sysid
python -m cartpole improve --iters 300
python tools/export_matlab.py
python tools/build_lab_bundle.py
```

Without real runs, use the simulator and fake-board checks; do not label them as real data. `--include-sim` is an explicit rehearsal option. Use a scratch data directory for training experiments if you want to preserve the current policy. PPO updates accept only unconsumed contiguous episodes from the current behavior-policy version. Keep old policy versions for rollback.

## 7. MATLAB/QUARC deployment alternative

`build_course_models` now prepares eight disarmed copies: LQR, pole placement, PID,
mouse/LQR catch, fixed sine, energy, Identify and phase-locked sine. Its fixed-size MATLAB
Function controller includes angle unwrapping, Kalman estimation, PID anti-windup, limits
and Arm=0 startup. Original protected I/O/watchdog logic is retained. `lab_voltage` is
logged after the original amplifier limiter. Run `validate_course` offline before building
or connecting. Numerical/QUARC compilation on licensed MATLAB remains pending.

On the lab PC, `build_course_lqr` fills a copy of the state-feedback course template with LQR gains, Arm=0 and a 4 V cap. It preserves the course's I/O and safety subsystems and does not compile/start hardware. Its native course derivative filters differ from the main Kalman pipeline; validate it separately. The script is prepared but could not be executed or compiled on the unlicensed Mac. Only LQR balance is wired into this hardware template; MATLAB swing-up/PPO simulations and the Python hardware controllers are separate paths.
