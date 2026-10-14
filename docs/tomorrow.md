# October 15: get the physical baseline working

Use **LQR balance, then energy swing-up** as the first-session path. Keep PID/pole
placement for the comparison exercises and PPO/ILC/MPC for after reliable deterministic
control. The optional ILC/MPC implementations are offline studies, not rig controllers.

## Before arriving

Bring `dist/cartpole-lab.zip`. It contains the course originals, Python stack,
MATLAB sources/contract, launchers, task map, tests and evidence. Activate MATLAB if
possible: the Mac installation currently cannot execute because its license is missing.
On the lab PC, check which route is available before opening any board connection:

```bat
setup_lab.cmd
.venv\Scripts\activate
python -m cartpole doctor --hardware
```

The Python route requires the Quanser Python API supplied with the lab installation;
`setup_lab.cmd` installs numpy, not the proprietary Quanser driver. The MATLAB route
requires licensed MATLAB/Simulink and QUARC. Stop one route before starting the other.
Do not change control software, estimator and policy simultaneously during commissioning.

## 1. Calibration and timing

With the cart centred and pole hanging, follow the sign/timing/watchdog probe in
[the lab runbook](lab-runbook.md). Verify that positive logical voltage moves toward
positive x and that alpha is zero upright, ±pi hanging. Zero only in the stated hanging
pose. This has to be checked on the actual rig; simulated probes do not establish it.

```bat
python -m cartpole probe
python -m cartpole probe --move
```

The second command moves the cart with small test pulses. The operator must be at the
rig. Resolve probe failures before running controllers. Check the Windows loop timing;
the design assumes 2 ms sampling and bounded latency.

## 2. Measure friction and motor response

Start the console with `start_hardware.cmd`. It opens disarmed with a **4 V cap** and
reduced Identify settings (1.5 V chirp amplitude, 0.5 V random steps). Record about
30–40 seconds of varied motion in both directions, only while its motion remains
controlled. STOP, then run **Learn → Sysid only**, with simulator runs unchecked.

The fit estimates cart mass, amplifier gain, viscous and Coulomb friction, pole inertia,
bearing friction and delay. Inspect held-out fit quality and physical plausibility.
Rejected fits do not replace the previous model. Record another excitation segment
at different speeds if parameters cannot be separated. Confirm the accepted model on
a **second recording**, not only its own estimation trace.

For the handout's scalar B_f exercise, or MATLAB recordings:

```matlab
% After regression.slx wrote workspace signals:
report = estimate_friction;
d = read_course_data;
writetable(d,'rig_identify.csv');

% For the new generated course models, use their output object if applicable:
% d = read_course_data(out);  % contains lab_state and lab_voltage
```

```bat
python -m cartpole friction rig_identify.csv
python -m cartpole import-run rig_identify.csv --hardware
python -m cartpole sysid
python tools/export_matlab.py
```

`--hardware` is only for an actual rig recording. Imports default to simulated. The
scalar fit assumes known mass/motor gain; the joint fit can reveal when those assumptions
are wrong. `lab_command` before a safety gate is not applied voltage and cannot substitute
for `lab_voltage`.

## 3. Verify balance at reduced voltage

While stopped, select **Runs → Use sysid fit**, then **State feedback / LQR**. Keep the
initial 4 V cap. Arm with the operator at the rig, bring the pole upright by hand and
observe the catch. First obtain a quiet centre hold, then test a small disturbance and
save the recording. Check clipping, oscillation, drift and timing before changing gains.
Use the PDF's PID/pole-placement/Q experiments after this baseline is dependable.

In MATLAB, run the offline checks first:

```matlab
validate_course
course_modeling
build_course_models
```

The builder creates eight models under `matlab/generated/`, each Arm=0 and capped at
4 V. Build/connect the chosen model with the TA. Their compiled QUARC behavior has **not**
been verified here. The earlier `build_course_lqr` builder uses the course derivative
filters; the new full-course builder uses an unwrapped-angle Kalman observer.

## 4. Swing up, then compare

Use the accepted model and energy controller first. The standard swing-up benchmark
used **10 V**, so a 4 V failure is not a valid test of that result. Increase the available
voltage only after balance, signs and track checks are satisfactory with the operator/TA.
Keep all position, voltage and watchdog limits in place. Save every attempt.

If swing-up stalls, compare its recorded angle/energy/voltage against the model before
raising gains. The broad uncertainty study still has failures; its candidate settings
were **not installed**. After an actual fit, rerun `python -m cartpole bench` and optionally
`python -m cartpole tune --output data/tuning_report.json`. Tuning reports are proposals,
not automatically applied gains.

## What tonight's evidence establishes

- Existing 4 V LQR passed 13/13 combined-uncertainty balance cases.
- Standard controllers passed 49/49 benchmark cases, including energy, sine and PPO.
- A deliberately mismatched simulated rig failed swing-up before identification. After a
  40 s excitation recording and accepted fit, it swung up in 3.448 s and balanced. Held-out
  cart regression R² was 0.9885; fitted B_f was 3.240 versus simulated truth 3.3 N s/m.
- The broad search's best candidate passed 18/25 fresh combined-error cases. That is
  insufficient for replacing the default or claiming general hardware robustness.
- No real-rig validation has occurred. Friction/stiction, encoder zero, backlash, amplifier
  direction and Windows/USB timing must still be measured tomorrow.

Keep `runs/`, `data/hardware.json`, `data/params_fitted.json`, policy versions and the
MATLAB workspace/CSV recordings together. They are the evidence needed to tune the next
attempt. See [the complete task map and derivations](course-tasks.md).
