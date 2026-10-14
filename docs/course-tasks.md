# Every handout task and its evidence

Prepared 2026-10-14 for the October 15 rig session. The handout's three completion
requirements are a derived model/simulator, physical upright balance, and physical
swing-up. Software exists for every numbered exercise and every optional playground
topic below. **Physical trials and TA sign-off are still pending.** MATLAB source
analysis passes; MATLAB/QUARC execution is blocked locally by the missing license.

| Handout task | Implementation / answer | Verification or remaining action |
|---|---|---|
| 1. Modeling: textbook coordinates and opposite angle sign | `cartpole/dynamics.py`, supplied solved-model PDF; equations below | Independent finite-difference linearization, energy conservation and PDF-number tests |
| 1. Add viscous friction and voltage-driven motor | `Plant.B_f`, `B_p`, back-EMF, optional reflected rotor mass and Coulomb term | Explicit priors until identified; `tools/rehearse_lab.py` exercises refitting |
| 1. MATLAB dynamics and Simulink, 5 V at 10 Hz | `matlab/course_modeling.m`, `build_cartpole_simulink('sine_test')`; Python `sine_test` method | Python full/ideal motor traces in `docs/evidence/course/`; MATLAB execution pending |
| 1. Run the real 5 V/10 Hz model and compare | Original `five_V_ten_Hz_oscillation.slx`; optional recording overlay in `course_modeling(recording)` | Real recording and TA review pending; initial 4 V cap must be deliberately changed for the exact 5 V exercise |
| 2. Rearrange into y = theta*x_dot, determine d | Formula below, `cartpole/friction.py`, `matlab/estimate_friction.m` | d = 1 for B_f alone; Coulomb extension has d = 2 |
| 2. Mouse excitation and recording | Original `regression.slx`, Python manual and Identify modes, `read_course_data.m` | Real recording pending; numeric, timeseries, Structure With Time and SimulationOutput supported |
| 2. Least squares B_f and check plausibility | Scalar fit plus Coulomb comparison, chronological holdout, time/direction/excitation checks | Analytic recovery/rejection tests; 40-second simulated recording; separate physical record still needed |
| 3. Two PID loops and causal derivative | `controllers/classic.py::PID`, selectable filtered derivative with adjustable cutoff, Kalman alternative | PID benchmark with a pole tap; cutoff can change only while stopped |
| 3. Tune Kp/Kd/Ki, freeze inactive integral | Live parameter schema, inactive reset, saturation anti-windup; copied PID model builder | Software regression tests; rig tuning pending |
| 3. Disturb and inspect response | Simulator tap buttons and benchmark | Physical disturbance test pending |
| 4. State, A/B/C/D, controllability/observability | `dynamics.linearize`, `lin.py`, `design_gains.m`, `course_modeling.m` | Both ranks 4; stored numeric analysis and PDF-reference test |
| 4. Choose/place/tune four poles | Defaults −5, −6, −7, −8; continuous/discrete design; copied state-feedback model | Pole-placement tests and seven-plant benchmark; physical trial pending |
| 4. LQR gains and Q/R experiment | UI controls, `tools/prepare_course.py`, MATLAB scripts | Four Q choices tested with limited voltage and disturbance; real comparison pending |
| 4. Continuous versus sampled control | Continuous and ZOH discrete designs at 2 ms | Explanation below; check actual Windows loop timing before relying on it |
| 5. Mouse swing-up and LQR handoff | `manual_swing` method, drag pad/slider/keys, copied mouse model | Handoff and released-input tests; physical attempt pending |
| 5. Fixed sinusoid, amplitude starting at 1 V, tune frequency | `sine` method's fixed/tracking/locked options; copied fixed-sine model defaults 1 V | Locked sine passes standard benchmark; fixed sine can fail when resonance changes |
| 5. Lyapunov energy derivative, pumping condition and catch | Formula below; energy controller; fixed-size `cp_lab_controller.m` in a MATLAB Function block | Standard benchmark passes; broad combined-error tests expose remaining failures |
| 6. Show and user inputs | Smooth cart reference in `playground`, live simulator/keyboard/pointer UI | Show simulation balances while following the reference |
| 6. Reinforcement learning | NumPy PPO, observation/reward/randomization, versioning and deployment improvement loop | Existing policy benchmark and full train/update tests; hardware exploration pending |
| 6. Iterative learning | Offline `playground::TrackingILC`, bounded learned feedforward around LQR, rollback on worse trial | Best simulated tracking RMS 0.441 cm versus 0.727 cm initially; not hardware-qualified |
| 6. Model predictive control | Offline 64-plan sampling MPC, 0.6 s linear horizon, LQR terminal cost and common safety filter | Simulated tap recovery; max travel 4.51 cm; not a 500 Hz hardware controller |
| 6. 10 cm CBF/QP | `matlab/course_barrier.m`, common Python filter with `x_soft=.10` in playground | Scalar clamp equals the one-input QP; uses `quadprog` when available |
| 6. Kalman state estimation | Two constant-velocity filters in Python/MATLAB and course model controller | Estimator replay and tracking tests; PPO is kept on its trained estimator contract |

## Model and friction equation

Use alpha = 0 upright, positive counter-clockwise, and pole centre of mass
`xp = x - l*sin(alpha)`, `yp = l*cos(alpha)`. With `ml = mp*l`,
`M = mc + mp + reflected_rotor_mass`, and `J = (1 + I_factor)*mp*l^2`:

```text
M*xdd - ml*cos(alpha)*alphadd + ml*sin(alpha)*alphad^2
    = F_motor - B_f*xd - F_coul*tanh(xd/v_c)
J*alphadd - ml*cos(alpha)*xdd = ml*g*sin(alpha) - B_p*alphad
F_motor = a*k_amp*V - b*xd
a = Kg*Kt/(Rm*rmp), b = Kg^2*Kt*Km/(Rm*rmp^2)
```

The handout's scalar exercise holds the other parameters fixed and neglects Coulomb
friction. Then `y = F_motor - M*xdd + ml*cos(alpha)*alphadd - ml*sin(alpha)*alphad^2`
and `y = B_f*xd`. Thus `B_f = sum(xd*y)/sum(xd^2)` and **d = 1**. Unknown mass or
motor gain can bias this scalar estimate; the joint `sysid` fit estimates those too.
The optional `F_coul*tanh(xd/.01)` term makes the regression two-dimensional.
It is a smooth Coulomb approximation, not a complete backlash/stiction model.

The supplied regression model logs `F_c = a*V - b*x_dot`, where `x_dot` is its own
filtered derivative. `read_course_data` reconstructs V from those exact two logged
signals before applying one consistent offline filter. It does not subtract back-EMF
twice. Applied voltage must come after safety gates; requested `lab_command` alone
is not sufficient for identification. The new builders log `lab_voltage` at the
original amplifier limiter and raw positions in `lab_state`.

Filtering nonlinear products is approximate. Fit scores and analytic tests do not
prove a correct physical model. A separate run, varied speed/direction, and comparison
of forward simulations against recordings remain necessary. Estimation and validation
are separated before filtering in the joint fit. This follows the separation described
in [MathWorks model validation guidance](https://www.mathworks.com/help/ident/ug/validating-models-after-estimation.html).

## Energy and sampled control answers

The handout's pole energy relative to hanging is
`E = ml*g*(cos(alpha)+1) + 0.5*J*alphad^2`. Differentiating and substituting the
pole equation gives `E_dot = ml*alphad*cos(alpha)*xdd - B_p*alphad^2`.
Without bearing loss, cart acceleration with the same sign as
`alphad*cos(alpha)` adds energy. With bearing loss, the positive term must exceed
`B_p*alphad^2`. The implementation shifts energy by the constant `2*ml*g`, making
upright energy zero. It adds a starting kick, limits acceleration/voltage, centres the
cart and hands over to LQR. These constraints and uncertain friction invalidate any
claim of guaranteed swing-up from the energy argument alone.

A digital loop holds each command for 2 ms. Continuous LQR is a reasonable starting
point when sampling/latency are fast relative to closed-loop dynamics; sampling,
velocity filtering, USB delay and saturation can invalidate that approximation.
The implemented discrete design uses `Ad = exp(A*h)` and the exact ZOH `Bd`, then
solves the discrete Riccati equation. Increasing Qx prioritizes centring; Qa prioritizes
angle error; velocity weights penalize rapid motion; increasing R reduces effort.
The saved Q experiment quantifies these choices rather than assuming bigger gains win.

For the track constraint, position has relative degree two. A first-derivative CBF
cannot constrain voltage for a double integrator. `course_barrier` uses the second-order
conditions `hdd + 2*w*hd + w^2*h >= 0` for both track ends. Feasible inputs and an
initial state satisfying the accompanying velocity constraints are required. The
10 cm classroom exercise is separate from the 17 cm operating barrier used for swing-up.

## One-command preparations

```text
python tools/prepare_course.py
python tools/rehearse_lab.py
python -m cartpole playground
python -m cartpole tune --output data/tuning_report.json
```

On a licensed MATLAB installation: `validate_course`, `course_modeling`,
`build_cartpole_simulink('sine_test')`, then `build_course_models` on the QUARC PC.
The latter prepares eight copies with Arm=0 and 4 V caps. It does not compile or run
them. Original protected I/O/watchdog logic is retained; only telemetry logging branches
are added inside those subsystems. The new controller unwraps angle before Kalman
velocity estimation instead of differentiating the wrapped angle at ±pi.

`build_course_lqr` remains as the earlier derivative-filter alternative; use the new
builder for the full course set. MATLAB Function code is assigned through the documented
[Stateflow.EMChart API](https://www.mathworks.com/help/simulink/slref/stateflow.emchart.html).
