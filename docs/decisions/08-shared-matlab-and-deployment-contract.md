# 08 — Share model/controller data; make deployment failures explicit

2026-10-14.

MATLAB and Python use one exported parameter/gain/policy contract. MATLAB implements the nonlinear equations, quantised measurements, Kalman filter, controller formulas and 500 Hz sampling, with Python reference vectors for verification. Base MATLAB is enough for native simulation; Control System Toolbox is only used for independent redesign. The Simulink adapter is normal-mode simulation, while the course QUARC LQR builder is a separate hardware route with the original derivative filters. We do not treat an interpreted simulation block as deployable QUARC code.

MATLAB startup was attempted both in the sandbox and with approved external execution. The latter reported License Manager Error -1,359 (missing license). All 20 source files pass the installed analyzer, but executable validation is deferred until activation.

The deployment audit found logical voltage commands were multiplied by the hardware direction sign again during identification. The correct contract is: controller/safety produce logical volts; the backend applies `volts_sign`; recorded volts remain logical. Every run now stores calibration so later global changes cannot reinterpret earlier encoders.

FakeQuanserBackend inherited a hardware classification despite simulated physics. This contaminated hardware-only data selection. It is now marked simulated, and legacy records containing a fake backend name are excluded by default.

The hardware watchdog previously fell back to software-only control after API errors and ignored reload return values. Hardware startup now fails if the watchdog is unavailable; expired/failed reloads abort the loop. This follows the return/expiration semantics in [Quanser's watchdog API documentation](https://docs.quanser.com/quarc/documentation/python/hardware/Functions/Watchdog/watchdog.html). [Q2-USB support documentation](https://docs.quanser.com/quarc/documentation/q2_usb.html) documents its watchdog capability; physical behavior still requires a rig check.

Evidence: 30/30 tests, 49/49 controller/plant cases, fake probe (2.000 ms mean, 2.02 ms max), and browser swing-up/STOP/log checks. Regression tests target expired watchdogs, missing watchdog support, infeasible braking, calibration signs, fake-data classification, repeated Arm and off-policy/discontinuous PPO records. These tests establish software behavior and are not a physical safety certification.
