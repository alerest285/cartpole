# worklog

Append-only, newest at bottom. `HH:MM — what — why — result`.

## 2026-10-01 — session 1 (built from the Mac; reconstructed at session end from the chat, times approximate)

- ~17:30 — read the course folder (handout PDF, system_setup.m, 7 .slx files unzipped) — learn the rig's interface — Q2-USB board 0, enc 0 cart / enc 1 pole, AO 0, DO 0–3 amp enable, 500 Hz, x = −counts·2.2749e-5, α = mod(counts·2π/4096, 2π) − π, watchdog ±0.2 m.
- ~17:45 — params, dynamics (batched), lin (expm, c2d, dare by doubling, dlqr, care, lqr, place, ctrb, obsv) — numpy only for the lab PC — eigenvalues match the PDF to 4 digits; LQR K has a negative x gain, so the PID position loop needs negative gains.
- ~18:00 — estimator (lab filter), controllers (idle, manual, pid, statefb, energy, sine), safety (cap, CBF, hard faults), headless closed loop — balancers 7/7 with taps first try; both swing-ups failed.
- ~18:15–19:30 — swing-up diagnosis — energy plateaued (spring drains the pole's energy; resonance with the pole); with true states it worked → culprit the 60 rad/s filter lag × back-EMF in model inversion; Kalman default; energy-neutral centring ε/(w²+ε); random searches over 7 plants (decision 03). Multiprocessing.Pool hung twice under load (killed at the 30/60 min limits); ProcessPoolExecutor works.
- ~19:30 — sine swing-up: fixed 0/160, frequency-tracking 0/160, phase-locked 6/7 (decision 04). Lead was clamped to ±90° by its own Param range; widened.
- ~19:40 — Quanser backend + FakeHIL, runtime (500 Hz thread, dead-man, run logs), PPO in numpy (policy, vectorised env, ppo), learned controller — first training killed at the 1 h limit (machine loaded: Bambu Studio, photolibraryd), no checkpoint → checkpoints every 100 iters.
- ~19:55 — sysid first test: pole exact, cart −15 % on every parameter → V smoothed like the outputs → within 1–9 %.
- ~20:15 — CLI, probe (PASS on the fake board incl. sign check and 2.04 ms max period), bench, server (stdlib WebSocket), web UI.
- ~22:55 — v001 training (1500 iters, 8 min) — 99.5 % upright in training.
- ~23:00 — UI end-to-end over WebSocket: energy swing-up in real time, 2.000 ms period, 0.05 ms compute. Screenshot showed ±7 V chatter while balancing → Kalman q too high; grid → q_x 2, q_a 0.3: 0.30 V RMS, all 7/7 (decision 02).
- ~23:10 — v001 eval through the deployment path: 14/14; v002 = v001 fine-tuned on the new estimator: 14/14, bench 1.85 s.
- ~23:15 — MATLAB design_gains.m + lyapunov.m — MATLAB not activated yet, unrun.
- ~23:20 — tests 22/23: improve end-to-end refused to save sysid (mass 0.18 kg) — SG kernel mismatch on 20 ms voltage steps, then half-sample alignment (decision 06) → 23/23.
- ~23:25 — UI: [hidden] lost to display:flex (tabs, fault, sim tools); pole ran off the canvas; mobile "overflow" was headless Chrome's 500 px minimum width, verified at a real 375 px in an iframe.
- ~23:30 — deployment-loop evidence on a rig outside the training randomisation (B_f 5.5, k_amp 0.8, B_p 0.005, F_coul 0.4, delay 2 ms, m_c 0.63): round 0 v002 balanced 1/5.
- ~23:35 — fast-forward demo inflated episodes (thread starvation: 279k "real" transitions); reran with real-time recording: v002 1/5 → v003 5/5 → v004 5/5, sysid within 4 % — docs/evidence/deployment_loop.json.
- ~23:40 — serve crashed on a taken port AFTER opening the board (amp enabled, no close) → bind first, close hardware on every path. Port 8766 belonged to the art_sharpa_glove server, left alone.
- ~23:45 — selftest 23/23, status snapshot 2026-10-01-built-in-sim, bundle rebuilt. Not committed (Ale's word needed).

## 2026-10-14 — simulation and first-session pipeline

- 22:44 — completed MATLAB native simulation, shared export, reference checks, Simulink and course LQR builders — reproduce the controller contract and prepare the lab handoff — 20 files pass MATLAB analyzer; execution blocked by missing license.
- 22:44 — fixed watchdog, voltage/sign, fake-data, logging, fitting and PPO episode handling — address concrete first-deployment failures — 30/30 tests, 49/49 benchmark cases, fake probe and browser checks pass.
- 22:44 — prepared launchers, course material, runbook and manifest ZIP builder — portable lab setup — real rig and MATLAB/QUARC execution remain pending.

- 22:49 — corrected simulator startup after user saw a motionless Idle screen — simulator launchers now use --demo, first browser connection starts energy swing-up, and a visible Run demo button resets/starts from any tab. Hardware rejects demo. Browser verification covers automatic swing-up/balance, restart from Setup, and STOP persisting across reload.
- 22:49 — verified first-launch demo and live server — 31/31 tests, 49/49 benchmark cases, actual browser startup/restart/STOP checks passed; lab bundle refreshed.

## 2026-10-14 — fluid display

- 22:53 — read the course handout and solved model PDF; measured the existing browser during swing-up — source of visible judder identified: 4.67 pose updates/s, although telemetry is sampled at 100 Hz and physics runs at 500 Hz.
- 22:55 — moved presentation to timestamped samples with a 70 ms interpolation buffer, continuous plot time, cached canvas sizing/theme, and a larger scene — smooth rendering without changing the controller, measured voltage, or run data; browser and regression checks next.
- 22:58 — six presentation tests, 31/31 Python tests (99.080 s), and isolated browser checks passed — measured 58 pose/plot updates/s and 0.20 ms p95 drawing callback; verified swing-up/balance, reset jumps, STOP/reload, mobile resizing, theme changes and freeze on lost telemetry. Full controller benchmark running next.
- 22:59 — full benchmark passed 49/49; refreshed the one open local simulator tab and verified the live server stays in Energy balancing with the new timeline loaded — display changes are active in Ale's browser. Updated README, project/status and decision 09; rebuilding the portable lab bundle with the same frontend and verification tools.

## 2026-10-14 — complete course tasks and prepare tomorrow's tuning

- 23:09 — audited every handout step and unpacked original Simulink wiring — identified missing scalar friction regression, MATLAB recording import, manual LQR catch, broader course builders and optional playground implementations; physical tasks remain explicitly pending.
- 23:13 — implemented the handout's one-parameter B_f fit and a separately scored Coulomb extension with chronological holdout, excitation/units/time checks and block estimates — four analytic regression/rejection tests pass; estimates do not automatically replace the model.
- 23:16 — combined uncertainty testing qualified existing LQR at 4 V (13/13 unseen cases), but exposed energy swing-up failures (5/13 baseline, 9/13 with an adaptive setting); kept defaults unchanged. A second candidate failed a fresh set (7/13), so no robustness claim or promotion was made.
- 23:20 — added MATLAB regression import/fit, normalized CSV import, joint-fit validation, manual swing-up/LQR handoff and PID saturation feedback; seven targeted tests pass, including fast-search/scalar-deployment parity. Started a bounded 128-candidate vectorized energy search with separate final validation.
- Correction to the 23:13 entry: the scalar friction implementation/test was completed around 23:11; that timestamp was entered ahead of the wall clock.
- 23:34 — completed the course task map, eight MATLAB/QUARC model builders, modeling/friction/barrier exercises and optional offline show/ILC/MPC — all 27 MATLAB sources pass static analysis; MATLAB execution/QUARC compilation remain blocked by the local license, and physical exercises remain pending.
- 23:34 — finished the identification-first rehearsal — a 40 s simulated recording produced held-out cart R² 0.9885; swing-up changed from failure with nominal parameters to stable balance after 3.448 s with the fitted model. Fits were confined to a temporary directory. Broad energy search reached only 18/25 fresh cases, so no candidate was promoted.
- 23:34 — final checks passed: 41/41 Python tests, 49/49 standard benchmark cases, 6/6 display tests and browser workflow checks at about 58 updates/s — verified the refreshed live simulator is balancing with six Course steps and no browser errors; included both project PDFs and prepared the portable bundle with the first-session checklist.

## 2026-10-14 — integrated dashboard and GitHub handoff

- 23:48 — integrated live simulation, commissioning notes, CSV import, friction analysis, active model, course tasks, evidence, MATLAB export and background jobs into the local dashboard — browser acceptance passes with no JavaScript errors at desktop/mobile widths, including actual CSV import/fit and a modeling subprocess. Four new API tests pass; full regression checks are in progress. Ale explicitly authorized committing/pushing to GitHub; authenticated account has no existing cart-pole repository, so the destination will be private alerest285/cartpole.
- 23:51 — full regression passed: 45/45 Python tests in 62.646 s, 49/49 controller benchmark cases, 6/6 presentation tests; browser fluidity is 58.67 updates/s with 0.20 ms p95 drawing time and no page errors. Updated the browser test navigation for the new default dashboard; STOP/reload, manual timeout, estimator changes and disconnect freeze all pass. Created the private GitHub repository and refreshed the local simulator with the new dashboard; preparing the initial verified commit and portable bundle.
