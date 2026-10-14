# Status 2026-10-01 — built and verified in simulation; rig not yet touched

**TL;DR.** Seven methods balance and swing up the simulated cart-pole on 7 mismatched plants; the
browser console drives them live; the Quanser path is complete and tested against a fake board;
PPO learns the swing-up from scratch in 8 minutes; the Improve loop took a policy from 1/5 to 5/5
on a rig it had never seen, from one session's worth of data. Nothing has run on the real rig yet.

## Shipped
- `python -m cartpole serve|probe|bench|train|improve|sysid|selftest`; console on 127.0.0.1:8000.
- Bench (`data/bench.json`), 7 plants each: PID 7/7, pole placement 7/7, LQR 7/7 (tilted + tap),
  energy 7/7 (2.55 s), locked sine 7/7 (2.74 s), PPO v002 7/7 (1.85 s), PPO + LQR catch 7/7.
- Motor noise while balancing ≈ 0.3 V RMS (Kalman q_x 2, q_a 0.3; decision 02).
- Policies: v001 (scratch, 1500 iters, 19.2 M steps), v002 (v001 fine-tuned on the final
  estimator) = current. Both 14/14 in evaluation.
- Deployment loop evidence (`tools/deployment_loop_demo.py`, `docs/evidence/deployment_loop.json`):
  rig B_f 5.5, k_amp 0.8, B_p 0.005, F_coul 0.4, delay 2 ms, m_c 0.63 (outside the training
  randomisation): v002 1/5 → v003 5/5 (3.31 s) → v004 5/5 (3.18 s); sysid recovered every
  parameter within 4 %. These v003/v004 lived in a scratch copy; the real registry is v001, v002.
- Tests 23/23 (`selftest`), including fake-board signs, watchdog, dead-man, sysid recovery,
  WebSocket protocol, improve end to end.
- Docs: lab-runbook, project, decisions 01–07, worklog. Bundle: `dist/cartpole-lab.zip` (335 KB).
- Course side: model PDF + .m files in the Desktop course folder; `matlab/design_gains.m`,
  `matlab/lyapunov.m` for the Simulink templates.

## Not verified (needs the lab PC)
1. Where `quanser_api` lives there and whether a user install is allowed (runbook §3).
2. Real encoder signs and the +V direction (`probe --move` writes corrections).
3. Windows loop timing (fake board on the Mac: 2.000 ms mean, 2.04 ms max).
4. Real friction and backlash: sysid R² on real data.
5. MATLAB scripts unrun (MATLAB on the Mac awaits Ale's ETH sign-in).

## Known limits
- Hand-designed swing-ups fail when pole friction is ≥ 2× the prior (decision 03); PPO tolerates
  more, and Improve refits B_p.
- In fast-forward simulation the control thread starves other Python threads (episode lengths
  overrun); real time and hardware are unaffected (the pacer sleeps).

## Next, ranked
1. Thursday 08.10, 13–15 ETL D10.1: runbook §1–5, first-session order (Identify → Sysid → state
   feedback at 4 V → swing-ups → PPO with catch → explore episodes → Improve).
2. Copy `runs/` back to the Mac after the session; Improve there with more iterations.
3. Commit the repo (only on Ale's word); decide whether it goes on GitHub for the lab PC.
