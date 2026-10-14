# cartpole — contract for any Claude session

The IfA cart-pole (P&S Mission Impossible: CartPole 4.0) driven from a browser: classic
controllers, swing-ups, a PPO policy that improves from rig sessions. Start from
`docs/status/` (newest = truth), then `docs/lab-runbook.md` if you are on the lab PC.

1. **Docs as engineering** (`~/.claude/reference/docs-pattern.md`): `docs/worklog.md`
   append-only `HH:MM — what — why — result` while working; a dated `docs/status/` snapshot at
   the end of a session; a numbered `docs/decisions/NN-slug.md` for every non-obvious choice,
   with the measured numbers that decided it.
2. **Hardware is live.** Never arm `--backend quanser` without Ale at the rig. Never remove or
   weaken a safety layer (`safety.py`, dead-man, board watchdog, `x_hard < x_watchdog`) to make
   a controller pass. A controller that needs it is wrong.
3. **One owner of the board.** QUARC/Simulink models and this program cannot run at once.
4. **Sign conventions are fixed** by the course's Simulink files: alpha = 0 upright,
   counter-clockwise positive, hanging = ±π; x = −counts·2.2749e-5; +V → +x. If the rig
   disagrees, change `data/hardware.json` (via `probe`), never the model.
5. **Every claim of "works" names the test.** `python -m cartpole selftest` (45 tests) and
   `python -m cartpole bench` must pass before a change is done. Swing-up changes are judged
   on the 7-plant set in `rl/train.py::eval_plants`, not on the nominal plant alone.
6. **Training changes the estimator contract.** The RL env copies the Kalman steady-state
   gains from `Estimator()` defaults; change those and retrain or fine-tune the policy.
7. **Runs are data, policies are versions.** Never delete `runs/` or rewrite
   `data/policies/registry.json` by hand; promotion goes through `train.save_version`.
8. Long jobs (train, improve, searches) in the background with a timeout; numpy single-
   threaded per process (`OPENBLAS_NUM_THREADS=1`); one heavy job at a time on the Mac.
9. Commit only when Ale says so.
