# Status 2026-10-14 — integrated dashboard

The local application's default view is now a complete lab dashboard. The control room
retains all previous controller parameters, plots, training, runs and setup. Both views
share the live simulator/controller connection, STOP button and heartbeat.

Available in the dashboard:

- Live plant and separate simulation/hardware commissioning notes.
- Friction regression from saved runs or uploaded CSV; viscous/Coulomb comparison,
  held-out scores, provenance and downloadable results. No automatic parameter install.
- CSV import into recorded runs, simulated by default, with explicit physical-data marking.
- Active model, joint-identification/application links and current MATLAB export.
- Six course sections, commands, rig tasks, both PDFs and procedures in a document viewer.
- Standard benchmark, broader failed uncertainty cases, identification rehearsal and
  optional playground evidence, all clearly labeled as simulation.
- Background benchmark, uncertainty sweep, identification rehearsal, modeling/Q experiments,
  playground and MATLAB export, with job status, output and cancellation.
- Portable ZIP download when built with `python tools/build_lab_bundle.py`.

Recording analysis/import require a stopped controller. Background jobs prevent arming or
demo startup until complete/cancelled. New resources are allowlisted; no general filesystem
or arbitrary command endpoint was added. Physical control settings and policy are unchanged.

Verification: 45 Python tests, 49 standard benchmark cases and six presentation tests.
The dashboard browser check covers uploads, analysis, imports, real subprocess execution,
documents, selection staying disarmed, persistence and responsive widths. The previous
fluidity/STOP/manual/estimator tests remain in `tools/check_fluidity.py`.
See `docs/evidence/dashboard-browser.json`, `dashboard-verification.json` and screenshots.

MATLAB sources remain statically checked only; local license activation and lab QUARC
compilation/execution are pending. No real-rig commissioning or physical robustness claim.

GitHub destination: private `https://github.com/alerest285/cartpole`, requested by Ale.
Local recordings/imports and transient logs are excluded. This snapshot is included in the
initial dashboard commit; Git history and the remote identify the published revision.
