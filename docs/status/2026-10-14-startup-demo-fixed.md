# Status 2026-10-14 — visible demo on simulator startup

Supersedes the startup behavior in `2026-10-14-sim-to-real-ready.md`; MATLAB licensing and real-rig limits from that status still apply.

The earlier launcher opened a disarmed Idle screen, so the user saw no movement. Both simulator launchers now pass `--demo`. The first browser connection starts energy swing-up automatically. A prominent Run demo / Restart demo button is visible from every tab and restores the demo state/settings. It switches the visible panel to Control. A stopped simulator explains how to start it. STOP persists across page reload; reconnection does not re-arm a stopped run.

The demo is restricted to SimBackend, and `--demo --backend quanser` is rejected before hardware construction. Existing physical controller behavior is unchanged.

Verified in an actual browser: a fresh server plus first page visit swings up without configuring controls, then stays balanced; Restart demo works from Setup; STOP remains stopped after reload; no JavaScript errors. Evidence: `docs/evidence/demo-check.json`, `demo-startup.png`, `demo-live.png`. The live server was also observed running Energy with the pole within 0.001 degrees of upright.

Final checks: **31/31 Python tests** (64.339 s), **49/49 benchmark cases**. Lab ZIP rebuilt with the new launchers and UI. No commit or push.
