# 11 — Integrate the lab workflow into the local control application

The course implementations, fitting tools and evidence were spread across commands and
documents. A Course tab alone did not provide the requested working dashboard.

The new dashboard shares the existing WebSocket and live scene with the control room.
It combines commissioning notes, recording import, friction regression, current model,
six course sections, validation reports, MATLAB handoff, offline jobs and downloadable
materials. Switching views reparents the same canvas; it does not start another controller.
The STOP control and heartbeat remain in the shared header/application.

Friction analysis uses applied voltage and the active model's constants. It saves a report
without installing parameters. Uploaded CSV files default to simulated; only an explicit
physical-recording declaration enables real-run import. Raw imports and metadata are kept
locally. Estimation and import require a stopped controller and no running job. HTTP uploads
are bounded, JSON-only and same-origin for browsers. Materials/reports use fixed allowlists.

Offline commands are selected from a fixed task map, launched in a subprocess with NumPy
single-threaded, and share the existing one-job slot. Arming/demo are blocked until a job
finishes or is cancelled. Offline completion does not reload the plant; accepted identification
and policy workflows retain their existing behavior. No controller gains or safety thresholds
changed. Operator checklist notes are stored separately for simulation and hardware in browser
storage, with explicit wording that checks do not verify the rig.

Measured browser acceptance covers live balance, all six sections, both PDFs/bundle, friction
from uploaded and saved recordings, simulated import default, no auto-applied model, a real
offline modeling subprocess, controller selection staying disarmed, invalid CSV rejection,
STOP/notes across reload and widths 390/800/1440. No page errors were observed.

The user explicitly requested a GitHub push. There was no remote or previous commit, and no
cart-pole repository in the authenticated account. Use a new private `alerest285/cartpole`
repository, keeping local runs, imports, transient job logs and build outputs ignored.
