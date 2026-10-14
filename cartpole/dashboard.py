"""Allowlisted dashboard resources and recording analysis; never arms hardware."""
from __future__ import annotations

import json
import re
import tempfile
import uuid
from dataclasses import asdict
from pathlib import Path

import numpy as np

from . import friction, import_run
from .params import DATA, ROOT, RUNS
from .runtime import COLS

MATERIALS = {
    "handout": ("Course handout", ROOT / "course_original/PS_Mission_Impossible_CartPole4_0.pdf"),
    "model": ("Solved model", ROOT / "docs/course/cartpole_model_solved.pdf"),
    "tomorrow": ("First session checklist", ROOT / "docs/tomorrow.md"),
    "tasks": ("Every course task and derivation", ROOT / "docs/course-tasks.md"),
    "runbook": ("Hardware runbook", ROOT / "docs/lab-runbook.md"),
    "matlab": ("MATLAB and QUARC guide", ROOT / "matlab/README.md"),
    "bundle": ("Portable lab bundle", ROOT / "dist/cartpole-lab.zip"),
}
REPORTS = {
    "bench": DATA / "bench.json",
    "rehearsal": ROOT / "docs/evidence/lab-rehearsal/report.json",
    "search": ROOT / "docs/evidence/energy-search-final-validation.json",
    "playground": ROOT / "docs/evidence/playground/report.json",
    "modeling": ROOT / "docs/evidence/course/analysis.json",
    "verification": ROOT / "docs/evidence/dashboard-verification.json",
    "tuning": DATA / "tuning_report.json",
    "friction": DATA / "friction_latest.json",
}


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def snapshot():
    return {"reports": {key: read_json(path) for key, path in REPORTS.items()},
            "materials": [{"key": key, "title": title, "available": path.is_file(),
                           "format": path.suffix[1:], "url": "/materials/" + key}
                          for key, (title, path) in MATERIALS.items()],
            "course": read_json(ROOT / "web/course.json")}


def recording_path(rid, suffix):
    if not isinstance(rid, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}", rid):
        raise ValueError("Invalid recording ID")
    path = (RUNS / (rid + suffix)).resolve()
    if path.parent != RUNS.resolve() or not path.is_file():
        raise ValueError("Recording not found")
    return path


def csv_text(payload):
    value = payload.get("csv")
    if not isinstance(value, str) or not value.strip():
        raise ValueError("Choose a CSV with t, x, alpha and applied v columns")
    return value.removeprefix("\ufeff")


def analyze(payload, plant):
    delay = payload.get("delay_steps", 0)
    if not isinstance(delay, int) or isinstance(delay, bool) or not 0 <= delay <= 5:
        raise ValueError("Delay must be an integer from 0 to 5 samples")
    if payload.get("run_id"):
        rid = payload["run_id"]
        meta = read_json(recording_path(rid, ".json")) or {}
        arr = np.load(recording_path(rid, ".npy"), allow_pickle=False)
        if arr.ndim != 2 or arr.shape[1] != len(COLS):
            raise ValueError("Recording has an unsupported column layout")
        data = {key: arr[:, COLS.index(key)] for key in ("t", "x", "alpha", "v")}
        source = {"name": rid, "sim": meta.get("sim", True), "kind": "saved recording"}
    else:
        with tempfile.TemporaryDirectory(prefix="cartpole-friction-") as folder:
            path = Path(folder) / "recording.csv"
            path.write_text(csv_text(payload), encoding="utf-8")
            data = friction.load_csv(path)
        source = {"name": str(payload.get("filename", "uploaded CSV"))[:160],
                  "sim": payload.get("hardware") is not True, "kind": "uploaded CSV"}
    result = friction.estimate(data, plant, delay)
    result.update(source=source, assumed_plant=asdict(plant))
    DATA.mkdir(parents=True, exist_ok=True)
    (DATA / "friction_latest.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def import_recording(payload):
    folder = DATA / "imports"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (uuid.uuid4().hex + ".csv")
    path.write_text(csv_text(payload), encoding="utf-8")
    try:
        meta = import_run.import_csv(path, hardware=payload.get("hardware") is True)
    except Exception:
        path.unlink(missing_ok=True)
        raise
    meta["uploaded_name"] = str(payload.get("filename", "uploaded CSV"))[:160]
    (RUNS / (meta["id"] + ".json")).write_text(json.dumps(meta, indent=2) + "\n")
    return meta
