"""Bring normalized MATLAB recordings into the same identification pipeline."""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import numpy as np

from .friction import load_csv
from .params import H, RUNS, load_hardware
from .sysid import COL


def import_csv(path, hardware=False):
    path = Path(path); data = load_csv(path); hw = load_hardware()
    if "v" not in data:
        raise ValueError("Import requires logical voltage v; export with MATLAB read_course_data first")
    n = len(data["t"]); t = data["t"]
    if n < 1000 or not np.isfinite(np.column_stack(list(data.values()))).all():
        raise ValueError("Need at least 1,000 finite samples")
    if np.max(np.abs(np.diff(t)-H)) > H*.02:
        raise ValueError("Import requires uniform 500 Hz samples; do not relabel a slower recording")
    if np.max(np.abs(data["v"])) > hw.v_limit+.01:
        raise ValueError("Voltage exceeds hardware limit; check units and whether it is applied voltage")
    arr = np.zeros((n, len(COL))); alpha = np.unwrap(data["alpha"])
    for key in ("t", "x", "v"):
        arr[:, COL[key]] = data[key]
    arr[:, COL["alpha"]] = np.pi-np.mod(np.pi-alpha,2*np.pi)
    arr[:, COL["v_des"]] = data["v"]
    arr[:, COL["cart_counts"]] = np.rint(data["x"]/hw.cart_m_per_count)
    arr[:, COL["pole_counts"]] = np.rint((alpha+np.pi)/hw.pole_rad_per_count)
    arr[:, COL["xd"]] = np.gradient(data["x"],H)
    arr[:, COL["ad"]] = np.gradient(alpha,H)
    rid = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    meta = {"id": rid, "sim": not hardware, "backend": "MATLAB/QUARC import" if hardware else "MATLAB simulator import",
            "mode": "import", "duration_s": float(t[-1]-t[0]), "n": n, "h": H,
            "t0": float(t[0]), "hardware": asdict(hw), "cols": list(COL),
            "end": "imported recording", "voltage_convention": "logical_command_before_volts_sign",
            "source_file": str(path.resolve()), "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    RUNS.mkdir(parents=True, exist_ok=True)
    np.save(RUNS/f"{rid}.npy", arr)
    np.savetxt(RUNS/f"{rid}.csv", arr, delimiter=",", header=",".join(COL), comments="")
    (RUNS/f"{rid}.json").write_text(json.dumps(meta,indent=2)+"\n")
    return meta
