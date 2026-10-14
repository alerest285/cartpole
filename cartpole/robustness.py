"""Repeatable offline tuning against combined uncertainties, never on hardware.

Tune on one seeded plant set, qualify on an unseen set and retain a baseline.
Reports are proposals; no controller defaults, policy or fitted plant is changed.
"""
from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from .controllers import make
from .params import Plant, load_plant
from .safety import SafetyConfig
from .simulate import closed_loop, summary


def scenarios(p, n=12, seed=17):
    rng = np.random.default_rng(seed)
    out = [{"name": "nominal", "plant": p, "bias": (0., 0.), "deadband": 0.}]
    for i in range(n):
        true = replace(p, B_f=float(rng.uniform(.4, 6.0)), B_p=float(rng.uniform(.001, .005)),
                       F_coul=float(rng.uniform(0, .4)), k_amp=float(rng.uniform(.85, 1.15)),
                       m_c=float(p.m_c*rng.uniform(.9, 1.1)),
                       I_factor=float(rng.uniform(.22, .45)), delay_steps=int(rng.integers(0, 4)))
        out.append({"name": f"combined-{seed}-{i:02d}", "plant": true,
                    "bias": (float(rng.uniform(-.002, .002)), float(rng.uniform(-.005, .005))),
                    "deadband": float(rng.uniform(0, .15))})
    return out


def evaluate(key, params, centre, cases, cap, duration=10., log=print):
    results = []
    swing = key in ("energy", "sine", "ppo")
    for case in cases:
        ctrl = make(key, centre); ctrl.set_params(params)
        rows, fault = closed_loop(ctrl, case["plant"], duration,
            s0=None if swing else [0., .05, 0., 0.],
            pokes=[] if swing else [(3., .6)], safety=SafetyConfig(v_cap=cap),
            sensor_bias=case["bias"], motor_deadband=case["deadband"])
        sm = summary(rows, fault)
        tail = rows[:, 0] >= rows[-1, 0] - 2
        sm.update(name=case["name"], voltage_rms=float(np.sqrt(np.mean(rows[tail, 8]**2))),
                  saturation_fraction=float(np.mean(np.abs(rows[:, 8]) >= cap*.99)),
                  clamp_fraction=float(np.mean(np.abs(rows[:, 7]-rows[:, 8]) > .05)),
                  final_cart_rms_cm=float(100*np.sqrt(np.mean(rows[tail, 1]**2))))
        results.append(sm)
    passed = sum(r["balanced_at_end"] for r in results)
    # Failures dominate every performance tradeoff; rank survivors by margin/noise.
    cost = (1000*(len(results)-passed) + max(r["max_abs_x_cm"] for r in results)
            + 2*np.mean([r["voltage_rms"] for r in results])
            + np.mean([r["t_upright"] or duration for r in results])
            + np.mean([r["final_cart_rms_cm"] for r in results]))
    report = {"controller": key, "params": params, "voltage_cap": cap,
              "passed": passed, "total": len(results), "score": float(cost), "cases": results}
    log(f"{key} {params}: {passed}/{len(results)} at {cap:g} V; score {cost:.2f}")
    return report


def tune(output, n=8, duration=10., log=print):
    centre = load_plant(); train = scenarios(centre, n, 17); heldout = scenarios(centre, n*2, 91)
    candidates = {
        "statefb": [{}, {"q_x": 5., "q_a": 80., "r": .15},
                    {"q_x": 8., "q_a": 80., "q_ad": 1., "r": .1},
                    {"q_x": 10., "q_a": 100., "r": .2}],
        "energy": [{}, {"a_max": 4., "k_e": 4., "adapt": .2},
                   {"a_max": 5., "k_e": 5., "adapt": .3},
                   {"a_max": 6., "k_e": 4., "adapt": .3, "eps": 2.}],
    }
    report = {"schema": 1, "source": "simulation only", "model": centre.to_dict(),
              "training_seed": 17, "validation_seed": 91, "duration_s": duration, "controllers": {}}
    for key, choices in candidates.items():
        cap = 4. if key == "statefb" else 10.
        trials = [evaluate(key, p, centre, train, cap, duration, log) for p in choices]
        selected = min(trials, key=lambda r: r["score"])
        baseline = evaluate(key, {}, centre, heldout, cap, duration, log)
        validation = baseline if not selected["params"] else evaluate(key, selected["params"], centre, heldout, cap, duration, log)
        qualified = validation["passed"] == validation["total"] and validation["score"] <= baseline["score"]
        report["controllers"][key] = {"training": trials, "selected": selected["params"],
            "baseline_validation": baseline, "candidate_validation": validation,
            "qualified": qualified, "recommended_params": selected["params"] if qualified else None}
        destination = Path(output); destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n")
    report["uncertainty_set"] = [dict(name=c["name"], plant=c["plant"].to_dict(), sensor_bias=c["bias"],
                                      motor_deadband=c["deadband"]) for c in heldout]
    destination.write_text(json.dumps(report, indent=2) + "\n")
    log(f"Saved {destination}; no gains installed. Hardware validation remains required.")
    return report
