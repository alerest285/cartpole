"""Read-only lab preflight: no board is opened and no outputs are written."""
import importlib.util
import json
import platform
import sys

import numpy as np

from .controllers.learned import policy_path, registry
from .params import H, ROOT, load_hardware, load_plant
from .rl.policy import Policy


def doctor(require_hardware=False):
    reg, hw = registry(), load_hardware()
    version = reg.get("current")
    policy_ok = False
    try:
        p = Policy.load(policy_path(version))
        policy_ok = bool(np.isfinite(p.mean(np.zeros(6))).all())
    except (TypeError, OSError, ValueError):
        pass
    try:
        sdk = importlib.util.find_spec("quanser.hardware") is not None
    except (ModuleNotFoundError, ValueError):
        sdk = False
    report = {
        "python": sys.version.split()[0], "platform": platform.platform(),
        "numpy": np.__version__, "root": str(ROOT), "sample_period_s": H,
        "policy": version, "policy_loads": policy_ok, "quanser_api_found": sdk,
        "hardware_map": hw.__dict__, "plant": load_plant().to_dict(),
        "matlab_contract_exists": (ROOT / "matlab/generated/contract.json").is_file(),
        "board_opened": False, "hardware_validated": False,
        "next": "At the rig: stop QUARC, centre cart, hang pole, then python -m cartpole probe",
    }
    print(json.dumps(report, indent=2))
    return 0 if policy_ok and (sdk or not require_hardware) else 1
