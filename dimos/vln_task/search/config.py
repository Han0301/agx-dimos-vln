import math

import yaml

from dimos.plugins._agent_common import ROOT

DEFAULT_CONFIG = ROOT / "config/plugins/active_search/active_search.yaml"


def load_config(path=DEFAULT_CONFIG):
    settings = yaml.safe_load(path.read_text())
    keys = {"min_translation_m", "min_rotation_deg"}
    retired = {"verification_mode", "timeout_s", "max_attempts", "max_target_shift_m"}
    required = keys | {"plugin"}
    if (
        not isinstance(settings, dict)
        or not required <= settings.keys() <= required | retired
        or settings["plugin"] != "active_search"
    ):
        raise ValueError("Invalid active_search config")
    for key in keys:
        value = settings[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"active_search.{key} must be positive and finite")
    if settings["min_rotation_deg"] > 180:
        raise ValueError("active_search.min_rotation_deg must be <= 180")
    # Accept retired keys in older profiles.
    for key in retired:
        settings.pop(key, None)
    return settings
