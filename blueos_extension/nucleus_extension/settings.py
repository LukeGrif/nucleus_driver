"""
Persisted settings (JSON), validation and migration.

The file lives in ~/.config/nucleus_mount/settings.json. The container runs as root and BlueOS
bind-mounts /root/.config, so settings survive restarts and updates.

Migration: if no settings file exists yet, the hostname is imported (read-only) from the original
Nortek extension's ~/.config/nucleus/settings.json. That file is never written, so it cannot clash
with the Nortek extension if it is re-enabled. The original extension had no orientation setting,
so there is nothing else to convert; files without mount angles load with 0 deg (pointing down).
"""

import json
import logging
import os
from typing import Any, Dict, Optional

from nucleus_extension.mount_rotation import validate_angle
from nucleus_extension.processing import MESSAGE_TYPES, POSITION_DELTA

SETTINGS_VERSION = 1

SETTINGS_PATH = os.path.join(os.path.expanduser("~"), ".config", "nucleus_mount", "settings.json")
LEGACY_SETTINGS_PATH = os.path.join(os.path.expanduser("~"), ".config", "nucleus", "settings.json")

DEFAULTS: Dict[str, Any] = {
    "hostname": None,
    "stream_only": False,
    "autoconnect": False,
    "autostart": False,
    "enabled": True,
    "message_type": POSITION_DELTA,
    "rangefinder": False,
    "mount_pitch_deg": 0.0,
    "mount_roll_deg": 0.0,
}


def _bool(value, name):
    if isinstance(value, bool):
        return value
    raise ValueError(f"{name} must be true or false, got {value!r}")


def _hostname(value, name):
    if value is None:
        return None
    if isinstance(value, str) and value.strip() and len(value) <= 253 and " " not in value.strip():
        return value.strip()
    raise ValueError(f"{name} must be a hostname or IP address, got {value!r}")


def _message_type(value, name):
    if value in MESSAGE_TYPES:
        return value
    raise ValueError(f"{name} must be one of {', '.join(MESSAGE_TYPES)}, got {value!r}")


VALIDATORS = {
    "hostname": _hostname,
    "stream_only": _bool,
    "autoconnect": _bool,
    "autostart": _bool,
    "enabled": _bool,
    "message_type": _message_type,
    "rangefinder": _bool,
    "mount_pitch_deg": validate_angle,
    "mount_roll_deg": validate_angle,
}


def validate_update(update: Any) -> Dict[str, Any]:
    """Validate a (partial) settings update. Raises ValueError naming the first bad or unknown key."""
    if not isinstance(update, dict):
        raise ValueError("settings must be a JSON object")
    unknown = sorted(set(update) - set(VALIDATORS))
    if unknown:
        raise ValueError(f"unknown setting(s): {', '.join(unknown)}")
    return {key: VALIDATORS[key](value, key) for key, value in update.items()}


def _read_json(path: str) -> Optional[dict]:
    try:
        with open(path, encoding="utf-8") as file:
            data = json.load(file)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as error:
        logging.warning(f"Could not read settings file {path}: {error}")
        return None
    if not isinstance(data, dict):
        logging.warning(f"Settings file {path} does not contain a JSON object, ignoring it")
        return None
    return data


def migrate(data: dict) -> dict:
    """Bring an older settings dict up to SETTINGS_VERSION. Unknown keys are dropped."""
    migrated = {key: value for key, value in data.items() if key in DEFAULTS}
    migrated["version"] = SETTINGS_VERSION
    return migrated


def load_settings(path: str = SETTINGS_PATH, legacy_path: str = LEGACY_SETTINGS_PATH) -> Dict[str, Any]:
    """Load settings, falling back to defaults per key when a value is missing or invalid."""
    settings = dict(DEFAULTS)
    data = _read_json(path)
    if data is None:
        legacy = _read_json(legacy_path)
        if legacy and "hostname" in legacy:
            logging.info(f"Importing hostname from {legacy_path}")
            data = {"hostname": legacy["hostname"]}
        else:
            return settings

    for key, value in migrate(data).items():
        if key == "version":
            continue
        try:
            settings[key] = VALIDATORS[key](value, key)
        except ValueError as error:
            logging.warning(f"Ignoring saved setting: {error}. Using default {DEFAULTS[key]!r}")
    return settings


def save_settings(settings: Dict[str, Any], path: str = SETTINGS_PATH) -> None:
    """Write atomically so a power cut cannot leave a half-written file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {"version": SETTINGS_VERSION, **{key: settings[key] for key in DEFAULTS}}
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
    os.replace(temporary, path)
