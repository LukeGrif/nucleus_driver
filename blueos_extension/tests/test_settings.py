import json

import pytest

from nucleus_extension.settings import DEFAULTS, SETTINGS_VERSION, load_settings, save_settings, validate_update


@pytest.fixture
def paths(tmp_path):
    return str(tmp_path / "nucleus_mount" / "settings.json"), str(tmp_path / "nucleus" / "settings.json")


def write(path, data):
    import os  # pylint: disable=import-outside-toplevel

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        file.write(data if isinstance(data, str) else json.dumps(data))


def test_defaults_when_no_file(paths):
    assert load_settings(*paths) == DEFAULTS
    assert DEFAULTS["mount_pitch_deg"] == 0.0 and DEFAULTS["mount_roll_deg"] == 0.0


def test_save_load_round_trip(paths):
    settings = dict(DEFAULTS, hostname="192.168.2.201", mount_pitch_deg=30.0, mount_roll_deg=-12.5, rangefinder=True)
    save_settings(settings, paths[0])
    assert load_settings(*paths) == settings
    with open(paths[0], encoding="utf-8") as file:
        assert json.load(file)["version"] == SETTINGS_VERSION


def test_migrates_hostname_from_original_nortek_extension(paths):
    write(paths[1], {"hostname": "192.168.2.201"})
    settings = load_settings(*paths)
    assert settings == dict(DEFAULTS, hostname="192.168.2.201")


def test_legacy_file_is_never_written(paths):
    write(paths[1], {"hostname": "192.168.2.201", "other": "kept"})
    save_settings(load_settings(*paths), paths[0])
    with open(paths[1], encoding="utf-8") as file:
        assert json.load(file) == {"hostname": "192.168.2.201", "other": "kept"}


def test_own_file_takes_precedence_over_legacy(paths):
    write(paths[1], {"hostname": "10.0.0.1"})
    write(paths[0], {"hostname": "192.168.2.201"})
    assert load_settings(*paths)["hostname"] == "192.168.2.201"


def test_file_without_mount_angles_loads_as_pointing_down(paths):
    write(paths[0], {"hostname": "192.168.2.201", "enabled": False})
    settings = load_settings(*paths)
    assert settings["mount_pitch_deg"] == 0.0 and settings["mount_roll_deg"] == 0.0
    assert settings["enabled"] is False


def test_invalid_saved_values_fall_back_to_defaults(paths):
    write(paths[0], '{"mount_pitch_deg": NaN, "mount_roll_deg": 500, "message_type": "X", "unknown": 1}')
    assert load_settings(*paths) == DEFAULTS


@pytest.mark.parametrize("content", ["{not json", "[1, 2]", ""])
def test_corrupted_file_gives_defaults(paths, content):
    write(paths[0], content)
    assert load_settings(*paths) == DEFAULTS


@pytest.mark.parametrize(
    "update",
    [
        {"mount_pitch_deg": float("nan")},
        {"mount_pitch_deg": float("inf")},
        {"mount_roll_deg": -180.5},
        {"mount_roll_deg": "10"},
        {"message_type": "VISION_POSITION_ESTIMATE"},
        {"enabled": "yes"},
        {"hostname": "has space"},
        {"not_a_setting": 1},
        [1],
    ],
)
def test_validate_update_rejects(update):
    with pytest.raises(ValueError):
        validate_update(update)


def test_validate_update_accepts_partial():
    assert validate_update({"mount_pitch_deg": 45, "hostname": " 192.168.2.201 "}) == {
        "mount_pitch_deg": 45.0,
        "hostname": "192.168.2.201",
    }
