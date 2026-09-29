import json
import math

import pytest

from nucleus_extension.mount_rotation import MountAngles, rotation_matrix
from nucleus_extension.processing import POSITION_DELTA, SPEED_ESTIMATE, PacketProcessor, wrap_angle
from tests.legacy_reference import (
    DVL_DOWN,
    DVL_FORWARD,
    LegacyRovLink,
    water_linked_position_delta,
    water_linked_rangefinder_cm,
    water_linked_speed,
)
from tests.packets import T0, ahrs, altimeter, bottom_track, stream

C30 = math.sqrt(3) / 2


def full_stream():
    # Starts with a bottom-track packet before any AHRS packet, to cover that branch as well.
    return [bottom_track(T0, 0.1, 0.2, 0.3)] + stream()


def run(processor, packets, kind=None):
    out = []
    for packet in packets:
        out.extend(processor.handle_packet(packet))
    return [message for (k, message) in out if kind is None or k == kind]


def run_legacy(packets, fix_d2r=True):
    legacy = LegacyRovLink(packets)
    if fix_d2r:
        legacy.d2r = math.radians  # the documented bug fix; everything else is the original code
    legacy.run_all()
    return legacy.sent


def processor_at(pitch, roll=0.0, **kwargs):
    return PacketProcessor(MountAngles(pitch, roll), **kwargs)


# --- 0 deg reproduces the original extension exactly ---------------------------------------------


def test_position_delta_at_zero_equals_legacy_exactly():
    packets = full_stream()
    new = run(processor_at(0), packets, "VISION_POSITION_DELTA")
    legacy = run_legacy(packets)
    assert len(new) == len(legacy) > 150
    assert new == legacy
    # Bit for bit, as serialised for mavlink2rest (catches 0.0 vs -0.0 and int vs float).
    assert json.dumps(new) == json.dumps(legacy)


def test_legacy_d2r_bug_halved_angle_deltas():
    packets = full_stream()
    fixed = run_legacy(packets, fix_d2r=True)
    buggy = run_legacy(packets, fix_d2r=False)
    ratios = [b / f for fm, bm in zip(fixed, buggy) for f, b in zip(fm["angle_delta"], bm["angle_delta"]) if f != 0]
    assert ratios and all(r == pytest.approx(0.5, rel=1e-5) for r in ratios)
    # Everything except the angle deltas is identical.
    for fm, bm in zip(fixed, buggy):
        assert {k: v for k, v in fm.items() if k != "angle_delta"} == {
            k: v for k, v in bm.items() if k != "angle_delta"
        }


def test_speed_estimate_at_zero_equals_unrotated_velocity_exactly():
    packets = full_stream()
    new = run(processor_at(0, message_type=SPEED_ESTIMATE), packets, "VISION_SPEED_ESTIMATE")
    bottom = [p for p in packets if p["id"] == 0xB4]
    assert len(new) == len(bottom)
    for message, packet in zip(new, bottom):
        expected = water_linked_speed(DVL_DOWN, packet["velocityX"], packet["velocityY"], packet["velocityZ"])
        assert [message["x"], message["y"], message["z"]] == expected
        assert json.dumps([message["x"], message["y"], message["z"]]) == json.dumps(expected)


def test_rangefinder_at_zero_equals_unrotated_range_exactly():
    packets = full_stream()
    new = run(processor_at(0, rangefinder=True), packets, "DISTANCE_SENSOR")
    expected = [water_linked_rangefinder_cm(p["altimeterDistance"]) for p in packets if p["id"] == 0xAA]
    assert [m["current_distance"] for m in new] == [e for e in expected if e is not None]
    assert len(new) > 50


def test_rangefinder_is_off_by_default():
    assert run(processor_at(0), full_stream(), "DISTANCE_SENSOR") == []


# --- 90 deg reproduces the Water Linked FORWARD mapping exactly ----------------------------------


def test_position_delta_at_90_equals_water_linked_forward_exactly():
    packets = full_stream()
    new = run(processor_at(90), packets, "VISION_POSITION_DELTA")
    down = run(processor_at(0), packets, "VISION_POSITION_DELTA")
    assert len(new) == len(down)
    for n, d in zip(new, down):
        position, attitude = water_linked_position_delta(DVL_FORWARD, *d["position_delta"], *d["angle_delta"])
        assert n["position_delta"] == position
        assert n["angle_delta"] == attitude
        assert json.dumps(n["position_delta"]) == json.dumps(position)


def test_speed_estimate_at_90_equals_water_linked_forward_exactly():
    packets = full_stream()
    new = run(processor_at(90, message_type=SPEED_ESTIMATE), packets)
    bottom = [p for p in packets if p["id"] == 0xB4]
    for message, packet in zip(new, bottom):
        expected = water_linked_speed(DVL_FORWARD, packet["velocityX"], packet["velocityY"], packet["velocityZ"])
        assert [message["x"], message["y"], message["z"]] == expected


def test_no_rangefinder_when_looking_forward():
    assert run(processor_at(90, rangefinder=True), full_stream(), "DISTANCE_SENSOR") == []


# --- 30 deg against hand-computed values ---------------------------------------------------------


def test_pitch_30_velocity_and_range_hand_computed():
    processor = processor_at(30, message_type=SPEED_ESTIMATE, rangefinder=True)
    kind, message = processor.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4))[0]
    assert kind == "VISION_SPEED_ESTIMATE"
    # x = 0.8660254*0.2 + 0.5*0.4, y = 0.1, z = -0.5*0.2 + 0.8660254*0.4
    assert message["x"] == pytest.approx(0.3732050807568877, abs=1e-12)
    assert message["y"] == 0.1
    assert message["z"] == pytest.approx(0.2464101615137755, abs=1e-12)
    assert message["usec"] == T0 * 1_000_000

    kind, message = processor.handle_packet(altimeter(T0, 10.0))[0]
    assert kind == "DISTANCE_SENSOR"
    assert message["current_distance"] == 866  # int(10 m * cos 30 * 100)


def test_pitch_30_position_and_angle_delta_hand_computed():
    processor = processor_at(30)
    processor.handle_packet(ahrs(T0, 0.0, 0.0, 10.0))
    processor.handle_packet(bottom_track(T0, 0.0, 0.0, 0.0))  # first: initialises previous values
    processor.handle_packet(ahrs(T0 + 0.25, 2.0, 0.0, 10.0))  # +2 deg roll of the DVL
    ((_, message),) = processor.handle_packet(bottom_track(T0 + 0.5, 0.2, 0.1, 0.4))
    assert message["time_delta_usec"] == 500000
    for actual, expected in zip(message["position_delta"], (0.3732050807568877 / 2, 0.05, 0.2464101615137755 / 2)):
        assert actual == pytest.approx(expected, abs=1e-12)
    d = math.radians(2.0)
    # A DVL-frame roll rate maps to vehicle roll (cos 30) and yaw (-sin 30 -> R[2][0] = -0.5)
    for actual, expected in zip(message["angle_delta"], (C30 * d, 0.0, -0.5 * d)):
        assert actual == pytest.approx(expected, abs=1e-12)


# --- validity, wrap and live source --------------------------------------------------------------


def test_invalid_bottom_track_not_sent_but_state_advances():
    processor = processor_at(0)
    processor.handle_packet(bottom_track(T0, 0.1, 0.1, 0.1))
    assert processor.handle_packet(bottom_track(T0 + 1, -32.768, -32.768, -32.768, valid=False)) == []
    assert processor.dropped["invalid_velocity"] == 1
    ((_, message),) = processor.handle_packet(bottom_track(T0 + 1.5, 0.2, 0.0, 0.0))
    assert message["time_delta_usec"] == 500000  # measured from the invalid packet, like the original


def test_invalid_range_not_sent():
    processor = processor_at(0, rangefinder=True)
    assert processor.handle_packet(altimeter(T0, 5.0, valid=False)) == []
    assert processor.handle_packet(altimeter(T0, -5.0)) == []
    assert processor.handle_packet(altimeter(T0, 0.04)) == []
    assert processor.dropped["invalid_range"] == 3


def test_upside_down_negative_range_does_not_become_positive():
    processor = processor_at(180, rangefinder=True)
    assert processor.handle_packet(altimeter(T0, -5.0)) == []


def test_heading_wrap():
    processor = processor_at(0)
    processor.handle_packet(ahrs(T0, 0.0, 0.0, 359.0))
    processor.handle_packet(bottom_track(T0, 0.0, 0.0, 0.0))
    processor.handle_packet(ahrs(T0 + 0.1, 0.0, 0.0, 1.0))
    ((_, message),) = processor.handle_packet(bottom_track(T0 + 0.2, 0.0, 0.0, 0.0))
    assert message["angle_delta"][2] == pytest.approx(math.radians(2.0))


def test_wrap_angle_leaves_in_range_values_untouched():
    for value in (0.0, -0.0, 1.0, -math.pi, math.pi, 3.1):
        assert repr(wrap_angle(value)) == repr(value)
    assert wrap_angle(math.radians(-358)) == pytest.approx(math.radians(2))


def test_live_source_is_queried_with_measurement_timestamp():
    queries = []

    def gimbal(timestamp):
        queries.append(timestamp)
        return (90.0, 0.0) if timestamp >= T0 + 1 else (0.0, 0.0)

    processor = PacketProcessor(MountAngles(0, 0, source=gimbal), message_type=SPEED_ESTIMATE, rangefinder=True)
    ((_, early),) = processor.handle_packet(bottom_track(T0 + 0.5, 0.2, 0.1, 0.4))
    ((_, late),) = processor.handle_packet(bottom_track(T0 + 1.25, 0.2, 0.1, 0.4))
    assert processor.handle_packet(altimeter(T0 + 2.5, 3.0)) == []  # gimbal looking forward
    assert queries == [T0 + 0.5, T0 + 1.25, T0 + 2.5]
    assert (early["x"], early["y"], early["z"]) == (0.2, 0.1, 0.4)
    assert (late["x"], late["y"], late["z"]) == (0.4, 0.1, -0.2)


def test_live_source_without_angle_drops_measurement():
    processor = PacketProcessor(MountAngles(0, 0, source=lambda t: None))
    assert processor.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4)) == []
    assert processor.dropped["no_mount_angle"] == 1


def test_changing_angle_takes_effect_immediately():
    mount = MountAngles(0, 0)
    processor = PacketProcessor(mount, message_type=SPEED_ESTIMATE)
    mount.set_fixed(90, 0)
    ((_, message),) = processor.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4))
    assert (message["x"], message["y"], message["z"]) == (0.4, 0.1, -0.2)
    assert mount.matrix_at(T0) == rotation_matrix(90, 0)


@pytest.mark.parametrize("message_type", [POSITION_DELTA, SPEED_ESTIMATE])
def test_other_packets_ignored(message_type):
    processor = processor_at(0, message_type=message_type, rangefinder=True)
    assert processor.handle_packet({"id": 0xDC}) == []
    assert processor.handle_packet(None) == []
