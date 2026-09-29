import math
import random

import pytest

from nucleus_extension import mount_rotation
from nucleus_extension.mount_rotation import (
    MountAngles,
    cos_sin_deg,
    project_range,
    rotate,
    rotation_matrix,
    validate_angle,
)

C30 = math.sqrt(3) / 2  # cos 30 = 0.8660254037844386...
S30 = 0.5


def assert_vec_close(actual, expected, tol=1e-12):
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert a == pytest.approx(e, abs=tol), (actual, expected)


# --- exact values at multiples of 90 deg ---------------------------------------------------------


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, (1.0, 0.0)),
        (-0.0, (1.0, 0.0)),
        (90.0, (0.0, 1.0)),
        (-90.0, (0.0, -1.0)),
        (180.0, (-1.0, 0.0)),
        (-180.0, (-1.0, 0.0)),
    ],
)
def test_cos_sin_exact_at_quarter_turns(angle, expected):
    assert cos_sin_deg(angle) == expected


def test_identity_at_zero():
    assert rotation_matrix(0, 0) == ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def test_forward_matrix_is_exact_permutation():
    assert rotation_matrix(90, 0) == ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))


def test_rotate_at_zero_is_bit_for_bit():
    identity = rotation_matrix(0, 0)
    for vector in [(0.1, -0.2, 0.3), (-0.0, 0.0, -0.0), (1e-300, -1e300, 5e-324), (float("nan"), 1.0, 2.0)]:
        out = rotate(identity, vector)
        assert repr(out) == repr(tuple(vector))


def test_rotate_at_90_is_exact_axis_swap():
    vx, vy, vz = 0.123456789, -0.987654321, 0.5555555555
    assert rotate(rotation_matrix(90, 0), (vx, vy, vz)) == (vz, vy, -vx)
    assert rotate(rotation_matrix(0, 90), (vx, vy, vz)) == (vx, -vz, vy)
    assert rotate(rotation_matrix(180, 0), (vx, vy, vz)) == (-vx, vy, -vz)


def test_naive_trig_would_not_be_exact(monkeypatch):
    """Guards the exactness tests: with plain math.cos/sin the 90 deg preset is not an exact axis swap."""

    def naive(angle_deg):
        return math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))

    monkeypatch.setattr(mount_rotation, "cos_sin_deg", naive)
    vx, vy, vz = 0.123456789, -0.987654321, 0.5555555555
    assert rotate(rotation_matrix(90, 0), (vx, vy, vz)) != (vz, vy, -vx)


# --- 30 deg against hand-computed values ---------------------------------------------------------


def test_pitch_30_hand_computed():
    # R = Ry(30) = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    # v = (0.2, 0.1, 0.4): x = 0.8660254*0.2 + 0.5*0.4 = 0.37320508, y = 0.1, z = -0.5*0.2 + 0.8660254*0.4 = 0.24641016
    assert_vec_close(rotate(rotation_matrix(30, 0), (0.2, 0.1, 0.4)), (0.3732050807568877, 0.1, 0.2464101615137755))


def test_roll_30_hand_computed():
    # R = Rx(30) = [[1, 0, 0], [0, c, -s], [0, s, c]]
    # v = (0.2, 0.1, 0.4): y = 0.8660254*0.1 - 0.5*0.4 = -0.11339746, z = 0.5*0.1 + 0.8660254*0.4 = 0.39641016
    assert_vec_close(rotate(rotation_matrix(0, 30), (0.2, 0.1, 0.4)), (0.2, -0.1133974596215561, 0.3964101615137755))


def test_pitch_30_roll_30_matrix_hand_computed():
    # Ry(30) @ Rx(30) = [[c, s*s, s*c], [0, c, -s], [-s, c*s, c*c]]
    expected = ((C30, 0.25, S30 * C30), (0.0, C30, -S30), (-S30, C30 * S30, 0.75))
    for row, expected_row in zip(rotation_matrix(30, 30), expected):
        assert_vec_close(row, expected_row)


# --- sign convention -----------------------------------------------------------------------------


def test_pitch_90_dvl_looks_forward():
    r = rotation_matrix(90, 0)
    assert rotate(r, (0, 0, 1)) == (1, 0, 0)  # DVL down axis (its beam) = vehicle forward
    assert rotate(r, (1, 0, 0)) == (0, 0, -1)  # DVL nose points up (nose-up tilt)


def test_positive_pitch_is_nose_up():
    r = rotation_matrix(10, 0)
    nose = rotate(r, (1, 0, 0))
    assert nose[0] > 0 and nose[2] < 0  # forward and up (z is down)


def test_positive_roll_is_right_side_down():
    r = rotation_matrix(0, 10)
    right = rotate(r, (0, 1, 0))
    assert right[1] > 0 and right[2] > 0  # right and down


def test_vehicle_forward_motion_recovered_at_any_pitch():
    # Vehicle moves forward at 1 m/s. A DVL pitched by p sees (cos p, 0, sin p) in its own frame.
    for pitch in (0, 15, 30, 45, 60, 90):
        c, s = cos_sin_deg(pitch)
        assert_vec_close(rotate(rotation_matrix(pitch, 0), (c, 0.0, s)), (1.0, 0.0, 0.0))


def test_matrix_is_a_proper_rotation():
    rng = random.Random(3)
    for _ in range(200):
        r = rotation_matrix(rng.uniform(-180, 180), rng.uniform(-180, 180))
        for i in range(3):
            for j in range(3):
                dot = sum(r[k][i] * r[k][j] for k in range(3))
                assert dot == pytest.approx(1.0 if i == j else 0.0, abs=1e-12)
        det = (
            r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
        )
        assert det == pytest.approx(1.0, abs=1e-12)


# --- invalid angles ------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [float("nan"), float("inf"), float("-inf"), 180.0001, -180.0001, 360, 10**400, "30", None, True, [0]],
)
def test_invalid_angles_rejected(bad):
    with pytest.raises(ValueError):
        validate_angle(bad)
    with pytest.raises(ValueError):
        rotation_matrix(bad, 0)
    with pytest.raises(ValueError):
        rotation_matrix(0, bad)


@pytest.mark.parametrize("good", [-180, -180.0, 0, 30, 89.5, 180])
def test_valid_angles_accepted(good):
    assert validate_angle(good) == float(good)


def test_invalid_update_leaves_mount_unchanged():
    mount = MountAngles(30, 5)
    with pytest.raises(ValueError):
        mount.set_fixed(45, float("nan"))
    assert (mount.pitch_deg, mount.roll_deg) == (30.0, 5.0)
    assert mount.matrix_at(0.0) == rotation_matrix(30, 5)


# --- range projection ----------------------------------------------------------------------------


def test_range_projection():
    assert project_range(rotation_matrix(0, 0), 7.25) == 7.25
    assert project_range(rotation_matrix(30, 0), 10.0) == pytest.approx(10.0 * C30)
    assert project_range(rotation_matrix(0, 30), 10.0) == pytest.approx(10.0 * C30)
    assert project_range(rotation_matrix(30, 30), 10.0) == pytest.approx(10.0 * 0.75)


@pytest.mark.parametrize("pitch, roll", [(90, 0), (-90, 0), (120, 0), (180, 0), (0, 90), (0, 180)])
def test_range_not_sent_when_not_looking_down(pitch, roll):
    assert project_range(rotation_matrix(pitch, roll), 10.0) is None


@pytest.mark.parametrize("raw", [-5.0, 0.0, -0.0, float("nan"), float("inf"), None, True])
def test_invalid_raw_range_never_projected(raw):
    for pitch in (0, 30, 180):  # at 180, R22 = -1 would turn -5 into +5 if not checked first
        assert project_range(rotation_matrix(pitch, 0), raw) is None


# --- live (gimbal) angle source ------------------------------------------------------------------


class FakeGimbal:
    """Timestamped angle log with linear interpolation, like a gimbal would provide."""

    def __init__(self, samples):
        self.samples = samples  # [(timestamp, pitch, roll)]
        self.queries = []

    def __call__(self, timestamp):
        self.queries.append(timestamp)
        for (t0, p0, r0), (t1, p1, r1) in zip(self.samples, self.samples[1:]):
            if t0 <= timestamp <= t1:
                f = (timestamp - t0) / (t1 - t0)
                return p0 + f * (p1 - p0), r0 + f * (r1 - r0)
        return None


def test_live_source_overrides_fixed_angle_and_uses_measurement_time():
    gimbal = FakeGimbal([(100.0, 0.0, 0.0), (102.0, 60.0, 0.0)])
    mount = MountAngles(0, 0, source=gimbal)
    assert mount.mount_angles_at(101.0) == (30.0, 0.0)
    assert_vec_close(rotate(mount.matrix_at(101.0), (0.2, 0.1, 0.4)), (0.3732050807568877, 0.1, 0.2464101615137755))
    assert gimbal.queries == [101.0, 101.0]


def test_live_source_without_angle_gives_no_matrix():
    mount = MountAngles(30, 0, source=FakeGimbal([(100.0, 0.0, 0.0), (101.0, 0.0, 0.0)]))
    assert mount.matrix_at(500.0) is None  # no fallback to the fixed 30 deg


def test_live_source_invalid_angle_gives_no_matrix():
    mount = MountAngles(0, 0, source=lambda t: (float("nan"), 0.0))
    assert mount.matrix_at(1.0) is None


def test_removing_live_source_restores_fixed_angle():
    mount = MountAngles(30, 0, source=lambda t: (90.0, 0.0))
    mount.set_source(None)
    assert mount.matrix_at(1.0) == rotation_matrix(30, 0)
