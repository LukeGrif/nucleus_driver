"""
Mounting rotation between the Nucleus (DVL) frame and the vehicle frame.

Pure maths, no hardware or network dependencies.

Frames
------
Both frames are FRD: x forward, y right, z down.

Sign convention
---------------
* ``mount_pitch_deg``: positive = DVL tilted nose-up relative to the vehicle.
  0 deg = DVL pointing straight down (the normal mounting), 90 deg = DVL pointing forward.
* ``mount_roll_deg``: positive = DVL rolled right side down.

Rotation
--------
    R = Ry(pitch) @ Rx(roll)
    v_vehicle = R @ v_dvl

with

    Ry(p) = [[ cos p, 0, sin p],       Rx(r) = [[1,     0,      0],
             [     0, 1,     0],                [0, cos r, -sin r],
             [-sin p, 0, cos p]]                [0, sin r,  cos r]]

cos/sin are exact (0, +1, -1) at multiples of 90 deg, and rotate() skips zero
coefficients, so the 0 deg and 90 deg presets reproduce an axis copy / swap
bit for bit instead of adding floating point noise.
"""

import math
from typing import Callable, Optional, Sequence, Tuple

Matrix = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
Vector = Tuple[float, float, float]

ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0

# A live angle source (e.g. a gimbal) is called with the measurement timestamp in
# seconds (Nucleus clock) and returns (pitch_deg, roll_deg), or None when it has no
# angle for that time.
AngleSource = Callable[[float], Optional[Tuple[float, float]]]


def validate_angle(value, name: str = "angle") -> float:
    """Return value as float, or raise ValueError for NaN, inf, non-numbers or values outside [-180, 180]."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number, got {value!r}")
    try:
        angle = float(value)
    except OverflowError as error:  # huge ints
        raise ValueError(f"{name} must be finite, got {value!r}") from error
    if not math.isfinite(angle):
        raise ValueError(f"{name} must be finite, got {value!r}")
    if not ANGLE_MIN_DEG <= angle <= ANGLE_MAX_DEG:
        raise ValueError(f"{name} must be within [{ANGLE_MIN_DEG:g}, {ANGLE_MAX_DEG:g}] degrees, got {angle!r}")
    return angle


def cos_sin_deg(angle_deg: float) -> Tuple[float, float]:
    """cos and sin of an angle in degrees, exact at multiples of 90 deg."""
    quarter_turns, remainder = divmod(angle_deg, 90.0)
    if remainder == 0.0:
        return ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))[int(quarter_turns) % 4]
    radians = math.radians(angle_deg)
    return math.cos(radians), math.sin(radians)


def rotation_matrix(pitch_deg: float, roll_deg: float) -> Matrix:
    """R = Ry(pitch) @ Rx(roll), mapping DVL-frame vectors to the vehicle frame."""
    cos_p, sin_p = cos_sin_deg(validate_angle(pitch_deg, "mount_pitch_deg"))
    cos_r, sin_r = cos_sin_deg(validate_angle(roll_deg, "mount_roll_deg"))
    return (
        (cos_p, sin_p * sin_r, sin_p * cos_r),
        (0.0, cos_r, -sin_r),
        (-sin_p, cos_p * sin_r, cos_p * cos_r),
    )


def _dot(row: Sequence[float], vector: Sequence[float]) -> float:
    total = None
    for coefficient, value in zip(row, vector):
        if coefficient == 0.0:
            continue
        if coefficient == 1.0:
            term = value
        elif coefficient == -1.0:
            term = -value
        else:
            term = coefficient * value
        total = term if total is None else total + term
    return 0.0 if total is None else total


def rotate(matrix: Matrix, vector: Sequence[float]) -> Vector:
    """matrix @ vector. Zero coefficients are skipped and +-1 copy the value, so axis permutations are exact."""
    return (_dot(matrix[0], vector), _dot(matrix[1], vector), _dot(matrix[2], vector))


def down_component(matrix: Matrix) -> float:
    """R[2][2]: how much of the DVL's own down axis points along the vehicle's down axis."""
    return matrix[2][2]


def project_range(matrix: Matrix, raw_range: float) -> Optional[float]:
    """
    Project a range measured along the DVL's down axis onto the vehicle's down axis (range * R[2][2]).

    Returns None when the raw range is not a finite positive number (checked before projecting, so an
    invalid negative value can never become positive) or when the DVL is not looking down (R[2][2] <= 0).
    """
    if isinstance(raw_range, bool) or not isinstance(raw_range, (int, float)):
        return None
    if not math.isfinite(raw_range) or raw_range <= 0.0:
        return None
    r22 = down_component(matrix)
    if r22 <= 0.0:
        return None
    if r22 == 1.0:
        return raw_range
    return raw_range * r22


class MountAngles:
    """
    Where the rotation code gets the mounting angles from.

    The fixed angles come from the saved settings. An optional live source (e.g. a gimbal) overrides
    them: mount_angles_at(timestamp) asks it for the angles at the measurement's own timestamp. If the
    live source has no angle for that time it returns None and the measurement must not be sent,
    because falling back to the fixed angle could be badly wrong for a moving mount.
    """

    def __init__(self, pitch_deg: float = 0.0, roll_deg: float = 0.0, source: Optional[AngleSource] = None):
        self._pitch_deg = 0.0
        self._roll_deg = 0.0
        self._matrix = rotation_matrix(0.0, 0.0)
        self.source = source
        self.set_fixed(pitch_deg, roll_deg)

    @property
    def pitch_deg(self) -> float:
        return self._pitch_deg

    @property
    def roll_deg(self) -> float:
        return self._roll_deg

    def set_fixed(self, pitch_deg, roll_deg) -> None:
        """Validate both angles first, so an invalid value leaves the current setting untouched."""
        pitch = validate_angle(pitch_deg, "mount_pitch_deg")
        roll = validate_angle(roll_deg, "mount_roll_deg")
        self._matrix = rotation_matrix(pitch, roll)
        self._pitch_deg, self._roll_deg = pitch, roll

    def set_source(self, source: Optional[AngleSource]) -> None:
        self.source = source

    def mount_angles_at(self, timestamp: float) -> Optional[Tuple[float, float]]:
        """(pitch_deg, roll_deg) valid at timestamp, or None if the live source cannot provide them."""
        if self.source is None:
            return self._pitch_deg, self._roll_deg
        angles = self.source(timestamp)
        if angles is None:
            return None
        return validate_angle(angles[0], "live pitch"), validate_angle(angles[1], "live roll")

    def matrix_at(self, timestamp: float) -> Optional[Matrix]:
        """Rotation matrix valid at timestamp, or None if no angle is available."""
        if self.source is None:
            return self._matrix
        try:
            angles = self.mount_angles_at(timestamp)
        except (ValueError, TypeError, IndexError):
            return None
        if angles is None:
            return None
        return rotation_matrix(*angles)
