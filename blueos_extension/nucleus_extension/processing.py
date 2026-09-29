"""
Turns Nucleus packets (dicts from nucleus_driver) into MAVLink message bodies for mavlink2rest.

No hardware or network dependencies: the link feeds packets in and posts whatever comes out.

Based on handle_packet() of the original Nortek extension (rov_link.py, removed in 56d6ce1), with:
* degrees -> radians fixed (the original divided by 360 instead of 180, halving every angle delta),
* the mounting rotation applied to velocity, angle deltas and range,
* angle deltas wrapped to [-pi, pi] (the original sent ~2*pi when heading crossed 0/360),
* bottom-track packets whose X/Y/Z velocities are flagged invalid are no longer sent,
* optional VISION_SPEED_ESTIMATE instead of VISION_POSITION_DELTA, and optional DISTANCE_SENSOR.
"""

import math
from typing import Any, Dict, List, Optional

from nucleus_extension.mount_rotation import Matrix, MountAngles, project_range, rotate

ID_BOTTOMTRACK = 0xB4
ID_ALTIMETER = 0xAA
ID_AHRS = 0xD2

POSITION_DELTA = "POSITION_DELTA"
SPEED_ESTIMATE = "SPEED_ESTIMATE"
MESSAGE_TYPES = (POSITION_DELTA, SPEED_ESTIMATE)

# Ranges below this (after projection) are not sent, same threshold as the Water Linked extension.
MIN_RANGE_M = 0.05
MAX_RANGE_CM = 5000

BAD_FOM = 3


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi]. Values already inside are returned unchanged (bit for bit)."""
    if -math.pi <= angle <= math.pi:
        return angle
    return (angle + math.pi) % (2 * math.pi) - math.pi


def packet_timestamp(packet: Dict[str, Any]) -> float:
    return packet["timeStamp"] + packet["microSeconds"] * 1e-6


def bottom_track_valid(packet: Dict[str, Any]) -> bool:
    flags = ("status.xVelocityValid", "status.yVelocityValid", "status.zVelocityValid")
    if not all(packet.get(flag, False) for flag in flags):
        return False
    return all(math.isfinite(packet[key]) for key in ("velocityX", "velocityY", "velocityZ"))


class PacketProcessor:  # pylint: disable=too-many-instance-attributes
    """
    Stateful packet handler. handle_packet() returns a list of (kind, message) tuples where kind is
    "VISION_POSITION_DELTA", "VISION_SPEED_ESTIMATE" or "DISTANCE_SENSOR" and message is the
    mavlink2rest "message" body. Packets that are received but not sent are recorded in self.dropped.
    """

    def __init__(self, mount: MountAngles, message_type: str = POSITION_DELTA, rangefinder: bool = False):
        self.mount = mount
        self.message_type = message_type
        self.rangefinder = rangefinder

        self.timestamp_previous: Optional[float] = None
        self.orientation_current: Optional[List[float]] = None
        self.orientation_previous: Optional[List[float]] = None

        self.dropped: Dict[str, int] = {"invalid_velocity": 0, "invalid_range": 0, "no_mount_angle": 0}
        # Latest values for the web page: raw (DVL frame) and rotated (vehicle frame).
        self.last: Dict[str, Any] = {}

    def reset(self) -> None:
        self.timestamp_previous = None
        self.orientation_current = None
        self.orientation_previous = None

    def handle_packet(self, packet: Optional[Dict[str, Any]]) -> List[tuple]:
        if packet is None:
            return []
        packet_id = packet.get("id")
        if packet_id == ID_BOTTOMTRACK:
            return self._handle_bottom_track(packet)
        if packet_id == ID_AHRS:
            self._handle_ahrs(packet)
            return []
        if packet_id == ID_ALTIMETER and self.rangefinder:
            return self._handle_altimeter(packet)
        return []

    def _handle_ahrs(self, packet: Dict[str, Any]) -> None:
        self.orientation_current = [
            math.radians(packet["ahrsData.roll"]),
            math.radians(packet["ahrsData.pitch"]),
            math.radians(packet["ahrsData.heading"]),
        ]

    def _angle_delta(self) -> List[float]:
        # Same state handling as the original extension.
        if self.orientation_current is None:
            return [0, 0, 0]
        if self.orientation_previous is None or self.timestamp_previous is None:
            self.orientation_previous = self.orientation_current
            return [0, 0, 0]
        delta = [
            wrap_angle(current - previous)
            for (current, previous) in zip(self.orientation_current, self.orientation_previous)
        ]
        self.orientation_previous = self.orientation_current
        return delta

    def _handle_bottom_track(self, packet: Dict[str, Any]) -> List[tuple]:
        fom = max(packet["fomX"], packet["fomY"], packet["fomZ"])
        confidence = (BAD_FOM - min(fom, BAD_FOM)) * 100 / BAD_FOM

        velocity_dvl = (packet["velocityX"], packet["velocityY"], packet["velocityZ"])
        timestamp = packet_timestamp(packet)

        if self.timestamp_previous is None:
            dt = 0
        else:
            dt = timestamp - self.timestamp_previous

        angle_delta_dvl = self._angle_delta()
        self.timestamp_previous = timestamp

        self.last["velocity_dvl"] = list(velocity_dvl)
        self.last["velocity_valid"] = bottom_track_valid(packet)

        if not self.last["velocity_valid"]:
            self.dropped["invalid_velocity"] += 1
            return []

        matrix = self.mount.matrix_at(timestamp)
        if matrix is None:
            self.dropped["no_mount_angle"] += 1
            return []

        velocity = rotate(matrix, velocity_dvl)
        self.last["velocity_vehicle"] = list(velocity)
        self.last["timestamp"] = timestamp

        if self.message_type == SPEED_ESTIMATE:
            return [("VISION_SPEED_ESTIMATE", self._speed_estimate(timestamp, velocity))]

        angle_delta = list(rotate(matrix, angle_delta_dvl))
        position_delta = [velocity[0] * dt, velocity[1] * dt, velocity[2] * dt]
        return [
            (
                "VISION_POSITION_DELTA",
                {
                    "type": "VISION_POSITION_DELTA",
                    "time_usec": 0,
                    "time_delta_usec": int(dt * 1e6),
                    "angle_delta": [angle_delta[0], angle_delta[1], angle_delta[2]],
                    "position_delta": [position_delta[0], position_delta[1], position_delta[2]],
                    "confidence": int(confidence),
                },
            )
        ]

    @staticmethod
    def _speed_estimate(timestamp: float, velocity) -> Dict[str, Any]:
        return {
            "type": "VISION_SPEED_ESTIMATE",
            "usec": int(timestamp * 1e6),
            "x": velocity[0],
            "y": velocity[1],
            "z": velocity[2],
            "covariance": [0.0] * 9,
            "reset_counter": 0,
        }

    def _handle_altimeter(self, packet: Dict[str, Any]) -> List[tuple]:
        raw_range = packet.get("altimeterDistance")
        self.last["range_dvl"] = raw_range
        if not packet.get("status.altimeterDistanceValid", False):
            self.dropped["invalid_range"] += 1
            self.last["range_vehicle"] = None
            return []

        matrix: Optional[Matrix] = self.mount.matrix_at(packet_timestamp(packet))
        if matrix is None:
            self.dropped["no_mount_angle"] += 1
            return []

        distance = project_range(matrix, raw_range)
        self.last["range_vehicle"] = distance
        if distance is None or distance <= MIN_RANGE_M:
            self.dropped["invalid_range"] += 1
            return []

        return [("DISTANCE_SENSOR", distance_sensor_message(distance))]


def distance_sensor_message(distance_m: float) -> Dict[str, Any]:
    return {
        "type": "DISTANCE_SENSOR",
        "time_boot_ms": 0,
        "min_distance": 0,
        "max_distance": MAX_RANGE_CM,
        "current_distance": int(distance_m * 100),
        "mavtype": {"type": "MAV_DISTANCE_SENSOR_ULTRASOUND"},
        "id": 0,
        "orientation": {"type": "MAV_SENSOR_ROTATION_PITCH_270"},
        "covariance": 0,
        "horizontal_fov": 0.0,
        "vertical_fov": 0.0,
        "quaternion": [0.0, 0.0, 0.0, 0.0],
        "signal_quality": 0,
    }
