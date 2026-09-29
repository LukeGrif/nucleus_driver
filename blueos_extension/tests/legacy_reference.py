"""
Legacy reference implementations, copied verbatim so the tests can compare the new code against them.

1. LegacyRovLink: handle_packet() and d2r() copied unchanged from the original Nortek extension,
   blueos_extension/src/nucleus_extension/rov_link.py at commit 56d6ce1^ of NortekSupport/nucleus_driver
   (the last version before the extension code was removed). Only the I/O around it is replaced:
   read_packet() comes from a list and send_vision_position_delta() records the message it would post.

2. water_linked_*: the orientation handling of the Water Linked DVL extension
   (bluerobotics/BlueOS-Water-Linked-DVL, dvl-a50/dvl.py, handle_velocity()), used as the reference for
   the DOWN (0 deg) and FORWARD (90 deg) axis mappings of the speed estimate and rangefinder paths,
   which the original Nucleus extension did not have.
"""

# pylint: skip-file
# fmt: off
import time


class LegacyRovLink:

    def __init__(self, packets):
        self._packets = list(packets)
        self.sent = []

        self.timestamp_previous = None
        self.orientation_current = None
        self.orientation_previous = None
        self._enable_nucleus_input = True
        self.vision_position_delta_packet_counter = {'packets_sent': 0, 'packets_failed': 0, 'packets_skipped': 0}

        class _Driver:
            def read_packet(driver_self):
                return self._packets.pop(0) if self._packets else None

        self.nucleus_driver = _Driver()

    def send_vision_position_delta(self, position_delta, angle_delta, confidence, dt):
        # Builds the same "message" body as the original, without posting it.
        self.sent.append({
            "type": "VISION_POSITION_DELTA",
            "time_usec": 0,
            "time_delta_usec": dt,
            "angle_delta": [angle_delta[0], angle_delta[1], angle_delta[2]],
            "position_delta": [position_delta[0], position_delta[1], position_delta[2]],
            "confidence": confidence
        })

    def run_all(self):
        while self._packets:
            self.handle_packet()

    # ---- copied verbatim from rov_link.py ----
    def d2r(self, deg):

        return deg * 3.14159 / 360

    def handle_packet(self):

        packet = self.nucleus_driver.read_packet()

        if packet is None:
            time.sleep(0.005)
            return

        if packet['id'] == 0xb4:

            fom_x = packet['fomX']
            fom_y = packet['fomY']
            fom_z = packet['fomZ']

            fom = max(fom_x, fom_y, fom_z)

            bad_fom = 3
            confidence = (3 - min(fom, bad_fom)) * 100 / bad_fom  # TODO: optimize   operate from 0-3

            velocity_x = packet['velocityX']
            velocity_y = packet['velocityY']
            velocity_z = packet['velocityZ']

            timestamp = (packet['timeStamp'] + packet['microSeconds'] * 1e-6)

            if self.timestamp_previous is None:
                dt = 0
            else:
                dt = timestamp - self.timestamp_previous

            dx = velocity_x * dt
            dy = velocity_y * dt
            dz = velocity_z * dt

            delta_orientation = list()

            if self.orientation_current is None:
                delta_orientation = [0, 0, 0]

            elif self.orientation_previous is None or self.timestamp_previous is None:
                delta_orientation = [0, 0, 0]
                self.orientation_previous = self.orientation_current

            else:
                for (angle_current, angle_previous) in zip(self.orientation_current, self.orientation_previous):
                    delta_orientation.append(angle_current - angle_previous)

                self.orientation_previous = self.orientation_current

            self.timestamp_previous = timestamp

            if self._enable_nucleus_input:
                self.send_vision_position_delta(position_delta=[dx, dy, dz], angle_delta=delta_orientation, confidence=int(confidence), dt=int(dt * 1e6))
            else:
                self.vision_position_delta_packet_counter['packets_skipped'] += 1

        elif packet['id'] == 0xd2:

            orientation = list()
            orientation.append(self.d2r(packet['ahrsData.roll']))
            orientation.append(self.d2r(packet['ahrsData.pitch']))
            orientation.append(self.d2r(packet['ahrsData.heading']))

            self.orientation_current = orientation

    # ---- end of verbatim copy ----


# ---- Water Linked dvl.py handle_velocity(), orientation mapping copied verbatim ----
DVL_DOWN = 1
DVL_FORWARD = 2


def water_linked_position_delta(orientation, dx, dy, dz, dRoll, dPitch, dYaw):
    position_delta = [0, 0, 0]
    attitude_delta = [0, 0, 0]
    if orientation == DVL_DOWN:
        position_delta = [dx, dy, dz]
        attitude_delta = [dRoll, dPitch, dYaw]
    elif orientation == DVL_FORWARD:
        position_delta = [dz, dy, -dx]
        attitude_delta = [dYaw, dPitch, -dRoll]
    return position_delta, attitude_delta


def water_linked_speed(orientation, vx, vy, vz):
    velocity = [vx, vy, vz] if orientation == DVL_DOWN else [vz, vy, -vx]  # DVL_FORWARD
    return velocity


def water_linked_rangefinder_cm(alt):
    # `if self.rangefinder and alt > 0.05: self.mav.send_rangefinder(alt)` -> int(distance * 100)
    if alt > 0.05:
        return int(alt * 100)
    return None
