"""
Connection to the Nucleus (via nucleus_driver) and to ArduSub (via BlueOS mavlink2rest).

Connection, start/stop and device setup follow the original Nortek extension (rov_link.py).
"""

import logging
import math
import re
import threading
import time
from typing import Any, Callable, Dict, Optional

import requests

from nucleus_extension.mount_rotation import AngleSource, MountAngles
from nucleus_extension.processing import PacketProcessor
from nucleus_extension.settings import DEFAULTS, save_settings

HOST_URL = "http://host.docker.internal"
MAVLINK2REST_URL = f"{HOST_URL}/mavlink2rest/mavlink"

COMMAND_PORT = 9000
STREAM_PORT = 9002
AUTOCONNECT_INTERVAL_S = 5.0

MESSAGE_KINDS = ("VISION_POSITION_DELTA", "VISION_SPEED_ESTIMATE", "DISTANCE_SENSOR")


class Mavlink2RestSender:
    def __init__(self, url: str = MAVLINK2REST_URL, timeout: float = 1.0):
        self.url = url
        self.timeout = timeout
        self.session = requests.Session()

    def send(self, message: Dict[str, Any]) -> bool:
        body = {"header": {"system_id": 255, "component_id": 0, "sequence": 0}, "message": message}
        try:
            response = self.session.post(self.url, json=body, timeout=self.timeout)
        except requests.RequestException as error:
            logging.warning(f"{message['type']} failed: {error}")
            return False
        if 200 <= response.status_code < 300:
            return True
        logging.warning(f"{message['type']} not accepted: {response.status_code} - {response.text}")
        return False


def _json_safe(value):
    """NaN/inf are not valid JSON for the browser: show them as null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def parse_device_rotation(reply) -> Optional[Dict[str, float]]:
    """Parse the reply to GETINST,ROTXY,ROTYZ,ROTXZ into a dict, or None if it cannot be parsed."""
    lines = reply if isinstance(reply, list) else [reply]
    for line in lines:
        text = line.decode(errors="replace") if isinstance(line, bytes) else str(line)
        if "OK" in text or "ERROR" in text:
            continue
        numbers = re.findall(r"-?\d+(?:\.\d+)?", text)
        if len(numbers) >= 3:
            return dict(zip(("ROTXY", "ROTYZ", "ROTXZ"), (float(number) for number in numbers[-3:])))
    return None


# pylint: disable=too-many-instance-attributes
class NucleusLink:
    def __init__(
        self,
        settings: Dict[str, Any],
        driver=None,
        sender: Optional[Mavlink2RestSender] = None,
        save: Callable[[Dict[str, Any]], None] = save_settings,
    ):
        if driver is None:
            from nucleus_driver import NucleusDriver  # pylint: disable=import-outside-toplevel

            driver = NucleusDriver()
        self.driver = driver
        self.sender = sender or Mavlink2RestSender()
        self._save = save
        self.settings = dict(DEFAULTS, **settings)

        self.mount = MountAngles(self.settings["mount_pitch_deg"], self.settings["mount_roll_deg"])
        self.processor = PacketProcessor(
            self.mount, message_type=self.settings["message_type"], rangefinder=self.settings["rangefinder"]
        )

        self.lock = threading.RLock()
        self.thread: Optional[threading.Thread] = None
        self.thread_running = False

        self.connected = False
        self.running = False
        self.status_text = "Not connected"
        self.nucleus_id = None
        self.nucleus_firmware = None
        self.device_rotation: Optional[Dict[str, float]] = None
        self.device_rotation_raw = ""
        self._last_autoconnect = 0.0
        self._autostart_done = False

        self.counters = {kind: {"sent": 0, "failed": 0, "skipped": 0} for kind in MESSAGE_KINDS}

    # Settings ------------------------------------------------------------------------------------

    def update_settings(self, update: Dict[str, Any]) -> None:
        """Apply an already validated update and save it."""
        with self.lock:
            if "mount_pitch_deg" in update or "mount_roll_deg" in update:
                pitch = update.get("mount_pitch_deg", self.settings["mount_pitch_deg"])
                roll = update.get("mount_roll_deg", self.settings["mount_roll_deg"])
                self.mount.set_fixed(pitch, roll)
            self.settings.update(update)
            self.processor.message_type = self.settings["message_type"]
            self.processor.rangefinder = self.settings["rangefinder"]
            self._save(self.settings)

    def set_angle_source(self, source: Optional[AngleSource]) -> None:
        """Plug in a live (e.g. gimbal) angle source. It overrides the fixed mount angles."""
        self.mount.set_source(source)

    # Nucleus -------------------------------------------------------------------------------------

    def connect(self) -> bool:
        with self.lock:
            hostname = self.settings["hostname"]
            if not hostname:
                self.status_text = "No hostname set"
                return False
            if self.connected:
                self.disconnect()
            port = STREAM_PORT if self.settings["stream_only"] else COMMAND_PORT
            self.status_text = f"Connecting to {hostname}:{port}..."
            self.driver.set_tcp_configuration(host=hostname, port=port)
            if not self.driver.connect(connection_type="tcp"):
                self.status_text = f"Failed to connect to {hostname}:{port}"
                return False
            self.connected = True
            self.running = False
            self.nucleus_id = getattr(self.driver.connection, "nucleus_id", None)
            self.nucleus_firmware = getattr(self.driver.connection, "firmware_version", None)
            self.status_text = f"Connected to {hostname}:{port}"
            logging.info(f"Nucleus connected. ID: {self.nucleus_id} firmware: {self.nucleus_firmware}")
            return True

    def disconnect(self) -> bool:
        with self.lock:
            if self.running:
                self.stop()
            ok = bool(self.driver.disconnect())
            self.connected = False
            self.running = False
            self.status_text = "Disconnected" if ok else "Failed to disconnect"
            return ok

    def read_device_rotation(self) -> None:
        """Read the Nucleus' own mounting rotation so it is not applied twice (only on the command port)."""
        try:
            reply = self.driver.commands.get_inst(rotxy=True, rotyz=True, rotxz=True)
        except Exception as error:
            logging.warning(f"GETINST failed: {error}")
            return
        self.device_rotation_raw = b"".join(line for line in reply if isinstance(line, bytes)).decode(errors="replace")
        self.device_rotation = parse_device_rotation(reply)

    def setup_nucleus(self) -> bool:
        """Same device configuration as the original extension."""
        reply_ahrs = self.driver.commands.set_ahrs(ds="ON")
        reply_bt = self.driver.commands.set_bt(wt="ON", ds="ON")
        ok = b"OK\r\n" in reply_ahrs and b"OK\r\n" in reply_bt
        if not ok:
            logging.warning(f"Nucleus setup failed. SETAHRS: {reply_ahrs} SETBT: {reply_bt}")
        return ok

    def start(self) -> bool:
        with self.lock:
            if not self.connected:
                self.status_text = "Not connected"
                return False
            self.processor.reset()
            if self.settings["stream_only"]:
                # Commands cannot be sent on the stream port: only start reading.
                if not self.driver.parser.thread_running:
                    self.driver.parser.start()
                self.running = True
                self.status_text = "Streaming (stream only)"
                return True
            self.driver.stop()
            self.read_device_rotation()
            if not self.setup_nucleus():
                self.status_text = "Nucleus setup failed"
                return False
            if b"OK\r\n" not in self.driver.start_measurement():
                self.status_text = "Failed to start Nucleus"
                return False
            self.running = True
            self.status_text = "Running"
            return True

    def stop(self) -> bool:
        with self.lock:
            if self.settings["stream_only"]:
                if self.driver.parser.thread_running:
                    self.driver.parser.stop()
                self.running = False
                self.status_text = "Stopped reading"
                return True
            reply = self.driver.stop()
            ok = b"OK\r\n" in reply or b"ERROR\r\n" in reply
            if ok:
                self.running = False
                self.status_text = "Stopped"
            else:
                self.status_text = "Failed to stop Nucleus"
            return ok

    # Main loop -----------------------------------------------------------------------------------

    def handle_packet(self, packet) -> None:
        with self.lock:
            outputs = self.processor.handle_packet(packet)
            enabled = self.settings["enabled"]
        for kind, message in outputs:
            counter = self.counters[kind]
            if not enabled:
                counter["skipped"] += 1
            elif self.sender.send(message):
                counter["sent"] += 1
            else:
                counter["failed"] += 1

    def _autoconnect(self) -> None:
        if not self.settings["autoconnect"] or self.connected:
            return
        now = time.monotonic()
        if now - self._last_autoconnect < AUTOCONNECT_INTERVAL_S:
            return
        self._last_autoconnect = now
        if self.connect() and self.settings["autostart"] and not self._autostart_done:
            self._autostart_done = self.start()

    def run(self) -> None:
        self.thread_running = True
        while self.thread_running:
            try:
                self._autoconnect()
                if self.running:
                    packet = self.driver.read_packet()
                    if packet is None:
                        time.sleep(0.005)
                    else:
                        self.handle_packet(packet)
                else:
                    time.sleep(0.05)
            except Exception as error:  # keep the loop alive, the vehicle depends on it
                logging.exception(f"Error in main loop: {error}")
                time.sleep(0.5)

    def start_thread(self) -> None:
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    # Status --------------------------------------------------------------------------------------

    def get_status(self) -> Dict[str, Any]:
        rotation_on_device = bool(self.device_rotation) and any(self.device_rotation.values())
        mount_set = self.mount.pitch_deg != 0.0 or self.mount.roll_deg != 0.0
        return {
            "status": self.status_text,
            "connected": self.connected,
            "running": self.running,
            "nucleus_id": self.nucleus_id,
            "nucleus_firmware": self.nucleus_firmware,
            "settings": dict(self.settings),
            "counters": self.counters,
            "dropped": dict(self.processor.dropped),
            "live": {key: _json_safe(value) for key, value in self.processor.last.items()},
            "live_angle_source": self.mount.source is not None,
            "device_rotation": self.device_rotation,
            "device_rotation_raw": self.device_rotation_raw,
            "double_rotation_warning": rotation_on_device and mount_set,
        }
