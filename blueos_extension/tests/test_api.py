"""REST API and link tests with a fake Nucleus driver and a fake mavlink2rest sender (no hardware or network)."""

import pytest

from nucleus_extension.api import create_app
from nucleus_extension.link import NucleusLink, parse_device_rotation
from nucleus_extension.settings import DEFAULTS
from tests.packets import T0, altimeter, bottom_track


class FakeParser:
    thread_running = False

    def start(self):
        self.thread_running = True

    def stop(self):
        self.thread_running = False


class FakeCommands:
    def __init__(self, rotation_reply):
        self.rotation_reply = rotation_reply

    def get_inst(self, **_):
        return self.rotation_reply

    def set_ahrs(self, **_):
        return [b"OK\r\n"]

    def set_bt(self, **_):
        return [b"OK\r\n"]


class FakeDriver:
    def __init__(self, rotation_reply=(b"0.00,0.00,0.00\r\n", b"OK\r\n")):
        self.parser = FakeParser()
        self.commands = FakeCommands(list(rotation_reply))
        self.connection = type("Connection", (), {"nucleus_id": 1234, "firmware_version": "test"})()
        self.tcp = None

    def set_tcp_configuration(self, host=None, port=None):
        self.tcp = (host, port)

    def connect(self, connection_type):
        return connection_type == "tcp"

    def disconnect(self):
        return True

    def start_measurement(self):
        return [b"OK\r\n"]

    def stop(self):
        return [b"OK\r\n"]

    def read_packet(self):
        return None


class FakeSender:
    def __init__(self, ok=True):
        self.ok = ok
        self.sent = []

    def send(self, message):
        self.sent.append(message)
        return self.ok


@pytest.fixture
def setup():
    saved = []
    sender = FakeSender()
    link = NucleusLink(dict(DEFAULTS), driver=FakeDriver(), sender=sender, save=lambda s: saved.append(dict(s)))
    client = create_app(link).test_client()
    return link, client, sender, saved


def test_mount_endpoint_sets_saves_and_applies(setup):
    link, client, sender, saved = setup
    response = client.post("/api/mount", json={"mount_pitch_deg": 90, "mount_roll_deg": 0})
    assert response.status_code == 200
    assert response.get_json()["mount_pitch_deg"] == 90.0
    assert saved[-1]["mount_pitch_deg"] == 90.0

    client.post("/api/settings", json={"message_type": "SPEED_ESTIMATE"})
    link.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4))
    assert (sender.sent[-1]["x"], sender.sent[-1]["y"], sender.sent[-1]["z"]) == (0.4, 0.1, -0.2)


@pytest.mark.parametrize(
    "body",
    [
        '{"mount_pitch_deg": NaN, "mount_roll_deg": 0}',
        '{"mount_pitch_deg": Infinity, "mount_roll_deg": 0}',
        '{"mount_pitch_deg": 1e999, "mount_roll_deg": 0}',
        '{"mount_pitch_deg": 181, "mount_roll_deg": 0}',
        '{"mount_pitch_deg": 0, "mount_roll_deg": -200}',
        '{"mount_pitch_deg": "30", "mount_roll_deg": 0}',
        '{"mount_yaw_deg": 10}',
        "{}",
        "not json",
    ],
)
def test_mount_endpoint_rejects_invalid(setup, body):
    link, client, _, saved = setup
    client.post("/api/mount", json={"mount_pitch_deg": 30, "mount_roll_deg": 5})
    response = client.post("/api/mount", data=body, content_type="application/json")
    assert response.status_code == 400
    assert "error" in response.get_json()
    assert (link.mount.pitch_deg, link.mount.roll_deg) == (30.0, 5.0)
    assert saved[-1]["mount_pitch_deg"] == 30.0


def test_settings_endpoint(setup):
    link, client, _, _ = setup
    assert client.post("/api/settings", json={"rangefinder": True, "hostname": "192.168.2.201"}).status_code == 200
    assert link.processor.rangefinder is True
    assert client.get("/api/settings").get_json()["hostname"] == "192.168.2.201"
    assert client.post("/api/settings", json={"message_type": "nope"}).status_code == 400


def test_page_and_service_served(setup):
    _, client, _, _ = setup
    assert b"Mounting" in client.get("/").data
    assert client.get("/register_service").get_json()["name"]


def test_connect_start_and_status(setup):
    link, client, sender, _ = setup
    client.post("/api/settings", json={"hostname": "192.168.2.201"})
    assert client.post("/api/connect").status_code == 200
    assert link.driver.tcp == ("192.168.2.201", 9000)
    assert client.post("/api/start").status_code == 200
    status = client.get("/api/status").get_json()
    assert status["running"] and status["device_rotation"] == {"ROTXY": 0.0, "ROTYZ": 0.0, "ROTXZ": 0.0}
    assert status["double_rotation_warning"] is False

    link.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4))
    assert client.get("/api/status").get_json()["counters"]["VISION_POSITION_DELTA"]["sent"] == 1
    assert len(sender.sent) == 1


def test_stream_only_uses_port_9002_and_sends_no_commands(setup):
    link, client, _, _ = setup
    client.post("/api/settings", json={"hostname": "192.168.2.201", "stream_only": True})
    client.post("/api/connect")
    assert link.driver.tcp == ("192.168.2.201", 9002)
    assert client.post("/api/start").status_code == 200
    assert link.driver.parser.thread_running
    assert link.device_rotation is None  # GETINST is not sent on the stream port


def test_double_rotation_warning():
    link = NucleusLink(
        dict(DEFAULTS, hostname="h", mount_pitch_deg=30.0),
        driver=FakeDriver(rotation_reply=(b"0.00,0.00,30.00\r\n", b"OK\r\n")),
        sender=FakeSender(),
        save=lambda s: None,
    )
    link.connect()
    link.start()
    assert link.get_status()["double_rotation_warning"] is True


def test_disabled_output_counts_skipped(setup):
    link, client, sender, _ = setup
    client.post("/api/settings", json={"enabled": False, "rangefinder": True})
    link.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4))
    link.handle_packet(altimeter(T0, 3.0))
    counters = link.get_status()["counters"]
    assert counters["VISION_POSITION_DELTA"]["skipped"] == 1
    assert counters["DISTANCE_SENSOR"]["skipped"] == 1
    assert sender.sent == []


def test_failed_send_counted():
    sender = FakeSender(ok=False)
    link = NucleusLink(dict(DEFAULTS), driver=FakeDriver(), sender=sender, save=lambda s: None)
    link.handle_packet(bottom_track(T0, 0.2, 0.1, 0.4))
    assert link.counters["VISION_POSITION_DELTA"]["failed"] == 1


@pytest.mark.parametrize(
    "reply, expected",
    [
        ([b"0.00,0.00,0.00\r\n", b"OK\r\n"], {"ROTXY": 0.0, "ROTYZ": 0.0, "ROTXZ": 0.0}),
        ([b"GETINST,ROTXY=10,ROTYZ=-5.5,ROTXZ=0\r\n", b"OK\r\n"], {"ROTXY": 10.0, "ROTYZ": -5.5, "ROTXZ": 0.0}),
        ([b"ERROR\r\n"], None),
        ([], None),
    ],
)
def test_parse_device_rotation(reply, expected):
    assert parse_device_rotation(reply) == expected


def test_status_is_valid_json_with_non_finite_values(setup):
    link, client, _, _ = setup
    link.handle_packet(bottom_track(T0, float("nan"), 0.1, 0.2, valid=False))
    body = client.get("/api/status").get_data(as_text=True)
    assert "NaN" not in body
    assert client.get("/api/status").get_json()["live"]["velocity_dvl"] == [None, 0.1, 0.2]
