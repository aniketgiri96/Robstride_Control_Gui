"""MuJoCo <-> motor bridge: mapping, throttle, feedback, and the UDP link.

No hardware and no MuJoCo: a fake worker captures posted commands and feeds
synthetic feedback, and the UDP link is exercised over a real localhost socket.
"""

from __future__ import annotations

import json
import socket
import time

import pytest
from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from robstride_gui import worker as wk
from robstride_gui.protocol import MotorStatus, RunMode
from robstride_gui.sim import MujocoBridge, UdpTargetLink
from robstride_gui.ui.sim_dock import SimBridgeDock


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


class FakeWorker(QObject):
    """Minimal stand-in for ControlWorker: records posts, emits feedback."""

    statusUpdated = Signal(int, object)

    def __init__(self):
        super().__init__()
        self.posted: list = []

    def post(self, command) -> None:
        self.posted.append(command)


def _status(device_id: int, position: float) -> MotorStatus:
    return MotorStatus(device_id=device_id, position=position, velocity=0.1,
                       torque=0.2, temperature=30.0)


# -- sim -> motor ---------------------------------------------------------------

def test_push_targets_maps_labels_and_skips_unmapped_and_nonfinite(app):
    # Arrange
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"revolute_1": 1, "j2": 2}, max_rate_hz=0)

    # Act
    sent = bridge.push_targets({"revolute_1": 0.5, "j2": -0.3,
                                "not_mapped": 1.0, "j2_nan": float("nan")})

    # Assert
    assert sent == 2
    posts = {c.device_id: c for c in worker.posted}
    assert isinstance(posts[1], wk.SetTarget) and posts[1].position == pytest.approx(0.5)
    assert posts[2].position == pytest.approx(-0.3)
    assert set(posts) == {1, 2}


def test_push_targets_is_throttled(app):
    # Arrange: 100 Hz -> a second call inside 10 ms is dropped.
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"revolute_1": 1}, max_rate_hz=100.0)

    # Act
    first = bridge.push_targets({"revolute_1": 0.1})
    second = bridge.push_targets({"revolute_1": 0.2})

    # Assert
    assert first == 1 and second == 0
    assert len(worker.posted) == 1


def test_mit_mode_forwards_gains_and_feedforward_torque(app):
    # Arrange
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"revolute_1": 1}, mode=RunMode.MIT,
                          max_rate_hz=0, send_kp=12.0, send_kd=0.8)

    # Act
    bridge.push_targets({"revolute_1": 0.4}, torques_ff={"revolute_1": 0.25})

    # Assert
    cmd = worker.posted[0]
    assert cmd.kp == pytest.approx(12.0)
    assert cmd.kd == pytest.approx(0.8)
    assert cmd.torque_ff == pytest.approx(0.25)


# -- motor -> sim ---------------------------------------------------------------

def test_feedback_is_cached_by_joint_label(app):
    # Arrange
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"revolute_1": 1, "j2": 2}, max_rate_hz=0)

    # Act
    worker.statusUpdated.emit(1, _status(1, 0.5))
    worker.statusUpdated.emit(9, _status(9, 1.0))  # unmapped id: ignored

    # Assert
    state = bridge.latest_state()
    assert set(state) == {"revolute_1"}
    assert state["revolute_1"].position == pytest.approx(0.5)
    payload = bridge.state_payload()
    assert payload["revolute_1"]["id"] == 1
    assert payload["revolute_1"]["pos"] == pytest.approx(0.5)


def test_set_joint_map_rebuilds_forward_and_reverse_maps(app):
    # Arrange
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"old": 1}, max_rate_hz=0)

    # Act
    bridge.set_joint_map({"hip": 3})
    bridge.push_targets({"hip": 0.2, "old": 0.9})
    worker.statusUpdated.emit(3, _status(3, 0.2))

    # Assert
    assert bridge.joint_map == {"hip": 3}
    assert [c.device_id for c in worker.posted] == [3]     # "old" no longer mapped
    assert set(bridge.latest_state()) == {"hip"}


# -- UDP link -------------------------------------------------------------------

def test_udp_link_round_trip_commands_and_returns_state(app):
    # Arrange: bind an ephemeral port so the test never collides.
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"revolute_1": 1}, max_rate_hz=0)
    worker.statusUpdated.emit(1, _status(1, 0.5))
    link = UdpTargetLink(bridge, host="127.0.0.1", port=0)
    link.start()
    port = link._sock.getsockname()[1]
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(2.0)
    try:
        # Act
        client.sendto(json.dumps({"targets": {"revolute_1": 0.75}}).encode(),
                      ("127.0.0.1", port))
        reply, _ = client.recvfrom(65535)

        # Assert
        state = json.loads(reply.decode())["state"]
        assert state["revolute_1"]["pos"] == pytest.approx(0.5)
        deadline = time.monotonic() + 1.0
        while not worker.posted and time.monotonic() < deadline:
            time.sleep(0.01)
        assert worker.posted[0].position == pytest.approx(0.75)
    finally:
        client.close()
        link.stop()


def test_refresh_joint_map_is_noop_while_link_stopped(app):
    # Arrange: dock created but never started (no UDP link).
    worker = FakeWorker()
    dock = SimBridgeDock(worker, lambda: {"revolute_1": 1}, port=8651)

    # Act / Assert: refresh must not build a bridge or raise while stopped.
    dock.refresh_joint_map()
    assert dock._bridge is None
    dock.shutdown()


def test_refresh_joint_map_picks_up_motor_added_while_streaming(app):
    # Arrange: link started with one motor mapped.
    worker = FakeWorker()
    motors = {"revolute_1": 1}
    dock = SimBridgeDock(worker, lambda: dict(motors), port=8652)
    dock._check.setChecked(True)   # emits toggled -> _start(), snapshots the map
    try:
        assert dock._bridge.joint_map == {"revolute_1": 1}

        # Act: a motor connects after the bridge started, then the window refreshes.
        motors["revolute_2"] = 2
        dock.refresh_joint_map()

        # Assert: the new motor is now in the live joint map without a re-toggle.
        assert dock._bridge.joint_map == {"revolute_1": 1, "revolute_2": 2}
    finally:
        dock.shutdown()


def test_udp_link_survives_a_malformed_datagram(app):
    # Arrange
    worker = FakeWorker()
    bridge = MujocoBridge(worker, {"revolute_1": 1}, max_rate_hz=0)
    link = UdpTargetLink(bridge, host="127.0.0.1", port=0)
    link.start()
    port = link._sock.getsockname()[1]
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(2.0)
    try:
        # Act: garbage first, then a valid request must still get a reply.
        client.sendto(b"not json at all", ("127.0.0.1", port))
        time.sleep(0.05)
        client.sendto(json.dumps({"targets": {}}).encode(), ("127.0.0.1", port))

        # Assert
        reply, _ = client.recvfrom(65535)
        assert "state" in json.loads(reply.decode())
    finally:
        client.close()
        link.stop()
