"""Bus-id collision detection (no hardware needed: a fake transport replays frames)."""

from __future__ import annotations

from robstride_gui.bus import Motor, RobstrideBus
from robstride_gui.protocol import CommunicationType, Frame


class FakeTransport:
    """A Transport stub for hardware-free tests.

    Two modes:
    * ``replies=[...]`` - replay a fixed, ordered list of frames (``send`` is a
      no-op). Handy for single-id tests.
    * ``motors={can_id: [uid, ...]}`` - a responder model: each ping enqueues a
      reply per motor on that id, mirroring how a real bus only answers pings
      addressed to a present motor. Needed for multi-id inventory tests.

    ``recv`` pops the next queued frame and returns ``None`` once exhausted, so
    the bus's listen-window loop terminates on timeout.
    """

    name = "fake"

    def __init__(self, replies: list[Frame] | None = None,
                 motors: dict[int, list[bytes]] | None = None):
        self._pending: list[Frame] = list(replies or [])
        self._motors = motors or {}
        self._open = True

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def send(self, frame: Frame) -> None:
        if frame.comm_type == CommunicationType.GET_DEVICE_ID and self._motors:
            for uid in self._motors.get(frame.device_id, []):
                self._pending.append(_device_id_reply(frame.device_id, uid))

    def recv(self, timeout: float = 0.1):
        return self._pending.pop(0) if self._pending else None


def _device_id_reply(device_id: int, uid: bytes) -> Frame:
    """A GET_DEVICE_ID reply from a motor at ``device_id`` carrying a unique id."""
    return Frame(CommunicationType.GET_DEVICE_ID, device_id, device_id, uid)


def test_single_motor_is_not_a_collision():
    transport = FakeTransport([_device_id_reply(1, b"\x01" * 8)])
    bus = RobstrideBus(transport)
    assert bus.count_responders(1, timeout=0.02) == 1
    assert bus.find_collisions([1]) == []


def test_two_motors_on_same_id_is_a_collision():
    # Two distinct MCU ids answering the same CAN id 1.
    transport = FakeTransport([
        _device_id_reply(1, b"\xAA" * 8),
        _device_id_reply(1, b"\xBB" * 8),
    ])
    bus = RobstrideBus(transport)
    assert bus.count_responders(1, timeout=0.02) == 2


def test_duplicate_payloads_count_once():
    # The same motor echoing twice must not be mistaken for two motors.
    transport = FakeTransport([
        _device_id_reply(1, b"\xAA" * 8),
        _device_id_reply(1, b"\xAA" * 8),
    ])
    bus = RobstrideBus(transport)
    assert bus.count_responders(1, timeout=0.02) == 1


def test_find_collisions_flags_only_shared_ids():
    transport = FakeTransport([
        _device_id_reply(1, b"\xAA" * 8),   # id 1: two responders -> collision
        _device_id_reply(1, b"\xBB" * 8),
        _device_id_reply(2, b"\xCC" * 8),   # id 2: single responder -> ok
    ])
    bus = RobstrideBus(transport)
    assert bus.find_collisions([1, 2]) == [1]


def test_frames_for_other_ids_are_ignored():
    transport = FakeTransport([
        _device_id_reply(5, b"\xAA" * 8),   # stray reply for a different id
        _device_id_reply(1, b"\xBB" * 8),
    ])
    bus = RobstrideBus(transport)
    assert bus.count_responders(1, timeout=0.02) == 1


# --- identify / inventory --------------------------------------------------------


def test_identify_returns_unique_ids_in_arrival_order():
    transport = FakeTransport([
        _device_id_reply(1, b"\xAA" * 8),
        _device_id_reply(1, b"\xBB" * 8),
        _device_id_reply(1, b"\xAA" * 8),   # duplicate echo, ignored
    ])
    bus = RobstrideBus(transport)
    assert bus.identify(1, timeout=0.02) == [b"\xAA" * 8, b"\xBB" * 8]


def test_inventory_pairs_each_id_with_its_motor_ids():
    # Responder model: id 1 has two motors (collision), id 2 has one.
    transport = FakeTransport(motors={
        1: [b"\xAA" * 8, b"\xBB" * 8],
        2: [b"\xCC" * 8],
    })
    bus = RobstrideBus(transport)
    inv = bus.inventory(start=1, end=2, identify_timeout=0.02)
    assert inv == [(1, [b"\xAA" * 8, b"\xBB" * 8]), (2, [b"\xCC" * 8])]


# --- set_motor_id verification ---------------------------------------------------


def test_set_motor_id_follows_motor_to_new_id_when_change_takes_effect():
    # Motor now answers at the new id 7 -> set_motor_id reports and re-keys to 7.
    transport = FakeTransport(motors={7: [b"\x09" * 8]})
    bus = RobstrideBus(transport)
    bus.add_motor(Motor(device_id=0))
    live = bus.set_motor_id(0, 7)
    assert live == 7
    assert 7 in bus.motors and 0 not in bus.motors


def test_set_motor_id_reports_old_id_when_change_not_applied():
    # Motor still answers only at its old id 0 (new id needs a power cycle).
    transport = FakeTransport(motors={0: [b"\x09" * 8]})
    bus = RobstrideBus(transport)
    bus.add_motor(Motor(device_id=0))
    live = bus.set_motor_id(0, 7)
    assert live == 0
    assert 0 in bus.motors and 7 not in bus.motors


def test_set_motor_id_returns_none_when_motor_silent():
    # Motor answers at neither id (e.g. powered off mid-assign).
    transport = FakeTransport(motors={})
    bus = RobstrideBus(transport)
    bus.add_motor(Motor(device_id=0))
    live = bus.set_motor_id(0, 7)
    assert live is None
