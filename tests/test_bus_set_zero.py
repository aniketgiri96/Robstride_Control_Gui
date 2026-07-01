"""Set-zero persists to flash, and the motor's zero can be read back.

Hardware-free: a recording transport captures the frames the bus sends and can
replay canned READ_PARAMETER responses.
"""

from __future__ import annotations

import struct

from robstride_gui.bus import RobstrideBus
from robstride_gui.protocol import (
    DEFAULT_HOST_ID, CommunicationType, Frame, Param, ParameterType,
)


class RecordingTransport:
    """Records every sent frame and replays a FIFO queue of reply frames."""

    name = "fake"

    def __init__(self):
        self.sent: list[Frame] = []
        self._replies: list[Frame] = []
        self._open = True

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def queue(self, frame: Frame) -> None:
        self._replies.append(frame)

    def send(self, frame: Frame) -> None:
        self.sent.append(frame)

    def recv(self, timeout: float = 0.1):
        return self._replies.pop(0) if self._replies else None


def _param_reply(device_id: int, param: Param, value) -> Frame:
    """A READ_PARAMETER response: 2-byte index + 2 pad + value bytes (data[4:])."""
    value_buf = struct.pack("<" + param.fmt,
                            float(value) if param.fmt == "f" else int(value))
    data = struct.pack("<HH", param.index, 0) + value_buf
    return Frame(CommunicationType.READ_PARAMETER, DEFAULT_HOST_ID, device_id, data)


def test_set_zero_sends_set_zero_then_save():
    t = RecordingTransport()
    bus = RobstrideBus(t)

    bus.set_zero(3)

    types = [f.comm_type for f in t.sent]
    assert CommunicationType.SET_ZERO_POSITION in types
    assert CommunicationType.SAVE_PARAMETERS in types
    # The save must come *after* the set-zero, or it would persist the old zero.
    assert (types.index(CommunicationType.SET_ZERO_POSITION)
            < types.index(CommunicationType.SAVE_PARAMETERS))
    save = next(f for f in t.sent if f.comm_type == CommunicationType.SAVE_PARAMETERS)
    assert save.device_id == 3  # persisted at the same motor


def test_read_zero_state_returns_markers():
    t = RecordingTransport()
    t.queue(_param_reply(2, ParameterType.ZERO_STATE, 1))
    t.queue(_param_reply(2, ParameterType.MECHANICAL_OFFSET, 0.25))
    bus = RobstrideBus(t)

    info = bus.read_zero_state(2)

    assert info is not None
    assert info["zero_sta"] == 1
    assert abs(info["mech_offset"] - 0.25) < 1e-6


def test_read_zero_state_none_when_motor_silent():
    bus = RobstrideBus(RecordingTransport())  # no replies queued
    assert bus.read_zero_state(2) is None
