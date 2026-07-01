"""Worker reads the motor's persisted zero back and emits it for display.

No Qt event loop needed: signals connected directly fire synchronously on emit.
"""

from __future__ import annotations

from robstride_gui import worker as wk


class FakeBus:
    """Minimal bus stub for set-zero / read-zero-state paths."""

    def __init__(self, zero_state: dict | None):
        self._zero_state = zero_state
        self.set_zero_calls: list[int] = []

    def set_zero(self, device_id: int) -> None:
        self.set_zero_calls.append(device_id)

    def read_zero_state(self, device_id: int) -> dict | None:
        return self._zero_state


def _worker(bus: FakeBus) -> wk.ControlWorker:
    worker = wk.ControlWorker()
    worker._bus = bus
    return worker


def test_read_zero_state_emits_info():
    worker = _worker(FakeBus({"zero_sta": 1, "mech_offset": 0.5}))
    captured: list[tuple[int, wk.ZeroStateInfo]] = []
    worker.zeroStateUpdated.connect(lambda did, info: captured.append((did, info)))

    worker._read_zero_state(7)

    assert len(captured) == 1
    device_id, info = captured[0]
    assert device_id == 7
    assert info.zero_sta == 1
    assert info.mech_offset == 0.5


def test_read_zero_state_silent_when_unavailable():
    worker = _worker(FakeBus(None))  # motor did not answer
    captured: list = []
    worker.zeroStateUpdated.connect(lambda did, info: captured.append(info))

    worker._read_zero_state(7)

    assert captured == []


def test_set_zero_command_saves_then_reads_back():
    bus = FakeBus({"zero_sta": 1, "mech_offset": 0.0})
    worker = _worker(bus)
    captured: list[int] = []
    worker.zeroStateUpdated.connect(lambda did, info: captured.append(did))

    worker._apply(wk.SetZero(5))

    assert bus.set_zero_calls == [5]      # hardware zero set (which also saves to flash)
    assert captured == [5]                # and the motor's zero was read back for the UI
