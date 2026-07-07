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

    def poll_status(self, device_id: int):
        return None  # no live frame in this stub; readout refresh is skipped


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


def test_set_zero_clears_software_offset_but_keeps_direction():
    bus = FakeBus({"zero_sta": 1, "mech_offset": 0.0})
    worker = _worker(bus)
    # A software offset + inverted direction were in effect before zeroing.
    worker._calib[5] = wk.Calibration(direction=-1, offset=1.234)
    changes: list[tuple[int, int, float]] = []
    worker.calibrationChanged.connect(
        lambda did, d, off: changes.append((did, d, off)))

    worker._apply(wk.SetZero(5))

    # Hardware zero remade the frame: the offset must be cleared so the readout
    # reads ~0, while the invert (direction) is preserved.
    assert worker._calib[5].offset == 0.0
    assert worker._calib[5].direction == -1
    assert changes == [(5, -1, 0.0)]


class RecordingBus:
    """Fake bus that records the call order across the set-zero flow.

    Covers the enabled path too: disable/enable/set_run_mode/poll_status/
    set_position are the calls ``_set_zero`` -> ``_disable``/``_enable`` make.
    """

    def __init__(self, current_pos: float = 0.0):
        self.current_pos = current_pos
        self.calls: list[str] = []

    def disable(self, device_id):
        self.calls.append("disable")

    def set_zero(self, device_id):
        self.calls.append("set_zero")

    def read_zero_state(self, device_id):
        self.calls.append("read_zero_state")
        return {"zero_sta": 1, "mech_offset": 0.0}

    def poll_status(self, device_id):
        self.calls.append("poll_status")
        from robstride_gui.protocol import MotorStatus
        return MotorStatus(device_id=device_id, position=self.current_pos,
                           velocity=0.0, torque=0.0, temperature=0.0)

    def set_run_mode(self, device_id, mode):
        self.calls.append("set_run_mode")

    def set_position(self, device_id, position_rad, velocity_limit=None):
        self.calls.append("set_position")
        return None

    def enable(self, device_id):
        self.calls.append("enable")
        return None


def test_set_zero_while_enabled_disables_zeros_then_reenables():
    """A Set Zero on an enabled position-mode motor must bracket the zero with a
    disable/re-enable so the shaft does not twitch and the readout snaps to 0."""
    from robstride_gui.protocol import RunMode

    bus = RecordingBus()
    worker = _worker(bus)
    worker.motor_can_timeout_ms = 0  # skip the watchdog write for a clean trace
    worker._targets[3] = wk.MotorTarget(mode=RunMode.POSITION_PP, enabled=True)
    worker._targets[3].position = 1.5  # a stale hold setpoint from before zeroing

    worker._apply(wk.SetZero(3))

    # Disable happens before the hardware zero, and enable after it: the motor
    # is stopped while its mechanical frame is redefined.
    assert bus.calls.index("disable") < bus.calls.index("set_zero")
    assert bus.calls.index("set_zero") < bus.calls.index("enable")
    # It ends up running again, holding the new zero (not the stale 1.5 rad).
    assert worker._targets[3].enabled is True
    assert worker._targets[3].position == 0.0
    assert worker._calib[3].offset == 0.0


def test_set_zero_while_disabled_does_not_touch_enable():
    """A Set Zero on an already-disabled motor is a plain zero: no disable/enable
    bracketing, just the zero, readout refresh, and read-back."""
    bus = RecordingBus()
    worker = _worker(bus)
    worker._targets[3] = wk.MotorTarget(enabled=False)

    worker._apply(wk.SetZero(3))

    assert "disable" not in bus.calls
    assert "enable" not in bus.calls
    assert "set_zero" in bus.calls
    assert worker._targets[3].enabled is False
