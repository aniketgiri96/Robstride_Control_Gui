"""Safe-enable: a motor must come up HOLDING its current position.

Without this, enabling a position-mode motor snaps it to the default 0.0 target
the instant it is enabled, driving attached hardware into whatever is next to
it. The worker reads the shaft's current position and pre-loads it as the hold
setpoint (and loc_ref) *before* sending the enable frame.

No Qt loop and no real bus: a tiny fake bus records the call order so we can
assert loc_ref is written before enable.
"""

from __future__ import annotations

import math

from robstride_gui import worker as wk
from robstride_gui.protocol import MotorStatus, ParameterType, RunMode


class FakeBus:
    def __init__(self, current_pos: float = 1.234):
        self.current_pos = current_pos
        self.calls: list[tuple] = []

    def set_run_mode(self, device_id, mode):
        self.calls.append(("set_run_mode", device_id, mode))

    def read_param(self, device_id, param):
        self.calls.append(("read_param", device_id, param))
        if param is ParameterType.MEASURED_POSITION:
            return self.current_pos
        return 0  # canTimeout ack etc.

    def poll_status(self, device_id):
        self.calls.append(("poll_status", device_id))
        return MotorStatus(device_id=device_id, position=self.current_pos,
                           velocity=0.0, torque=0.0, temperature=0.0)

    def write_param(self, device_id, param, value):
        self.calls.append(("write_param", device_id, param, value))
        return None

    def set_position(self, device_id, position_rad, velocity_limit=None):
        self.calls.append(("set_position", device_id, position_rad))
        return None

    def enable(self, device_id):
        self.calls.append(("enable", device_id))
        return None

    def _names(self):
        return [c[0] for c in self.calls]


def _make_worker(mode: int, current_pos: float = 1.234):
    worker = wk.ControlWorker()
    worker._bus = FakeBus(current_pos)
    worker.motor_can_timeout_raw = 0  # skip the watchdog write for a clean trace
    worker._targets[1] = wk.MotorTarget(mode=mode)
    return worker


def test_enable_seeds_current_position_as_setpoint():
    worker = _make_worker(RunMode.POSITION_PP, current_pos=1.5)

    worker._apply(wk.Enable(device_id=1))

    # The held target is the shaft's current position, not the default 0.0.
    assert math.isclose(worker._targets[1].position, 1.5)
    assert math.isclose(worker._last_raw_pos[1], 1.5)


def test_enable_writes_loc_ref_before_enabling():
    worker = _make_worker(RunMode.POSITION_PP)
    bus = worker._bus

    worker._apply(wk.Enable(device_id=1))

    names = bus._names()
    assert "set_position" in names and "enable" in names
    # loc_ref must be pre-loaded BEFORE the enable frame, or the motor jumps.
    assert names.index("set_position") < names.index("enable")


def test_enable_reasserts_loc_ref_after_enabling():
    # RobStride ignores a loc_ref write while the motor is disabled, so the
    # pre-enable seed alone lets the motor keep its stale internal target and
    # jump there on enable. loc_ref must be written AGAIN after the enable frame.
    worker = _make_worker(RunMode.POSITION_PP, current_pos=0.0)
    bus = worker._bus

    worker._apply(wk.Enable(device_id=1))

    names = bus._names()
    # There is a post-enable loc_ref write: the last set_position follows enable.
    last_set_pos = max(i for i, n in enumerate(names) if n == "set_position")
    assert last_set_pos > names.index("enable")


def test_enable_after_zero_holds_new_zero_not_stale_setpoint():
    # Reproduces the field bug: motor was swinging at ~45 deg, disabled (which
    # freezes the setpoint at 45), then zeroed at a new spot so poll_status now
    # reads ~0. Enable must hold ~0, NOT swing back to the stale 45.
    bus = FakeBus(current_pos=0.0)          # post-zero reading in the live frame
    worker = _make_worker(RunMode.POSITION_PP)
    worker._bus = bus
    worker._targets[1].position = math.radians(45)  # stale hold setpoint

    worker._apply(wk.Enable(device_id=1))

    assert abs(worker._targets[1].position) < 1e-6   # seeded to current, not 45
    # loc_ref is pre-loaded with the current (0) position, not the stale one.
    assert ("set_position", 1, 0.0) in bus.calls


def test_enable_seed_respects_calibration_offset():
    worker = _make_worker(RunMode.POSITION_PP, current_pos=2.0)
    # user = (raw - offset) * direction -> (2.0 - 0.5) * 1 = 1.5
    worker._calib[1] = wk.Calibration(direction=1, offset=0.5)

    worker._apply(wk.Enable(device_id=1))

    assert math.isclose(worker._targets[1].position, 1.5)


def test_velocity_mode_enable_does_not_seed_position():
    worker = _make_worker(RunMode.VELOCITY)
    bus = worker._bus

    worker._apply(wk.Enable(device_id=1))

    # No position read or loc_ref write for a velocity-mode enable.
    assert "poll_status" not in bus._names()
    assert "set_position" not in bus._names()


def test_failed_position_read_still_enables_but_warns():
    worker = _make_worker(RunMode.POSITION_PP)
    worker._bus.poll_status = lambda device_id: None       # operation-frame poll fails
    worker._bus.read_param = lambda device_id, param: None  # and the mechPos read too
    errors: list[str] = []
    worker.error.connect(errors.append)

    worker._apply(wk.Enable(device_id=1))

    assert worker._targets[1].enabled is True   # still enables
    assert errors and "could not read position" in errors[0]


def test_position_read_falls_back_to_param_when_poll_fails():
    # A disabled or fault-latched motor can decline the operation-frame poll while
    # still answering a plain parameter read. The seed must fall back to mechPos
    # and NOT cry "check the connection" for a reachable motor.
    worker = _make_worker(RunMode.POSITION_PP, current_pos=0.7)
    worker._bus.poll_status = lambda device_id: None  # only the poll fails
    errors: list[str] = []
    worker.error.connect(errors.append)

    worker._apply(wk.Enable(device_id=1))

    assert not errors                                       # reachable => no false alarm
    assert math.isclose(worker._targets[1].position, 0.7)   # seeded from mechPos read
    assert math.isclose(worker._last_raw_pos[1], 0.7)
