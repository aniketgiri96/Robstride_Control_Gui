"""Mode switch on an already-enabled motor must re-assert the hold setpoint.

Switching run-mode brackets a disable/enable pulse. RobStride ignores a loc_ref
(POSITION_TARGET) write while the motor is disabled, so seeding the hold before
the enable frame does not latch - the position loop comes up with no valid hold
and the motor sits limp (zero torque) and drifts. This reproduces the field bug
"the motor dropped its hold after a mode switch" and locks in the fix: a
post-enable loc_ref write, exactly as a plain Enable does.

No Qt loop and no real bus: a fake bus records the call order.
"""

from __future__ import annotations

import math

from robstride_gui import worker as wk
from robstride_gui.protocol import MotorStatus, ParameterType, RunMode


class FakeBus:
    def __init__(self, current_pos: float = 1.0):
        self.current_pos = current_pos
        self.calls: list[tuple] = []

    def disable(self, device_id):
        self.calls.append(("disable", device_id))
        return None

    def set_run_mode(self, device_id, mode):
        self.calls.append(("set_run_mode", device_id, mode))

    def read_param(self, device_id, param):
        self.calls.append(("read_param", device_id, param))
        if param is ParameterType.MEASURED_POSITION:
            return self.current_pos
        return 0

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


def _enabled_worker(mode: int = RunMode.POSITION_PP, current_pos: float = 1.0):
    worker = wk.ControlWorker()
    worker._bus = FakeBus(current_pos)
    worker.motor_can_timeout_raw = 0  # skip watchdog write for a clean trace
    worker._targets[1] = wk.MotorTarget(mode=mode, enabled=True)
    return worker


def test_mode_switch_reasserts_loc_ref_after_enable():
    # The regression: without the post-enable loc_ref write, the motor re-enables
    # with no latched hold and goes limp. There must be a set_position AFTER enable.
    worker = _enabled_worker(RunMode.POSITION_PP)
    bus = worker._bus

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.POSITION_PP))

    names = bus._names()
    assert "enable" in names and "set_position" in names
    last_set_pos = max(i for i, n in enumerate(names) if n == "set_position")
    assert last_set_pos > names.index("enable"), \
        "loc_ref must be re-written AFTER the enable frame, or the motor sits limp"


def test_mode_switch_still_seeds_before_enable_too():
    # Belt-and-suspenders: the pre-enable seed is kept, so loc_ref is written both
    # before AND after enable (the after-write is the one the motor actually acts on).
    worker = _enabled_worker(RunMode.POSITION_PP)
    bus = worker._bus

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.POSITION_PP))

    names = bus._names()
    first_set_pos = names.index("set_position")
    assert first_set_pos < names.index("enable")


def test_mode_switch_holds_current_position_not_stale_target():
    # Motor was holding at 45 deg, disabled (freezing the stale target), then the
    # live shaft sits at ~0. A mode switch must re-seed to the current 0, not swing
    # back to the stale 45.
    worker = _enabled_worker(RunMode.POSITION_PP, current_pos=0.0)
    worker._targets[1].position = math.radians(45)  # stale
    bus = worker._bus

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.POSITION_PP))

    assert abs(worker._targets[1].position) < 1e-6
    # The post-enable re-assert writes the current (0.0) raw position.
    post_enable = bus.calls[bus._names().index("enable"):]
    assert ("set_position", 1, 0.0) in post_enable


def test_switch_to_velocity_mode_has_no_position_reassert():
    # Velocity mode holds no position; there must be no loc_ref write at all.
    worker = _enabled_worker(RunMode.POSITION_PP)
    bus = worker._bus

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.VELOCITY))

    assert "set_position" not in bus._names()
    assert worker._targets[1].mode == RunMode.VELOCITY


def test_mode_switch_on_disabled_motor_does_not_enable():
    # Switching mode on a disabled motor changes mode only - it must not energise.
    worker = _enabled_worker(RunMode.POSITION_PP)
    worker._targets[1].enabled = False
    bus = worker._bus

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.POSITION_PP))

    assert "enable" not in bus._names()
