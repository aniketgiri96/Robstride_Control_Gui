"""Safety guards for motor operation (senior-dev edge-case review).

Covers four independent guards, no Qt loop and no hardware:

* C1 - feedback range cutout: disable the motor if live position leaves the
  calibrated range (the only protection in velocity/current modes).
* C2 - safe re-enable on mode switch: changing mode on an enabled motor must
  seed the current position, not snap to a stale setpoint.
* H1 - E-stop blocks enable: no energising while E-stop is latched.
* H2 - sweep stops on disable / E-stop so a later enable holds position.
"""

from __future__ import annotations

import math

from robstride_gui import worker as wk
from robstride_gui.protocol import MotorStatus, ParameterType, RunMode


class FakeBus:
    def __init__(self, current_pos: float = 1.5):
        self.current_pos = current_pos
        self.calls: list[tuple] = []

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

    def set_position(self, device_id, position_rad, velocity_limit=None):
        self.calls.append(("set_position", device_id, position_rad))
        return None

    def enable(self, device_id):
        self.calls.append(("enable", device_id))
        return None

    def disable(self, device_id):
        self.calls.append(("disable", device_id))
        return None

    def names(self):
        return [c[0] for c in self.calls]


def _worker(bus=None):
    worker = wk.ControlWorker()
    worker._bus = bus or FakeBus()
    worker.motor_can_timeout_ms = 0
    return worker


# -- H1: E-stop blocks enable -------------------------------------------------


def test_enable_refused_while_estop_engaged():
    worker = _worker()
    worker._safety.engage_estop()
    worker._targets[1] = wk.MotorTarget(mode=RunMode.POSITION_PP)
    errors: list[str] = []
    worker.error.connect(errors.append)

    worker._apply(wk.Enable(device_id=1))

    assert "enable" not in worker._bus.names()      # never energised
    assert worker._targets[1].enabled is False
    assert errors and "E-STOP" in errors[0]


# -- C2: safe re-enable on mode switch ---------------------------------------


def test_mode_switch_on_enabled_motor_seeds_current_position():
    bus = FakeBus(current_pos=1.5)
    worker = _worker(bus)
    worker._targets[1] = wk.MotorTarget(mode=RunMode.VELOCITY, enabled=True)

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.POSITION_PP))

    assert math.isclose(worker._targets[1].position, 1.5)   # holds current spot
    names = bus.names()
    # loc_ref pre-loaded before the re-enable, or the motor jumps.
    assert names.index("set_position") < names.index("enable")


def test_mode_switch_on_disabled_motor_does_not_enable():
    bus = FakeBus()
    worker = _worker(bus)
    worker._targets[1] = wk.MotorTarget(mode=RunMode.VELOCITY, enabled=False)

    worker._apply(wk.SetMode(device_id=1, mode=RunMode.POSITION_PP))

    assert "enable" not in bus.names()


# -- H2: sweep stops on disable / E-stop -------------------------------------


def test_disable_stops_running_sweep():
    worker = _worker()
    worker._targets[1] = wk.MotorTarget(enabled=True, sweep_enabled=True)
    stopped: list[int] = []
    worker.sweepStopped.connect(stopped.append)

    worker._apply(wk.Disable(device_id=1))

    assert worker._targets[1].sweep_enabled is False
    assert stopped == [1]


def test_estop_stops_running_sweep():
    worker = _worker()
    worker._targets[1] = wk.MotorTarget(enabled=True, sweep_enabled=True)
    stopped: list[int] = []
    worker.sweepStopped.connect(stopped.append)

    worker._apply(wk.EStop(engage=True))

    assert worker._targets[1].sweep_enabled is False
    assert worker._targets[1].enabled is False
    assert stopped == [1]


# -- C1: feedback range cutout ------------------------------------------------


def test_range_cutout_disables_when_feedback_leaves_range():
    worker = _worker()
    target = wk.MotorTarget(enabled=True)
    worker._targets[1] = target
    worker._range[1] = (-1.0, 1.0)
    errors: list[str] = []
    worker.error.connect(errors.append)

    # Well past the high bound + margin.
    worker._enforce_range_cutout(1, target, 1.5)

    assert target.enabled is False
    assert "disable" in worker._bus.names()
    assert errors and "calibrated range" in errors[0]


def test_range_cutout_ignores_position_inside_range():
    worker = _worker()
    target = wk.MotorTarget(enabled=True)
    worker._targets[1] = target
    worker._range[1] = (-1.0, 1.0)

    worker._enforce_range_cutout(1, target, 0.5)

    assert target.enabled is True
    assert "disable" not in worker._bus.names()


def test_range_cutout_skipped_during_range_calibration():
    worker = _worker()
    target = wk.MotorTarget(enabled=True)
    worker._targets[1] = target
    worker._range[1] = (-1.0, 1.0)
    worker._range_cal[1] = {"min": None, "max": None}  # calibration in progress

    worker._enforce_range_cutout(1, target, 5.0)  # far outside, but calibrating

    assert target.enabled is True  # not tripped: we are redefining the range
