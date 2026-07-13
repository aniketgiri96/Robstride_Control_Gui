"""Enabling one motor must not starve the others' firmware watchdogs.

The worker is single-threaded: a multi-step enable / mode-switch handshake for
one motor is drained in full before the service loop feeds any other motor. If
that window is longer than a bystander motor's CAN watchdog, the bystander
reverts to standby and goes limp - the field bug where M6 dropped to mode 0 the
instant M5 was enabled (steady VBUS, no fault bit: pure command starvation).

The fix interleaves keepalive hold frames to every *other* enabled motor between
the blocking steps of the handshake. These tests lock that in: during one
motor's enable, the already-holding motor keeps receiving hold frames, and each
one re-asserts that motor's *own* setpoint (not the motor being enabled).

No Qt loop and no real bus: a fake bus records the call order with device ids.
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

    def operation(self, device_id, position, velocity, kp, kd, torque_ff=0.0):
        self.calls.append(("operation", device_id, position))
        return MotorStatus(device_id=device_id, position=position,
                           velocity=0.0, torque=0.0, temperature=0.0)

    def enable(self, device_id):
        self.calls.append(("enable", device_id))
        return None


def _two_motor_worker(bystander_pos: float = 0.5):
    """M1 already enabled and holding at ``bystander_pos``; M2 idle."""
    worker = wk.ControlWorker()
    worker._bus = FakeBus()
    worker.motor_can_timeout_raw = 0  # skip watchdog write for a clean trace
    worker._targets[1] = wk.MotorTarget(mode=RunMode.POSITION_PP,
                                        position=bystander_pos, enabled=True)
    worker._targets[2] = wk.MotorTarget(mode=RunMode.POSITION_PP)
    return worker


def _index_of(calls, name, device_id):
    return next(i for i, c in enumerate(calls)
               if c[0] == name and c[1] == device_id)


def test_bystander_is_fed_before_the_new_motor_finishes_enabling():
    # The core regression: while M2 runs its enable handshake, M1 (already
    # holding) must keep getting hold frames - not go dark until M2 is done.
    worker = _two_motor_worker()
    bus = worker._bus

    worker._apply(wk.Enable(device_id=2))

    m2_enable = _index_of(bus.calls, "enable", 2)
    fed_before = [c for c in bus.calls[:m2_enable]
                  if c[0] == "set_position" and c[1] == 1]
    assert fed_before, \
        "bystander M1 received no hold frame during M2's enable - it would " \
        "starve its firmware watchdog and go limp"


def test_bystander_keepalive_holds_its_own_setpoint_not_the_new_motor():
    # Each keepalive must re-command M1's own position (0.5), so the frame
    # actually holds M1 where it is rather than being an empty ping.
    worker = _two_motor_worker(bystander_pos=0.5)
    bus = worker._bus

    worker._apply(wk.Enable(device_id=2))

    m1_holds = [c for c in bus.calls if c[0] == "set_position" and c[1] == 1]
    assert m1_holds, "no keepalive frame reached M1"
    assert all(math.isclose(c[2], 0.5) for c in m1_holds)


def test_enable_feeds_bystander_multiple_times_across_the_handshake():
    # The keepalive is interleaved at several breakpoints, so a single slow step
    # cannot starve the bystander for the whole (multi-step) handshake.
    worker = _two_motor_worker()
    bus = worker._bus

    worker._apply(wk.Enable(device_id=2))

    feeds = [c for c in bus.calls if c[0] == "set_position" and c[1] == 1]
    assert len(feeds) >= 3


def test_mode_switch_also_feeds_the_bystander():
    # The mode-switch re-enable is the exact handshake the field log caught
    # starving M6; it must feed the others too.
    worker = _two_motor_worker()
    worker._targets[2].enabled = True   # both enabled; switch M2's mode
    bus = worker._bus

    worker._apply(wk.SetMode(device_id=2, mode=RunMode.POSITION_PP))

    m2_enable = _index_of(bus.calls, "enable", 2)
    fed_before = [c for c in bus.calls[:m2_enable]
                  if c[0] == "set_position" and c[1] == 1]
    assert fed_before, "bystander M1 was not fed during M2's mode switch"


def test_solo_enable_needs_no_keepalive():
    # With no other enabled motor there is nothing to feed; the enable proceeds
    # normally and issues no frame to a non-existent bystander.
    worker = wk.ControlWorker()
    worker._bus = FakeBus()
    worker.motor_can_timeout_raw = 0
    worker._targets[1] = wk.MotorTarget(mode=RunMode.POSITION_PP)

    worker._apply(wk.Enable(device_id=1))

    # Every frame in the trace is addressed to the motor being enabled.
    assert all(len(c) < 2 or c[1] == 1 for c in worker._bus.calls)
