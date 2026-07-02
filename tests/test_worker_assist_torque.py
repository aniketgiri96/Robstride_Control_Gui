"""Assist / feed-forward torque flows through SetTarget into the MIT frame.

No hardware, no Qt loop: a fake bus captures the operation() arguments and we
assert the worker forwards the (safety-clamped) torque_ff a backdrive assist
slider would emit.
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import RunMode


class FakeBus:
    def __init__(self):
        self.operation_calls: list[tuple] = []

    def operation(self, device_id, position, velocity, kp, kd, torque_ff):
        self.operation_calls.append(
            (device_id, position, velocity, kp, kd, torque_ff))
        return None


def _mit_worker(bus: FakeBus) -> wk.ControlWorker:
    worker = wk.ControlWorker()
    worker._bus = bus
    return worker


def test_set_target_updates_torque_ff():
    worker = _mit_worker(FakeBus())

    worker._set_target(wk.SetTarget(device_id=4, torque_ff=1.5))

    assert worker._targets[4].torque_ff == 1.5


def test_mit_command_forwards_assist_torque():
    bus = FakeBus()
    worker = _mit_worker(bus)
    target = wk.MotorTarget(mode=RunMode.MIT, kp=0.0, kd=0.0, torque_ff=1.0)

    worker._command_motor(4, target)

    assert len(bus.operation_calls) == 1
    _, _, _, kp, kd, torque_ff = bus.operation_calls[0]
    assert kp == 0.0 and kd == 0.0
    assert torque_ff == 1.0  # within the default torque cap -> passed through


def test_mit_command_clamps_assist_torque_to_safety_cap():
    bus = FakeBus()
    worker = _mit_worker(bus)
    cap = worker._safety.limits.torque_max
    target = wk.MotorTarget(mode=RunMode.MIT, torque_ff=cap + 100.0)

    worker._command_motor(4, target)

    _, _, _, _, _, torque_ff = bus.operation_calls[0]
    assert torque_ff == cap  # clamped, never exceeds the configured torque cap
