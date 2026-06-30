"""AddMotor registers an after-Connect motor so it is polled and driveable.

No hardware and no Qt event loop: the worker's command handlers are called
directly via ``_apply`` with a fake transport.
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import CommunicationType, Frame


class FakeTransport:
    """Minimal Transport stub: open/close flags, send/recv no-ops."""

    name = "fake"

    def __init__(self):
        self._open = False

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def send(self, frame: Frame) -> None:  # pragma: no cover - not exercised here
        pass

    def recv(self, timeout: float = 0.1):
        return None


def _connected_worker() -> wk.ControlWorker:
    worker = wk.ControlWorker()
    worker._apply(wk.Connect(transport=FakeTransport(), motors=[]))
    return worker


def test_add_motor_registers_on_bus_and_targets():
    worker = _connected_worker()

    worker._apply(wk.AddMotor(device_id=3, model="rs-04"))

    assert 3 in worker._bus.motors
    assert 3 in worker._targets


def test_add_motor_is_idempotent():
    worker = _connected_worker()

    worker._apply(wk.AddMotor(device_id=5))
    existing_target = worker._targets[5]
    worker._apply(wk.AddMotor(device_id=5))

    # Re-registering keeps the same target object (setpoints survive a re-Detect).
    assert worker._targets[5] is existing_target


def test_add_motor_without_connection_is_noop():
    worker = wk.ControlWorker()  # never connected -> no bus

    worker._apply(wk.AddMotor(device_id=2))

    assert worker._bus is None
    assert 2 not in worker._targets
