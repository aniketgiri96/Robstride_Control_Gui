"""Communication watchdog: the worker must not keep driving a dead bus.

Two sides, no hardware needed:

* Host side - consecutive :class:`TransportError` failures in the control loop
  must (a) surface ONE error, not one per 100 Hz cycle, and (b) tear the
  connection down once :data:`worker.COMM_FAILURE_LIMIT` is reached, instead of
  spinning on a dead adapter while enabled motors run unsupervised.
* Motor side - Enable must arm the motor's own ``canTimeout`` register so the
  motor stops itself if the *host* dies (a host-side watchdog cannot help with
  that failure).
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import ParameterType
from robstride_gui.transport import TransportError


class DeadBus:
    """Bus whose every setpoint write fails at the transport level."""

    def __init__(self):
        self.closed = False
        self.disable_calls: list[int] = []

    def set_position(self, device_id, position, velocity_limit=None):
        raise TransportError("serial write failed: device gone")

    def disable(self, device_id):
        self.disable_calls.append(device_id)

    def close(self):
        self.closed = True

    def read_param(self, device_id, param):
        return None  # power registers silent; _read_power emits nothing


class FlakyBus(DeadBus):
    """Fails ``fail_first`` times, then starts answering (with no status)."""

    def __init__(self, fail_first: int):
        super().__init__()
        self._remaining_failures = fail_first

    def set_position(self, device_id, position, velocity_limit=None):
        if self._remaining_failures > 0:
            self._remaining_failures -= 1
            raise TransportError("serial write failed: transient")
        return None


def _worker_with_enabled_motor(bus) -> wk.ControlWorker:
    worker = wk.ControlWorker()
    worker._bus = bus
    worker._targets[4] = wk.MotorTarget(enabled=True)
    return worker


def _service_n_times(worker: wk.ControlWorker, n: int) -> None:
    for _ in range(n):
        worker._service_motors()


# -- host-side watchdog -------------------------------------------------------


def test_transport_failures_below_limit_emit_single_error():
    # Arrange
    worker = _worker_with_enabled_motor(DeadBus())
    errors: list[str] = []
    worker.error.connect(errors.append)

    # Act: several failing cycles, but fewer than the disconnect threshold
    _service_n_times(worker, wk.COMM_FAILURE_LIMIT - 1)

    # Assert: one surfaced error, not one per cycle, and still connected
    assert len(errors) == 1
    assert worker._bus is not None


def test_reaching_failure_limit_tears_down_connection():
    # Arrange
    bus = DeadBus()
    worker = _worker_with_enabled_motor(bus)
    states: list[bool] = []
    worker.connectionChanged.connect(states.append)

    # Act
    _service_n_times(worker, wk.COMM_FAILURE_LIMIT)

    # Assert: bus closed and dropped, disconnect signalled, motor marked off
    assert bus.closed
    assert worker._bus is None
    assert states == [False]
    assert worker._targets[4].enabled is False


def test_teardown_attempts_to_disable_enabled_motors():
    bus = DeadBus()
    worker = _worker_with_enabled_motor(bus)

    _service_n_times(worker, wk.COMM_FAILURE_LIMIT)

    # Best-effort stop was at least attempted on the way down.
    assert bus.disable_calls == [4]


def test_successful_cycle_resets_failure_counter():
    # Arrange: fails a few times, then recovers
    worker = _worker_with_enabled_motor(FlakyBus(fail_first=3))

    # Act: 3 failing cycles then good ones - never hits the limit
    _service_n_times(worker, wk.COMM_FAILURE_LIMIT * 3)

    # Assert: recovery reset the counter, so the connection survived
    assert worker._bus is not None
    assert worker._comm_failures == 0


def test_read_power_failures_count_toward_watchdog():
    class PowerDeadBus(DeadBus):
        def read_param(self, device_id, param):
            raise TransportError("serial read failed: device gone")

    worker = _worker_with_enabled_motor(PowerDeadBus())

    before = worker._comm_failures
    worker._read_power(4)

    assert worker._comm_failures == before + 1


# -- motor-side watchdog ------------------------------------------------------


class RecordingBus:
    """Records the enable sequence so the test can assert the watchdog write."""

    def __init__(self):
        self.param_writes: list[tuple[int, int, object]] = []
        self.enabled: list[int] = []

    def set_run_mode(self, device_id, mode):
        pass

    def write_param(self, device_id, param, value):
        self.param_writes.append((device_id, param.index, value))
        return object()  # a truthy "status ack"

    def enable(self, device_id):
        self.enabled.append(device_id)
        return None


def test_enable_arms_motor_side_can_timeout():
    bus = RecordingBus()
    worker = wk.ControlWorker()
    worker._bus = bus

    worker._enable(4)

    timeout_writes = [(did, value) for did, index, value in bus.param_writes
                      if index == ParameterType.CAN_TIMEOUT.index]
    assert timeout_writes == [(4, wk.MOTOR_CAN_TIMEOUT_MS)]
    assert bus.enabled == [4]


def test_enable_skips_can_timeout_when_disabled_by_config():
    bus = RecordingBus()
    worker = wk.ControlWorker()
    worker._bus = bus
    worker.motor_can_timeout_ms = 0  # operator opted out

    worker._enable(4)

    indices = [index for _, index, _ in bus.param_writes]
    assert ParameterType.CAN_TIMEOUT.index not in indices
    assert bus.enabled == [4]


def test_unacked_can_timeout_write_still_enables_but_raises_error():
    class NoAckBus(RecordingBus):
        def write_param(self, device_id, param, value):
            super().write_param(device_id, param, value)
            return None  # motor never acked

    bus = NoAckBus()
    worker = wk.ControlWorker()
    worker._bus = bus
    errors: list[str] = []
    worker.error.connect(errors.append)

    worker._enable(4)

    assert bus.enabled == [4]  # enable is not blocked by the missing ack
    # An unarmed motor-side watchdog is safety-relevant: it must surface as an
    # error (popup + log), not disappear into the log dock.
    assert any("canTimeout" in m for m in errors)


def test_one_dead_motor_among_healthy_ones_still_trips_watchdog():
    """A healthy motor's success must not reset the count a dead one accrues.

    Regression: with a shared counter reset on any success, motor 5 (healthy)
    would clear motor 4's (dead) failures every cycle - the watchdog could
    never trip and the first-error dedup re-fired every cycle.
    """
    class HalfDeadBus(DeadBus):
        def set_position(self, device_id, position, velocity_limit=None):
            if device_id == 4:
                raise TransportError("serial write failed: device gone")
            return None  # motor 5 keeps answering

    bus = HalfDeadBus()
    worker = _worker_with_enabled_motor(bus)
    worker._targets[5] = wk.MotorTarget(enabled=True)
    errors: list[str] = []
    worker.error.connect(errors.append)

    _service_n_times(worker, wk.COMM_FAILURE_LIMIT)

    assert worker._bus is None          # watchdog tripped
    assert bus.closed
    assert len(errors) == 2             # first failure + the disconnect notice
