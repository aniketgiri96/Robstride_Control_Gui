"""Blocking-duration instrumentation for the single-threaded control loop.

The worker services motors and applies queued commands on one thread, so a slow
command handler (a multi-round-trip enable or mode-switch) is dead time in which
no *other* enabled motor gets a hold frame. ``_log_slow_command`` measures that
window and logs it when it exceeds ``SLOW_COMMAND_LOG_MS``, turning "M6 drops
when M5 mode-switches" from inference into a number to compare against a motor's
firmware watchdog. No hardware and no Qt loop: the log signal is captured with a
plain stub.
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import RunMode


def _worker_capturing():
    worker = wk.ControlWorker()
    logs: list[str] = []
    worker.log.connect(logs.append)
    return worker, logs


def test_slow_command_is_logged_with_duration_and_target():
    # Arrange
    worker, logs = _worker_capturing()
    over = (wk.SLOW_COMMAND_LOG_MS + 50.0) / 1000.0  # seconds, safely over threshold

    # Act
    worker._log_slow_command(wk.SetMode(device_id=5, mode=RunMode.POSITION_PP), over)

    # Assert
    assert len(logs) == 1
    line = logs[0]
    assert "[timing]" in line
    assert "M5" in line
    assert "SetMode" in line
    assert "ms" in line


def test_drain_commands_is_bounded_per_call():
    # A burst larger than MAX_COMMANDS_PER_DRAIN must not be drained in one
    # call - service_motors needs a turn between bursts so no enabled motor's
    # keepalive frame is starved by an arbitrarily large queue.
    worker, _ = _worker_capturing()
    burst = wk.MAX_COMMANDS_PER_DRAIN * 2
    for i in range(burst):
        worker.post(wk.SetTarget(device_id=1, position=float(i)))

    worker._drain_commands()
    assert worker._queue.qsize() == burst - wk.MAX_COMMANDS_PER_DRAIN

    worker._drain_commands()
    assert worker._queue.qsize() == 0
    assert worker._targets[1].position == float(burst - 1)


def test_fast_command_is_not_logged():
    # Arrange
    worker, logs = _worker_capturing()
    under = (wk.SLOW_COMMAND_LOG_MS - 1.0) / 1000.0  # just under the threshold

    # Act
    worker._log_slow_command(wk.SetMode(device_id=5, mode=RunMode.POSITION_PP), under)

    # Assert
    assert logs == []


def test_command_without_device_id_omits_target():
    # A connection-scoped command carries no device id; the line must still log,
    # just without an "M<n>" tag rather than crashing on the missing attribute.
    worker, logs = _worker_capturing()
    over = (wk.SLOW_COMMAND_LOG_MS + 50.0) / 1000.0

    class _NoDevice:  # a minimal command-shaped object with no device_id
        pass

    worker._log_slow_command(_NoDevice(), over)

    assert len(logs) == 1
    assert "[timing]" in logs[0]
    assert "M" not in logs[0].split("]", 1)[1]  # no "M<n>" after the "[timing]" tag


def test_drain_commands_times_each_applied_command():
    # Integration: a slow _apply routed through _drain_commands emits a timing
    # line. Force slowness by stubbing _apply to block past the threshold.
    worker, logs = _worker_capturing()
    delay_s = (wk.SLOW_COMMAND_LOG_MS + 40.0) / 1000.0

    import time as _time

    def _slow_apply(cmd):
        _time.sleep(delay_s)

    worker._apply = _slow_apply
    worker.post(wk.SetMode(device_id=6, mode=RunMode.POSITION_PP))

    worker._drain_commands()

    timing = [l for l in logs if l.startswith("[timing]")]
    assert len(timing) == 1
    assert "M6" in timing[0]
