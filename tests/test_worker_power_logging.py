"""Edge-triggered VBUS/Iq diagnostic logging.

``_log_power_change`` writes a log line only when the bus voltage moves at least
``VBUS_LOG_DELTA_V`` from the last logged value, so a supply sag is visible in
the log timeline without flooding it at the ~25 Hz read rate.
"""

from __future__ import annotations

from robstride_gui import worker as wk


def _worker_capturing():
    worker = wk.ControlWorker()
    logs: list[str] = []
    worker.log.connect(logs.append)
    return worker, logs


def test_first_reading_seeds_baseline_silently():
    worker, logs = _worker_capturing()
    worker._log_power_change(6, 48.1, 0.4)
    assert logs == []


def test_small_drift_stays_silent():
    worker, logs = _worker_capturing()
    worker._log_power_change(6, 48.1, 0.4)   # baseline
    worker._log_power_change(6, 47.8, 0.5)   # 0.3 V < 1.0 V threshold
    worker._log_power_change(6, 48.0, 0.4)
    assert logs == []


def test_significant_sag_logs_once():
    worker, logs = _worker_capturing()
    worker._log_power_change(6, 48.1, 0.4)   # baseline
    worker._log_power_change(6, 44.0, 6.2)   # 4.1 V drop -> log
    assert len(logs) == 1
    assert "VBUS 48.1 -> 44.0 V" in logs[0]
    assert "Iq +6.2 A" in logs[0]


def test_baseline_is_fixed_not_ratcheted_by_drift():
    # A slow drift under the threshold must not creep the baseline; a later real
    # sag is still measured against the original reference.
    worker, logs = _worker_capturing()
    worker._log_power_change(6, 48.0, 0.4)   # baseline
    for v in (47.5, 47.2, 47.4, 47.6):       # each <1.0 V from 48.0
        worker._log_power_change(6, v, 0.4)
    assert logs == []
    worker._log_power_change(6, 46.9, 5.0)   # 1.1 V from the fixed 48.0 baseline
    assert len(logs) == 1
    assert "48.0 -> 46.9 V" in logs[0]


def test_recovery_also_logs():
    worker, logs = _worker_capturing()
    worker._log_power_change(6, 48.0, 0.4)
    worker._log_power_change(6, 44.0, 6.0)   # sag
    worker._log_power_change(6, 48.0, 0.4)   # recovery back up -> logs again
    assert len(logs) == 2
    assert "44.0 -> 48.0 V" in logs[1]
