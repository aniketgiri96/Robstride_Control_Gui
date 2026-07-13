"""Edge-triggered motor fault logging.

``_log_motor_faults`` turns the fault bits in every 100 Hz feedback frame into a
readable log line, but only when the active fault set *changes* - so a sustained
fault does not flood the UI. No hardware and no Qt loop: signals are captured
with plain stubs.
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import MotorStatus


def _status(**faults) -> MotorStatus:
    return MotorStatus(
        device_id=1, position=0.0, velocity=0.0, torque=0.5,
        temperature=30.0, **faults,
    )


def _worker_capturing():
    worker = wk.ControlWorker()
    errors: list[str] = []
    logs: list[str] = []
    worker.error.connect(errors.append)
    worker.log.connect(logs.append)
    return worker, errors, logs


def test_rising_edge_emits_error_once():
    worker, errors, logs = _worker_capturing()

    worker._log_motor_faults(1, _status(undervoltage=True))
    worker._log_motor_faults(1, _status(undervoltage=True))  # same set -> silent
    worker._log_motor_faults(1, _status(undervoltage=True))

    assert len(errors) == 1
    assert "UNDERVOLTAGE" in errors[0]
    assert logs == []


def test_new_fault_appearing_re_emits():
    worker, errors, _ = _worker_capturing()

    worker._log_motor_faults(1, _status(undervoltage=True))
    # overcurrent joins the set -> the active set changed -> log again
    worker._log_motor_faults(1, _status(undervoltage=True, overcurrent=True))

    assert len(errors) == 2
    assert "OVERCURRENT" in errors[1]


def test_clearing_faults_logs_recovery():
    worker, errors, logs = _worker_capturing()

    worker._log_motor_faults(1, _status(stalled=True))
    worker._log_motor_faults(1, _status())  # all clear

    assert len(errors) == 1
    assert logs == ["M1: faults cleared"]


def test_no_fault_stays_silent():
    worker, errors, logs = _worker_capturing()

    worker._log_motor_faults(1, _status())

    assert errors == []
    assert logs == []


def test_disable_resets_fault_state_so_reenable_relogs():
    worker, errors, _ = _worker_capturing()
    worker._bus = _NoopBus()

    worker._log_motor_faults(1, _status(undervoltage=True))
    worker._disable(1)  # forgets the last fault set
    worker._log_motor_faults(1, _status(undervoltage=True))

    assert len(errors) == 2  # logged again after the disable, not deduped


class _NoopBus:
    def disable(self, device_id: int):
        return None
