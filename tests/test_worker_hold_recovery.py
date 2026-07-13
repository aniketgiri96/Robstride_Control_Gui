"""Auto-recover a motor that silently reverts to standby while enabled.

A RobStride motor whose firmware CAN-watchdog trips de-energises itself to
MotorMode.RESET - limp, ignoring setpoints - while still ACKing our writes and
setting NO fault bit. The only tell is the mode field of the status frame. The
worker watches for that drop on a motor it still holds enabled and re-runs the
full safe-enable, giving up (and disabling + alerting) only if the motor keeps
dropping. Re-enabling feeds the other motors (fix #1), so recovery is safe and
not the ping-pong it would have been without keepalives.

No Qt loop and no real bus: a fake bus whose status frames carry a settable mode.
"""

from __future__ import annotations

import struct

from robstride_gui import worker as wk
from robstride_gui.protocol import (CommunicationType, Frame, MotorMode,
                                    MotorStatus, ParameterType, RunMode,
                                    parse_status)


class FakeBus:
    """Records calls; every command/poll returns a status carrying ``mode``."""

    def __init__(self, mode: int = MotorMode.RESET, pos: float = 0.5):
        self.mode = mode
        self.pos = pos
        self.calls: list[tuple] = []

    def _status(self, device_id, mode=None):
        return MotorStatus(device_id=device_id, position=self.pos, velocity=0.0,
                           torque=0.0, temperature=0.0,
                           mode=self.mode if mode is None else mode)

    def set_run_mode(self, device_id, mode):
        self.calls.append(("set_run_mode", device_id))

    def read_param(self, device_id, param):
        self.calls.append(("read_param", device_id))
        return self.pos if param is ParameterType.MEASURED_POSITION else 0

    def poll_status(self, device_id):
        self.calls.append(("poll_status", device_id))
        # The enable-time seed poll always reads "running" so seeding succeeds;
        # only the per-tick command frame carries the drop under test.
        return self._status(device_id, mode=MotorMode.MOTOR)

    def write_param(self, device_id, param, value):
        self.calls.append(("write_param", device_id))
        return None

    def set_position(self, device_id, position_rad, velocity_limit=None):
        self.calls.append(("set_position", device_id))
        return self._status(device_id)

    def enable(self, device_id):
        self.calls.append(("enable", device_id))

    def disable(self, device_id):
        self.calls.append(("disable", device_id))

    def _names(self):
        return [c[0] for c in self.calls]


def _worker(mode: int = MotorMode.RESET):
    worker = wk.ControlWorker()
    worker._bus = FakeBus(mode=mode)
    worker.motor_can_timeout_raw = 0
    worker._targets[1] = wk.MotorTarget(mode=RunMode.POSITION_PP,
                                        position=0.5, enabled=True)
    return worker, worker._bus


# --- mode decoding --------------------------------------------------------------


def _status_frame(motor_id: int, mode: int, faults: int = 0) -> Frame:
    extra = (mode << 14) | (faults << 8) | motor_id
    data = struct.pack(">HHHH", 32768, 32768, 32768, 250)
    return Frame(CommunicationType.OPERATION_STATUS, extra, 253, data)


def test_parse_status_decodes_operational_mode():
    running = parse_status(_status_frame(6, MotorMode.MOTOR), "rs-04")
    assert running.device_id == 6
    assert running.mode == MotorMode.MOTOR and not running.is_standby

    limp = parse_status(_status_frame(6, MotorMode.RESET), "rs-04")
    assert limp.mode == MotorMode.RESET and limp.is_standby
    assert not limp.has_fault   # the whole point: a drop carries no fault bit


def test_status_mode_defaults_to_running():
    # A hand-built status (tests, synthetic feedback) must not look like a drop.
    assert MotorStatus(device_id=1, position=0, velocity=0, torque=0,
                       temperature=0).mode == MotorMode.MOTOR


# --- recovery behaviour ---------------------------------------------------------


def test_standby_drop_triggers_reenable():
    worker, bus = _worker(mode=MotorMode.RESET)

    worker._service_motors()

    assert ("enable", 1) in bus.calls, "a dropped motor must be re-enabled"
    assert worker._hold_recovery_attempts[1] == 1


def test_running_motor_is_not_reenabled_and_resets_counter():
    worker, bus = _worker(mode=MotorMode.MOTOR)
    worker._hold_recovery_attempts[1] = 2   # stale count from an earlier blip

    worker._service_motors()

    assert ("enable", 1) not in bus.calls
    assert 1 not in worker._hold_recovery_attempts   # cleared once it holds again


def test_persistent_drop_gives_up_disables_and_alerts():
    worker, bus = _worker(mode=MotorMode.RESET)   # never recovers
    errors: list[str] = []
    worker.error.connect(errors.append)

    # Tick until the motor is given up on (it stays enabled until then).
    for _ in range(wk.MAX_HOLD_RECOVERY_ATTEMPTS + 1):
        if worker._targets[1].enabled:
            worker._service_motors()

    assert worker._targets[1].enabled is False
    assert ("disable", 1) in bus.calls
    assert errors and "keeps dropping to standby" in errors[0]
    # Exactly MAX re-enables were attempted before giving up - not an unbounded storm.
    assert bus._names().count("enable") == wk.MAX_HOLD_RECOVERY_ATTEMPTS


def test_recovered_motor_after_blips_does_not_carry_stale_attempts():
    # Two isolated drops that each recover on the next tick must not accumulate
    # toward the give-up limit.
    worker, bus = _worker(mode=MotorMode.RESET)
    worker._service_motors()                 # drop -> re-enable (attempt 1)
    assert worker._hold_recovery_attempts[1] == 1
    bus.mode = MotorMode.MOTOR
    worker._service_motors()                 # holds again -> counter cleared
    assert 1 not in worker._hold_recovery_attempts
    bus.mode = MotorMode.RESET
    worker._service_motors()                 # a later, unrelated drop starts fresh
    assert worker._hold_recovery_attempts[1] == 1


def test_estop_blocks_recovery():
    worker, bus = _worker(mode=MotorMode.RESET)
    worker._safety.engage_estop()
    st = MotorStatus(device_id=1, position=0.5, velocity=0, torque=0,
                     temperature=0, mode=MotorMode.RESET)

    worker._recover_dropped_hold(1, worker._targets[1], st)

    assert ("enable", 1) not in bus.calls   # never energise under E-STOP


def test_range_calibration_blocks_recovery():
    # During range calibration a motor is deliberately limp/moved; a standby
    # reading there must not trigger a re-enable that fights the operator.
    worker, bus = _worker(mode=MotorMode.RESET)
    worker._range_cal[1] = {"min": None, "max": None}
    st = MotorStatus(device_id=1, position=0.5, velocity=0, torque=0,
                     temperature=0, mode=MotorMode.RESET)

    worker._recover_dropped_hold(1, worker._targets[1], st)

    assert ("enable", 1) not in bus.calls
