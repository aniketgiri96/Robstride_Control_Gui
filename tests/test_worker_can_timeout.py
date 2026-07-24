"""Fix #3: the motor-side CAN watchdog is armed with a non-zero canTimeout.

A RobStride motor's stored default canTimeout is short (~60 ms) - short enough
that a brief bus starvation trips it and the motor goes limp. Arming a longer,
known value (:data:`worker.MOTOR_CAN_TIMEOUT_RAW`, 1000 ms) on every enable
widens that window past ordinary servicing gaps while still stopping the motor
if the host truly dies. Because the millisecond unit is unverified on this
hardware, ``_configure_motor_watchdog`` reads the value back and reports what
actually stuck.

No Qt loop and no real bus: a fake bus with a settable canTimeout readback.
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import MotorStatus, ParameterType, RunMode
from robstride_gui.transport import TransportError


class WatchdogBus:
    """Enable-capable fake bus; ``readback`` controls what canTimeout reads as.

    ``readback=None`` means "echo whatever was written" (the healthy motor).
    Any other value is returned verbatim to simulate a clamp/rescale/reject.
    ``read_raises`` makes the readback throw, to prove it is non-fatal.
    """

    def __init__(self, readback=None, read_raises=False):
        self._readback = readback
        self._read_raises = read_raises
        self._written_timeout = None
        self.param_writes: list[tuple[int, int, object]] = []
        self.enabled: list[int] = []

    def set_run_mode(self, device_id, mode):
        pass

    def poll_status(self, device_id):
        return MotorStatus(device_id=device_id, position=0.0, velocity=0.0,
                           torque=0.0, temperature=0.0)

    def set_position(self, device_id, position_rad, velocity_limit=None):
        return None

    def write_param(self, device_id, param, value):
        self.param_writes.append((device_id, param.index, value))
        if param is ParameterType.CAN_TIMEOUT:
            self._written_timeout = value
        return object()  # truthy status ack

    def read_param(self, device_id, param):
        if param is ParameterType.CAN_TIMEOUT:
            if self._read_raises:
                raise TransportError("readback failed")
            return self._written_timeout if self._readback is None else self._readback
        return 0.0

    def enable(self, device_id):
        self.enabled.append(device_id)


def _worker(bus):
    worker = wk.ControlWorker()
    worker._bus = bus
    worker._targets[4] = wk.MotorTarget(mode=RunMode.POSITION_PP)
    return worker


def _timeout_writes(bus):
    return [(did, value) for did, index, value in bus.param_writes
            if index == ParameterType.CAN_TIMEOUT.index]


def test_default_config_arms_a_nonzero_can_timeout():
    # The whole point of fix #3: a freshly constructed worker arms the watchdog
    # without the caller having to opt in.
    assert wk.MOTOR_CAN_TIMEOUT_RAW > 0
    bus = WatchdogBus()
    worker = _worker(bus)                      # no manual motor_can_timeout_raw

    worker._enable(4)

    assert _timeout_writes(bus) == [(4, wk.MOTOR_CAN_TIMEOUT_RAW)]
    assert bus.enabled == [4]


def test_healthy_readback_raises_no_error_or_warning():
    bus = WatchdogBus()                        # echoes the written value
    worker = _worker(bus)
    errors, logs = [], []
    worker.error.connect(errors.append)
    worker.log.connect(logs.append)

    worker._enable(4)

    assert not errors
    assert not any("canTimeout" in m for m in logs)


def test_readback_of_zero_surfaces_error():
    # Ack received but the value did not stick: the watchdog is not really armed.
    bus = WatchdogBus(readback=0)
    worker = _worker(bus)
    errors: list[str] = []
    worker.error.connect(errors.append)

    worker._enable(4)

    assert any("did not stick" in m for m in errors)
    assert bus.enabled == [4]                  # still enables


def test_readback_mismatch_logs_a_warning():
    # Firmware stored a different value (clamp/rescale): arm succeeded but the
    # real stop time is unknown - a log note, not a hard error.
    bus = WatchdogBus(readback=500)
    worker = _worker(bus)
    errors, logs = [], []
    worker.error.connect(errors.append)
    worker.log.connect(logs.append)

    worker._enable(4)

    assert not any("did not stick" in m for m in errors)
    assert any("motor stored 500" in m for m in logs)


def test_readback_transport_error_is_non_fatal():
    # A hiccup on the confirmation read must not abort the enable nor raise.
    bus = WatchdogBus(read_raises=True)
    worker = _worker(bus)
    errors: list[str] = []
    worker.error.connect(errors.append)

    worker._enable(4)

    assert bus.enabled == [4]
    assert not any("did not stick" in m for m in errors)


# -- calibration: raw <-> ms conversion ------------------------------------------
# The canTimeout register unit is NOT milliseconds (frames.log, 2026-07-10: 1000
# armed and read back, yet motors tripped at ~60 ms gaps). recommend_can_timeout_raw
# turns one starve-test point into the raw value for a target window.

def test_recommend_scales_raw_to_hit_target_window():
    # Arrange: an early field point - 1000 raw measured as a ~60 ms window.
    # Act: ask for the raw that lands on a 300 ms window.
    # Assert: linear scale => 300/60 * 1000 = 5000 (pure math check, independent
    # of whatever raw value is currently shipped as the default).
    assert wk.recommend_can_timeout_raw(1000, 60.0, 300.0) == 5000


def test_recommend_matches_shipped_default_from_measured_calibration():
    # Arrange: the real measured calibration point the shipped default is
    # derived from - measure_can_timeout(3, 5000) on real hardware
    # (2026-07-23) found 5000 raw held through 400 ms and dropped by 800 ms
    # silence (~600 ms estimate), and the target was widened to 3000 ms to
    # cover observed GIL-jitter gaps (see MOTOR_CAN_TIMEOUT_RAW).
    # Assert: linear scale => 3000/600 * 5000 = 25000, the shipped default.
    assert wk.recommend_can_timeout_raw(5000, 600.0, 3000.0) == 25000
    assert wk.recommend_can_timeout_raw(5000, 600.0, 3000.0) == wk.MOTOR_CAN_TIMEOUT_RAW


def test_recommend_uses_default_target_when_omitted():
    assert (wk.recommend_can_timeout_raw(1000, 60.0)
            == wk.recommend_can_timeout_raw(1000, 60.0, wk.MOTOR_CAN_TIMEOUT_TARGET_MS))


def test_recommend_never_returns_zero_for_tiny_target():
    # A target far below the measured window must still arm the watchdog (>=1),
    # never disable it by rounding to 0.
    assert wk.recommend_can_timeout_raw(1000, 60.0, 0.001) >= 1


def test_recommend_rejects_non_positive_measurement():
    # observed_stop_ms <= 0 means the starve test saw no stop: nothing to scale.
    import pytest
    with pytest.raises(ValueError):
        wk.recommend_can_timeout_raw(1000, 0.0, 300.0)
    with pytest.raises(ValueError):
        wk.recommend_can_timeout_raw(0, 60.0, 300.0)
