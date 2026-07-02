"""Continuous position sweep: SetSweep drives a smooth sine setpoint.

No hardware and no Qt loop: the worker's command handlers are called directly
via ``_apply`` and the pure sweep math via ``_sweep_position``.
"""

from __future__ import annotations

import math

from robstride_gui import worker as wk
from robstride_gui.protocol import RunMode


def _target_with_sweep(device_id: int = 1) -> wk.MotorTarget:
    worker = wk.ControlWorker()
    worker._apply(wk.SetSweep(
        device_id=device_id, enabled=True,
        from_pos=math.radians(-30.0), to_pos=math.radians(30.0), period=2.0))
    return worker._targets[device_id]


def test_setsweep_enables_and_stores_endpoints():
    t = _target_with_sweep()

    assert t.sweep_enabled is True
    assert math.isclose(t.sweep_from, math.radians(-30.0))
    assert math.isclose(t.sweep_to, math.radians(30.0))
    assert t.sweep_period == 2.0


def test_sweep_hits_from_at_t0_and_to_at_half_period():
    t = _target_with_sweep()

    # Endpoints are exact regardless of the wall clock: pin the phase by moving
    # the sweep's own t0 relative to "now".
    now = wk.time.monotonic()
    t.sweep_t0 = now                       # phase 0   -> from
    assert math.isclose(wk.ControlWorker._sweep_position(t), t.sweep_from, abs_tol=1e-3)

    t.sweep_t0 = now - t.sweep_period / 2  # phase 0.5 -> to
    assert math.isclose(wk.ControlWorker._sweep_position(t), t.sweep_to, abs_tol=1e-3)

    t.sweep_t0 = now - t.sweep_period      # phase 1.0 -> back to from
    assert math.isclose(wk.ControlWorker._sweep_position(t), t.sweep_from, abs_tol=1e-3)


def test_sweep_stays_within_endpoints():
    t = _target_with_sweep()
    now = wk.time.monotonic()
    lo, hi = sorted((t.sweep_from, t.sweep_to))
    for frac in (0.1, 0.25, 0.4, 0.6, 0.75, 0.9):
        t.sweep_t0 = now - frac * t.sweep_period
        pos = wk.ControlWorker._sweep_position(t)
        assert lo - 1e-9 <= pos <= hi + 1e-9


def test_disable_sweep_freezes_position_and_clears_flag():
    worker = wk.ControlWorker()
    worker._apply(wk.SetSweep(device_id=1, enabled=True,
                              from_pos=-1.0, to_pos=1.0, period=2.0))
    worker._apply(wk.SetSweep(device_id=1, enabled=False,
                              from_pos=-1.0, to_pos=1.0, period=2.0))

    t = worker._targets[1]
    assert t.sweep_enabled is False
    # Held setpoint is a real angle within the swept range, not reset to stale.
    assert -1.0 - 1e-9 <= t.position <= 1.0 + 1e-9


def test_zero_period_is_clamped_not_divide_by_zero():
    worker = wk.ControlWorker()
    # A 0 s period would divide by zero in the phase; the handler must clamp it.
    worker._apply(wk.SetSweep(device_id=1, enabled=True,
                              from_pos=-1.0, to_pos=1.0, period=0.0))

    t = worker._targets[1]
    assert t.sweep_period >= 0.05
    wk.ControlWorker._sweep_position(t)  # must not raise


def test_default_target_has_sweep_off():
    t = wk.MotorTarget()

    assert t.sweep_enabled is False
    assert t.mode == RunMode.POSITION_PP
