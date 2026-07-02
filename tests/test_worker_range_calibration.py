"""Range (end-stop) calibration and clamping in the control worker.

No hardware and no Qt loop: the worker's command handlers are exercised via
``_apply`` and the pure clamp/capture helpers are called directly. The worker
never opens a bus here, so ``_set_mode`` (used by the limp switch) is a safe
no-op and we can focus on the range bookkeeping.
"""

from __future__ import annotations

import math

from robstride_gui import worker as wk


def test_set_range_limits_stores_and_normalises_order():
    worker = wk.ControlWorker()
    # Min/max supplied out of order (an inverted motor reports ends either way).
    worker._apply(wk.SetRangeLimits(device_id=1, pos_min=0.5, pos_max=-0.5))

    assert worker._range[1] == (-0.5, 0.5)


def test_clamp_to_range_holds_setpoint_inside_bounds():
    worker = wk.ControlWorker()
    worker._apply(wk.SetRangeLimits(device_id=1, pos_min=-1.0, pos_max=1.0))

    assert worker._clamp_to_range(1, 5.0) == 1.0     # above max -> max
    assert worker._clamp_to_range(1, -5.0) == -1.0   # below min -> min
    assert worker._clamp_to_range(1, 0.25) == 0.25   # inside -> unchanged


def test_clamp_with_no_range_is_identity():
    worker = wk.ControlWorker()
    assert worker._clamp_to_range(99, 123.4) == 123.4


def test_one_sided_range_only_clamps_that_side():
    worker = wk.ControlWorker()
    worker._apply(wk.SetRangeLimits(device_id=1, pos_min=None, pos_max=2.0))

    assert worker._clamp_to_range(1, 10.0) == 2.0     # max enforced
    assert worker._clamp_to_range(1, -10.0) == -10.0  # min unbounded


def test_clear_range_removes_clamp():
    worker = wk.ControlWorker()
    worker._apply(wk.SetRangeLimits(device_id=1, pos_min=-1.0, pos_max=1.0))
    worker._apply(wk.SetRangeLimits(device_id=1, pos_min=None, pos_max=None))

    assert worker._clamp_to_range(1, 5.0) == 5.0


def test_calibration_captures_min_and_max_of_samples():
    worker = wk.ControlWorker()
    worker._apply(wk.StartRangeCalibration(device_id=1, make_limp=False))
    for pos in (0.1, -0.7, 0.9, 0.2, -0.3):
        worker._note_range_sample(1, pos)
    worker._apply(wk.StopRangeCalibration(device_id=1))

    assert worker._range[1] == (-0.7, 0.9)
    # Capture state is cleared once committed.
    assert 1 not in worker._range_cal


def test_calibration_seeds_from_current_position():
    worker = wk.ControlWorker()
    # Motor is holding still at a known raw position (direction=+1, offset=0).
    worker._last_raw_pos[1] = 0.42
    worker._apply(wk.StartRangeCalibration(device_id=1, make_limp=False))

    state = worker._range_cal[1]
    assert math.isclose(state["min"], 0.42)
    assert math.isclose(state["max"], 0.42)


def test_stop_without_motion_leaves_range_unchanged():
    worker = wk.ControlWorker()
    worker._apply(wk.SetRangeLimits(device_id=1, pos_min=-1.0, pos_max=1.0))
    # No feedback ever arrived: start then immediately stop.
    worker._apply(wk.StartRangeCalibration(device_id=1, make_limp=False))
    worker._apply(wk.StopRangeCalibration(device_id=1))

    # The prior range must survive an empty capture rather than being wiped.
    assert worker._range[1] == (-1.0, 1.0)


def test_note_sample_only_tracks_during_active_calibration():
    worker = wk.ControlWorker()
    # Not calibrating: samples are ignored, no capture state appears.
    worker._note_range_sample(1, 3.0)
    assert 1 not in worker._range_cal
