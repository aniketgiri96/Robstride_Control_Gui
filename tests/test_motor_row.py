"""Tests for the dashboard MotorRow card.

Runs the real widget under Qt's offscreen platform. Covers the calibration
workflow (capture-current-position, lock disables edits, explicit Save emits
clamped/ordered limits), the commanded-vs-actual angle tracking, and the
current/torque warning threshold.
"""

from __future__ import annotations

import math
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from dataclasses import dataclass

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui.protocol import MotorStatus, RunMode
from robstride_gui.ui.motor_row import MotorRow, is_warning


@dataclass
class _Power:
    iq: float


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def row(app):
    r = MotorRow(device_id=2, model="rs-04")
    yield r
    r.deleteLater()


def _status(pos=0.0, vel=0.0, torque=0.0):
    return MotorStatus(device_id=2, position=pos, velocity=vel,
                       torque=torque, temperature=25.0)


def _collect(signal):
    captured: list[tuple] = []
    signal.connect(lambda *args: captured.append(args))
    return captured


# -- calibration -----------------------------------------------------------------


def test_capture_min_copies_actual_angle_into_min_spin(row):
    # Arrange: unlock and feed a live actual angle
    row.set_locked(False)
    row.update_status(_status(pos=math.radians(15.0)))

    # Act
    row.capture_min_btn.click()

    # Assert: min spin now reads the actual angle (deg)
    assert row.min_spin.value() == pytest.approx(15.0, abs=0.01)


def test_lock_disables_calibration_edits(row):
    # Act
    row.set_locked(True)

    # Assert: every calibration edit widget is disabled while locked
    assert not row.min_spin.isEnabled()
    assert not row.max_spin.isEnabled()
    assert not row.capture_min_btn.isEnabled()
    assert not row.save_btn.isEnabled()


def test_unlock_enables_calibration_edits(row):
    row.set_locked(False)
    assert row.min_spin.isEnabled()
    assert row.save_btn.isEnabled()


def test_save_emits_range_limits_in_radians(row):
    # Arrange
    row.set_locked(False)
    emitted = _collect(row.rangeLimitsEdited)
    row.min_spin.setValue(-20.0)
    row.max_spin.setValue(40.0)

    # Act
    row.save_btn.click()

    # Assert: emitted in radians, only on explicit Save
    assert len(emitted) == 1
    device_id, lo, hi = emitted[0]
    assert device_id == 2
    assert lo == pytest.approx(math.radians(-20.0))
    assert hi == pytest.approx(math.radians(40.0))


def test_save_orders_min_max_when_reversed(row):
    # Arrange: user entered min > max
    row.set_locked(False)
    emitted = _collect(row.rangeLimitsEdited)
    row.min_spin.setValue(30.0)
    row.max_spin.setValue(-30.0)

    # Act
    row.save_btn.click()

    # Assert: normalised so lo <= hi
    _, lo, hi = emitted[0]
    assert lo == pytest.approx(math.radians(-30.0))
    assert hi == pytest.approx(math.radians(30.0))


def test_editing_spins_does_not_apply_without_save(row):
    # Arrange
    row.set_locked(False)
    emitted = _collect(row.rangeLimitsEdited)

    # Act: change values but never click Save
    row.min_spin.setValue(-5.0)
    row.max_spin.setValue(5.0)

    # Assert: nothing applied
    assert emitted == []


def test_lock_toggle_emits_lock_changed(row):
    emitted = _collect(row.lockChanged)
    row.lock_btn.setChecked(False)
    assert emitted[-1] == (2, False)


# -- commanded vs actual ---------------------------------------------------------


def test_send_target_records_commanded_and_emits_position(row):
    # Arrange
    targets = _collect(row.targetChanged)

    # Act
    row.target_spin.setValue(45.0)
    row._on_send_target()

    # Assert: a position target was emitted in radians
    device_id, changes = targets[-1]
    assert device_id == 2
    assert changes["position"] == pytest.approx(math.radians(45.0))


def test_enable_syncs_commanded_to_actual(row):
    # Arrange: actual is at 10 deg, commanded elsewhere
    row.update_status(_status(pos=math.radians(10.0)))
    row.set_commanded(math.radians(90.0))

    # Act: enabling comes up holding the current spot
    row.set_enabled_state(True)

    # Assert: commanded ghost realigned to actual (shown in the angle label)
    assert "+10.0° → +10.0°" in row.angle_lbl.text()


# -- warning ---------------------------------------------------------------------


def test_is_warning_trips_on_current():
    # rs-04 current_max is 10.0 A; 0.8 * 10 = 8.0 A threshold
    assert is_warning(current=8.5, torque=0.0, current_max=10.0, torque_max=60.0)
    assert not is_warning(current=5.0, torque=0.0, current_max=10.0, torque_max=60.0)


def test_is_warning_trips_on_torque():
    assert is_warning(current=0.0, torque=55.0, current_max=10.0, torque_max=60.0)


def test_is_warning_ignores_none_limits():
    assert not is_warning(current=999.0, torque=999.0,
                          current_max=None, torque_max=None)


def test_update_power_sets_current_draw(row):
    # Act: push a current reading
    row.update_power(_Power(iq=9.0))

    # Assert: row reports the draw magnitude for the totals bar
    assert row.current_draw() == pytest.approx(9.0)


# -- calibrated-range constraints ------------------------------------------------


def test_position_limits_rerange_motion_inputs(row):
    # Act: calibrate a tight range
    row.set_position_limits(math.radians(-10.0), math.radians(20.0))

    # Assert: target/A/B spins cannot request outside the calibrated travel
    for spin in (row.target_spin, row.a_spin, row.b_spin):
        assert spin.minimum() == pytest.approx(-10.0, abs=0.01)
        assert spin.maximum() == pytest.approx(20.0, abs=0.01)


def test_position_limits_clamp_existing_values(row):
    # Arrange: dial a wide target, then tighten the range under it
    row.target_spin.setValue(90.0)

    # Act
    row.set_position_limits(math.radians(-10.0), math.radians(20.0))

    # Assert: the out-of-range value was clamped to the new max
    assert row.target_spin.value() == pytest.approx(20.0, abs=0.01)


# -- A/B safety ------------------------------------------------------------------


def test_ab_start_refused_when_disabled(row):
    # Arrange: motor disabled, no position targets should be emitted
    row.set_enabled_state(False)
    targets = _collect(row.targetChanged)

    # Act: try to start A/B
    row.ab_btn.setChecked(True)

    # Assert: refused - button un-checks, timer idle, nothing commanded
    assert not row.ab_active
    assert not row.ab_btn.isChecked()
    assert targets == []


def test_ab_start_allowed_when_enabled(row):
    # Arrange
    row.set_enabled_state(True)
    targets = _collect(row.targetChanged)

    # Act
    row.ab_btn.setChecked(True)

    # Assert: cycling started and the first move was commanded
    assert row.ab_active
    assert targets  # at least the immediate first step
    row.ab_btn.setChecked(False)  # stop the timer for teardown


# -- Make LIMP -------------------------------------------------------------------


def test_make_limp_sets_mit_zeroes_gains_and_enables(row):
    # Arrange: motor disabled
    row.set_enabled_state(False)
    modes = _collect(row.modeChanged)
    targets = _collect(row.targetChanged)
    enables = _collect(row.enableToggled)

    # Act
    row.limp_btn.click()

    # Assert: MIT mode, zeroed gains as one target, and enable - the back-drivable
    # state for hand-teaching.
    assert modes[-1] == (2, RunMode.MIT)
    assert targets[-1] == (2, {"kp": 0.0, "kd": 0.0, "torque_ff": 0.0})
    assert enables[-1] == (2, True)


def test_make_limp_does_not_re_emit_enable_when_already_enabled(row):
    # Arrange: motor already enabled
    row.set_enabled_state(True)
    enables = _collect(row.enableToggled)

    # Act
    row.limp_btn.click()

    # Assert: gains still pushed, but no duplicate enable
    assert enables == []
