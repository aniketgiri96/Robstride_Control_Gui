"""Tests for the per-motor 'Make LIMP' bring-up button.

These run the real :class:`MotorPanel` widget under Qt's offscreen platform so
no display is needed. They verify that pressing 'Make LIMP' switches the motor
into MIT mode, zeros Kp/Kd and the assist torque, and enables the motor - the
back-drivable state used for hand-teaching and range calibration.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui.protocol import RunMode
from robstride_gui.ui.motor_panel import MotorPanel


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


@pytest.fixture
def panel(app):
    p = MotorPanel(device_id=3)
    yield p
    p.deleteLater()


def _collect(signal):
    captured: list[tuple] = []
    signal.connect(lambda *args: captured.append(args))
    return captured


def test_make_limp_switches_to_mit_zeroes_gains_and_enables(panel):
    # Arrange: start from the holding defaults (Position mode, Kp=28, Kd=6).
    modes = _collect(panel.modeChanged)
    targets = _collect(panel.targetChanged)
    enables = _collect(panel.enableToggled)

    # Act
    panel.limp_btn.clicked.emit()

    # Assert: MIT mode selected, one combined zero-gain target, motor enabled.
    assert modes[-1] == (3, RunMode.MIT)
    assert targets[-1] == (3, {"kp": 0.0, "kd": 0.0, "torque_ff": 0.0})
    assert enables[-1] == (3, True)
    # UI reflects the limp state so the operator sees zeroed gains.
    assert panel.kp_spin.value() == 0.0
    assert panel.kd_spin.value() == 0.0
    assert panel.tq_spin.value() == 0.0
    assert panel.mode_combo.currentData() == RunMode.MIT


def test_make_limp_from_already_mit_still_zeroes_and_enables(panel):
    # Arrange: already in MIT mode with non-zero gains.
    idx = panel.mode_combo.findData(RunMode.MIT)
    panel.mode_combo.setCurrentIndex(idx)
    panel.kp_spin.setValue(20.0)
    panel.kd_spin.setValue(4.0)
    modes = _collect(panel.modeChanged)
    targets = _collect(panel.targetChanged)
    enables = _collect(panel.enableToggled)

    # Act
    panel.limp_btn.clicked.emit()

    # Assert: no redundant mode change, but gains still zeroed and motor enabled.
    assert modes == []
    assert targets[-1] == (3, {"kp": 0.0, "kd": 0.0, "torque_ff": 0.0})
    assert enables[-1] == (3, True)
    assert panel.kp_spin.value() == 0.0
    assert panel.kd_spin.value() == 0.0


def test_make_limp_does_not_re_emit_enable_when_already_enabled(panel):
    # Arrange: motor already shown as enabled.
    panel.set_enabled_state(True)
    enables = _collect(panel.enableToggled)

    # Act
    panel.limp_btn.clicked.emit()

    # Assert: gains are re-zeroed but no duplicate enable is emitted.
    assert enables == []
    assert panel.kp_spin.value() == 0.0
    assert panel.kd_spin.value() == 0.0
