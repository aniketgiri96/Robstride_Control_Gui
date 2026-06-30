"""Tests for the per-motor jog (rotate CW/CCW) control.

These run the real :class:`MotorPanel` widget under Qt's offscreen platform so
no display is needed. They verify that pressing the CCW/CW jog buttons switches
the motor into velocity mode and commands the correct signed velocity, and that
releasing a button stops the motor.
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


def test_ccw_jog_commands_positive_velocity(panel):
    # Arrange
    panel.jog_speed_spin.setValue(2.5)
    modes = _collect(panel.modeChanged)
    targets = _collect(panel.targetChanged)

    # Act
    panel.jog_ccw_btn.pressed.emit()

    # Assert: mode switched to velocity and a positive velocity was sent
    assert modes[-1] == (3, RunMode.VELOCITY)
    assert targets[-1] == (3, {"velocity": 2.5})


def test_cw_jog_commands_negative_velocity(panel):
    # Arrange
    panel.jog_speed_spin.setValue(2.5)
    targets = _collect(panel.targetChanged)

    # Act
    panel.jog_cw_btn.pressed.emit()

    # Assert
    assert targets[-1] == (3, {"velocity": -2.5})


def test_releasing_jog_button_stops_motor(panel):
    # Arrange
    panel.jog_speed_spin.setValue(4.0)
    panel.jog_ccw_btn.pressed.emit()
    targets = _collect(panel.targetChanged)

    # Act
    panel.jog_ccw_btn.released.emit()

    # Assert: a zero-velocity command is emitted on release
    assert targets[-1] == (3, {"velocity": 0.0})


def test_jog_does_not_command_mode_while_already_in_velocity_mode(panel):
    # Arrange: already in velocity mode
    idx = panel.mode_combo.findData(RunMode.VELOCITY)
    panel.mode_combo.setCurrentIndex(idx)
    panel.jog_speed_spin.setValue(1.0)
    modes = _collect(panel.modeChanged)

    # Act
    panel.jog_cw_btn.pressed.emit()

    # Assert: mode was already velocity, so no extra mode change is emitted
    assert modes == []
