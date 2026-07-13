"""Tests for the dashboard totals aggregation and motor management.

Runs the real MotorDashboard offscreen. Verifies the totals bar sums current and
torque across rows, counts active (enabled) motors, and that adding/removing
motors keeps the channel map and totals consistent.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from dataclasses import dataclass

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui.protocol import MotorStatus
from robstride_gui.ui.dashboard import MotorDashboard


@dataclass
class _Power:
    iq: float


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def dash(app):
    d = MotorDashboard()
    yield d
    d.deleteLater()


def _status(device_id, torque=0.0):
    return MotorStatus(device_id=device_id, position=0.0, velocity=0.0,
                       torque=torque, temperature=25.0)


def test_add_motor_is_idempotent(dash):
    dash.add_motor(1, "rs-04")
    dash.add_motor(1, "rs-04")
    assert list(dash.rows) == [1]


def test_totals_sum_current_and_torque(dash):
    # Arrange
    dash.add_motor(1, "rs-04")
    dash.add_motor(2, "rs-04")
    dash.update_power(1, _Power(iq=2.0))
    dash.update_power(2, _Power(iq=3.0))
    dash.update_status(1, _status(1, torque=1.5))
    dash.update_status(2, _status(2, torque=2.5))

    # Act
    totals = dash.totals()

    # Assert
    assert totals["current"] == pytest.approx(5.0)
    assert totals["torque"] == pytest.approx(4.0)


def test_totals_current_uses_magnitude(dash):
    dash.add_motor(1, "rs-04")
    dash.add_motor(2, "rs-04")
    dash.update_power(1, _Power(iq=-4.0))   # regen / reverse draw
    dash.update_power(2, _Power(iq=1.0))
    assert dash.totals()["current"] == pytest.approx(5.0)


def test_active_count_tracks_enabled_motors(dash):
    dash.add_motor(1, "rs-04")
    dash.add_motor(2, "rs-04")
    dash.set_enabled_state(1, True)
    assert dash.totals()["active"] == 1
    dash.set_enabled_state(2, True)
    assert dash.totals()["active"] == 2
    dash.set_enabled_state(1, False)
    assert dash.totals()["active"] == 1


def test_channel_map_orders_by_can_id(dash):
    dash.add_motor(3, "rs-04")
    dash.add_motor(1, "rs-04")
    dash.add_motor(2, "rs-04")
    # channel 0 -> lowest id, etc.
    assert dash._channel_map == {0: 1, 1: 2, 2: 3}


def test_remove_motor_updates_totals_and_map(dash):
    dash.add_motor(1, "rs-04")
    dash.add_motor(2, "rs-04")
    dash.update_power(1, _Power(iq=2.0))
    dash.update_power(2, _Power(iq=2.0))
    assert dash.totals()["current"] == pytest.approx(4.0)

    dash.remove_motor(2)
    assert list(dash.rows) == [1]
    assert dash.totals()["current"] == pytest.approx(2.0)


def test_row_signals_are_reemitted_by_dashboard(dash):
    # Arrange
    dash.add_motor(1, "rs-04")
    emitted: list[tuple] = []
    dash.enableToggled.connect(lambda *a: emitted.append(a))

    # Act: simulate the row's enable button firing
    dash.rows[1].enableToggled.emit(1, True)

    # Assert: bubbled up unchanged
    assert emitted == [(1, True)]


def test_sequence_frame_forwards_position_target(dash):
    # Arrange
    dash.add_motor(1, "rs-04")
    targets: list[tuple] = []
    dash.targetChanged.connect(lambda *a: targets.append(a))

    # Act: player callback delivers one channel's angle
    dash._on_sequence_frame(1, 0.25)

    # Assert
    assert targets[-1] == (1, {"position": 0.25})


def test_play_refused_when_no_motor_enabled(dash):
    # Arrange: a loaded sequence but every motor disabled
    from robstride_gui.sequence import Sequence
    dash.add_motor(1, "rs-04")
    dash._seq = Sequence(fps=30.0, channels=("m1",), frames=((0.0,), (0.1,)))
    dash._rebuild_channel_map()
    logs: list[str] = []
    dash.log.connect(logs.append)

    # Act
    dash._on_play_clicked()

    # Assert: nothing plays, operator is told to enable a motor
    assert not dash.player.is_playing
    assert any("enable a motor" in m.lower() for m in logs)


def test_play_starts_when_a_motor_is_enabled(dash):
    # Arrange
    from robstride_gui.sequence import Sequence
    dash.add_motor(1, "rs-04")
    dash.set_enabled_state(1, True)
    dash._seq = Sequence(fps=30.0, channels=("m1",), frames=((0.0,), (0.1,)))
    dash._rebuild_channel_map()

    # Act
    dash._on_play_clicked()

    # Assert
    assert dash.player.is_playing
    dash.player.abort()  # stop the real timer for teardown


def _joint_log(tmp_path):
    p = tmp_path / "joint_log.csv"
    p.write_text(
        "time,mode,cmd_revolute_3_deg,pos_revolute_3_deg,"
        "cmd_revolute_6_deg,pos_revolute_6_deg\n"
        "0.000,manual,0.0,0.0,0.0,0.0\n"
        "0.020,manual,0.0,-0.1,30.0,29.5\n")
    return str(p)


def test_joint_log_import_pins_joints_to_can_ids(dash, tmp_path):
    # Arrange: a decoy motor id 5 is connected alongside the real targets 3 & 6.
    for did in (3, 5, 6):
        dash.add_motor(did, "rs-04")

    # Act
    dash.load_joint_log_file(_joint_log(tmp_path), joints=(3, 6))

    # Assert: channel map pins joint 3 -> id 3, joint 6 -> id 6 (not remapped by
    # ascending order, which would give {0:3, 1:5}).
    assert dash._channel_map == {0: 3, 1: 6}


def test_joint_log_map_survives_play_rebind(dash, tmp_path):
    # Arrange
    for did in (3, 5, 6):
        dash.add_motor(did, "rs-04")
        dash.set_enabled_state(did, True)
    dash.load_joint_log_file(_joint_log(tmp_path), joints=(3, 6))
    posts: list[tuple] = []
    dash.targetChanged.connect(lambda *a: posts.append(a))

    # Act: Play re-binds the map then steps one frame onto the motors.
    dash._on_play_clicked()
    dash.player.tick()
    dash.player.abort()  # stop the real timer for teardown

    # Assert: only the pinned ids 3 and 6 receive setpoints, never the decoy 5.
    driven = {device_id for device_id, _ in posts}
    assert driven == {3, 6}


def test_animation_import_clears_joint_log_pin(dash, tmp_path):
    # Arrange: a joint-log pin is active...
    for did in (3, 6):
        dash.add_motor(did, "rs-04")
    dash.load_joint_log_file(_joint_log(tmp_path), joints=(3, 6))
    assert dash._explicit_channel_map is not None

    # Act: importing a normal animation export must drop the pin.
    seq_csv = tmp_path / "anim.csv"
    seq_csv.write_text("frame,m1,m2\n0,0.0,0.1\n1,0.02,0.12\n")
    dash.load_sequence_file(str(seq_csv))

    # Assert: back to ascending-id auto-mapping.
    assert dash._explicit_channel_map is None
    assert dash._channel_map == {0: 3, 1: 6}
