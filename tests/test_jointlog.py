"""Tests for the joint-telemetry-log adapter.

Pure data parsing - no Qt, no hardware. Verifies the wide log shape (time, mode,
four columns per joint) is reduced to just the requested joints' angle column,
converted deg -> rad, with the frame rate derived from the time column and the
channel -> CAN id map pinned to the joint numbers.
"""

from __future__ import annotations

import math

import pytest

from robstride_gui.jointlog import load_joint_log
from robstride_gui.sequence import DEFAULT_FPS, SequenceError

# A 3-row log mirroring joint_log.csv's shape, trimmed to joints 3 and 6.
_HEADER = ("time,mode,"
           "cmd_revolute_3_deg,pos_revolute_3_deg,vel_revolute_3_degps,torque_revolute_3_Nm,"
           "cmd_revolute_6_deg,pos_revolute_6_deg,vel_revolute_6_degps,torque_revolute_6_Nm")
_LOG = (
    f"{_HEADER}\n"
    "0.000,manual,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0\n"
    "0.020,manual,0.0,-0.1,0.0,0.0,30.0,29.5,0.0,0.0\n"
    "0.040,auto,0.0,-0.2,0.0,0.0,-30.0,-29.5,0.0,0.0\n"
)


def _write(tmp_path, text=_LOG):
    p = tmp_path / "joint_log.csv"
    p.write_text(text)
    return p


def test_selects_requested_joint_angle_columns(tmp_path):
    # Act
    seq, channel_map = load_joint_log(_write(tmp_path), joints=(3, 6), source="pos")

    # Assert: two channels named after the joints, frame count matches data rows
    assert seq.channels == ("3", "6")
    assert seq.frame_count == 3
    # pos_revolute_6_deg at frame 1 is 29.5 deg -> radians
    assert seq.angle_at(1, 1) == pytest.approx(math.radians(29.5))
    assert seq.angle_at(2, 0) == pytest.approx(math.radians(-0.2))


def test_channel_map_pins_joint_to_can_id(tmp_path):
    _, channel_map = load_joint_log(_write(tmp_path), joints=(3, 6))
    assert channel_map == {0: 3, 1: 6}


def test_fps_derived_from_time_column(tmp_path):
    # 0.02 s median dt -> 50 fps
    seq, _ = load_joint_log(_write(tmp_path), joints=(3, 6))
    assert seq.fps == pytest.approx(50.0)


def test_source_cmd_selects_commanded_column(tmp_path):
    seq, _ = load_joint_log(_write(tmp_path), joints=(6,), source="cmd")
    # cmd_revolute_6_deg is 30 at frame 1, -30 at frame 2
    assert seq.angle_at(1, 0) == pytest.approx(math.radians(30.0))
    assert seq.angle_at(2, 0) == pytest.approx(math.radians(-30.0))


def test_missing_joint_column_raises(tmp_path):
    with pytest.raises(SequenceError):
        load_joint_log(_write(tmp_path), joints=(3, 99))


def test_unknown_source_raises(tmp_path):
    with pytest.raises(SequenceError):
        load_joint_log(_write(tmp_path), joints=(3,), source="torque")


def test_missing_file_raises(tmp_path):
    with pytest.raises(SequenceError):
        load_joint_log(tmp_path / "nope.csv")


def test_non_numeric_angle_raises(tmp_path):
    bad = (f"{_HEADER}\n"
           "0.000,manual,0.0,notanumber,0.0,0.0,0.0,0.0,0.0,0.0\n")
    with pytest.raises(SequenceError):
        load_joint_log(_write(tmp_path, bad), joints=(3,))


def test_fps_falls_back_without_time_column(tmp_path):
    # Header without a time column -> default fps, columns still resolve by name.
    text = ("mode,pos_revolute_6_deg\n"
            "manual,0.0\nmanual,10.0\n")
    seq, _ = load_joint_log(_write(tmp_path, text), joints=(6,))
    assert seq.fps == DEFAULT_FPS
    assert seq.frame_count == 2


def test_blank_rows_skipped(tmp_path):
    text = (f"{_HEADER}\n"
            "0.000,manual,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0\n"
            "\n"
            "0.020,manual,0.0,0.0,0.0,0.0,0.0,10.0,0.0,0.0\n")
    seq, _ = load_joint_log(_write(tmp_path, text), joints=(6,))
    assert seq.frame_count == 2
