"""Tests for the animation-sequence parser (JSON + CSV).

Pure data parsing - no Qt, no hardware. Verifies metadata (frame/channel
counts, fps, duration), unit conversion (deg -> rad), the leading-index-column
handling in CSV, and that malformed inputs are rejected with a clear error.
"""

from __future__ import annotations

import json
import math

import pytest

from robstride_gui.sequence import (
    DEFAULT_FPS, Sequence, SequenceError, load_sequence, parse_csv, parse_json,
)


def test_parse_json_reports_metadata():
    # Arrange
    doc = json.dumps({"fps": 30, "units": "rad",
                      "channels": ["m1", "m2"],
                      "frames": [[0.0, 0.1], [0.02, 0.12], [0.04, 0.14]]})

    # Act
    seq = parse_json(doc)

    # Assert
    assert seq.frame_count == 3
    assert seq.channel_count == 2
    assert seq.fps == 30
    assert seq.duration_s == pytest.approx(3 / 30)
    assert seq.angle_at(1, 1) == pytest.approx(0.12)


def test_parse_json_converts_degrees_to_radians():
    # Arrange
    doc = json.dumps({"units": "deg", "channels": ["m1"],
                      "frames": [[90.0], [180.0]]})

    # Act
    seq = parse_json(doc)

    # Assert
    assert seq.angle_at(0, 0) == pytest.approx(math.pi / 2)
    assert seq.angle_at(1, 0) == pytest.approx(math.pi)


def test_parse_json_defaults_fps_when_missing():
    seq = parse_json(json.dumps({"channels": ["m1"], "frames": [[0.0]]}))
    assert seq.fps == DEFAULT_FPS


def test_parse_json_rejects_ragged_frames():
    doc = json.dumps({"channels": ["m1", "m2"], "frames": [[0.0, 0.1], [0.2]]})
    with pytest.raises(SequenceError):
        parse_json(doc)


def test_parse_json_rejects_missing_channels():
    with pytest.raises(SequenceError):
        parse_json(json.dumps({"frames": [[0.0]]}))


def test_parse_json_rejects_non_finite_angle():
    # json.dumps emits Infinity (invalid JSON per spec but Python accepts it);
    # the parser must refuse a non-finite angle regardless.
    doc = json.dumps({"channels": ["m1"], "frames": [[float("inf")]]})
    with pytest.raises(SequenceError):
        parse_json(doc)


def test_parse_csv_drops_leading_frame_column():
    # Arrange: a "frame" index column that must not become a motor channel
    text = "frame,m1,m2\n0,0.0,0.1\n1,0.02,0.12\n"

    # Act
    seq = parse_csv(text)

    # Assert
    assert seq.channels == ("m1", "m2")
    assert seq.frame_count == 2
    assert seq.angle_at(1, 0) == pytest.approx(0.02)
    assert seq.fps == DEFAULT_FPS


def test_parse_csv_without_index_column_keeps_all_headers():
    text = "m1,m2,m3\n0.0,0.1,0.2\n"
    seq = parse_csv(text)
    assert seq.channels == ("m1", "m2", "m3")
    assert seq.channel_count == 3


def test_parse_csv_units_comment_converts_degrees():
    text = "# units: deg\nframe,m1\n0,90\n"
    seq = parse_csv(text)
    assert seq.angle_at(0, 0) == pytest.approx(math.pi / 2)


def test_parse_csv_skips_blank_rows():
    text = "frame,m1\n0,0.0\n\n1,0.5\n"
    seq = parse_csv(text)
    assert seq.frame_count == 2


def test_parse_csv_rejects_non_numeric_angle():
    with pytest.raises(SequenceError):
        parse_csv("frame,m1\n0,notanumber\n")


def test_load_sequence_missing_file_raises_sequence_error():
    with pytest.raises(SequenceError):
        load_sequence("/nonexistent/path/to/seq.json")


def test_load_sequence_dispatches_on_extension(tmp_path):
    # Arrange
    p = tmp_path / "move.json"
    p.write_text(json.dumps({"channels": ["m1"], "frames": [[1.0]]}))

    # Act
    seq = load_sequence(p)

    # Assert
    assert isinstance(seq, Sequence)
    assert seq.angle_at(0, 0) == 1.0


def test_describe_is_human_readable():
    seq = parse_json(json.dumps({"fps": 24, "channels": ["a", "b"],
                                 "frames": [[0, 0], [0, 0]]}))
    text = seq.describe()
    assert "2 frames" in text and "2 channels" in text and "24 fps" in text
