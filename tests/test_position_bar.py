"""Tests for the PositionBar geometry helpers.

Only the pure placement/direction math is exercised - the paint pass needs no
coverage and would need a real surface. Verifies the marker fraction is clamped
to the ends, handles a degenerate range, and that the direction indicator keys
off the velocity sign with a dead-band.
"""

from __future__ import annotations

import pytest

from robstride_gui.ui.position_bar import MOVING_EPS, direction, marker_fraction


def test_marker_fraction_maps_midpoint_to_half():
    assert marker_fraction(0.0, -1.0, 1.0) == pytest.approx(0.5)


def test_marker_fraction_maps_endpoints():
    assert marker_fraction(-1.0, -1.0, 1.0) == pytest.approx(0.0)
    assert marker_fraction(1.0, -1.0, 1.0) == pytest.approx(1.0)


def test_marker_fraction_clamps_out_of_range():
    assert marker_fraction(-5.0, -1.0, 1.0) == 0.0
    assert marker_fraction(5.0, -1.0, 1.0) == 1.0


def test_marker_fraction_degenerate_range_is_centered():
    # hi <= lo would divide by zero; the helper centres the marker instead.
    assert marker_fraction(3.0, 2.0, 2.0) == 0.5


def test_direction_positive_when_increasing():
    assert direction(1.0) == 1


def test_direction_negative_when_decreasing():
    assert direction(-1.0) == -1


def test_direction_stopped_within_dead_band():
    assert direction(MOVING_EPS / 2) == 0
    assert direction(-MOVING_EPS / 2) == 0
    assert direction(0.0) == 0
