"""Tests for :mod:`robstride_gui.interp` - zero-order-hold smoothing.

A hand-teach recording logs at ~100 Hz while the motor state behind it updates
at ~5 Hz, so each joint value sits flat for ~200 ms then jumps several degrees in
one frame. These tests pin the reconstruction: collapse the held-flat runs to the
waypoint where the value actually changed, then piecewise-linearly interpolate.
"""

import math

from robstride_gui.interp import dedupe_holds, lerp_at, smooth_columns


class TestDedupeHolds:
    def test_collapses_a_held_run_to_its_first_sample(self):
        # Arrange: a value held flat, then a jump, then held flat again.
        times = [0.0, 0.1, 0.2, 0.3, 0.4]
        values = [10.0, 10.0, 10.0, 22.0, 22.0]

        # Act
        wt, wv = dedupe_holds(times, values)

        # Assert: waypoint at each value's FIRST occurrence, plus the final time
        # so the ending pose is held to the end.
        assert wt == [0.0, 0.3, 0.4]
        assert wv == [10.0, 22.0, 22.0]

    def test_keeps_every_genuinely_changing_sample(self):
        # Arrange: encoder-tick motion, no held runs.
        times = [0.0, 0.1, 0.2]
        values = [0.0, 0.022, 0.044]

        # Act
        wt, wv = dedupe_holds(times, values)

        # Assert: nothing collapses.
        assert wt == times
        assert wv == values

    def test_single_sample_is_returned_unchanged(self):
        # Arrange / Act
        wt, wv = dedupe_holds([1.0], [5.0])

        # Assert
        assert wt == [1.0]
        assert wv == [5.0]


class TestLerpAt:
    def test_interpolates_between_waypoints(self):
        # Arrange
        wt, wv = [0.0, 1.0], [0.0, 10.0]

        # Act / Assert
        assert lerp_at(wt, wv, 0.5) == 5.0
        assert lerp_at(wt, wv, 0.25) == 2.5

    def test_clamps_outside_the_waypoint_range(self):
        # Arrange
        wt, wv = [0.0, 1.0], [3.0, 7.0]

        # Act / Assert: no extrapolation past the recorded endpoints.
        assert lerp_at(wt, wv, -5.0) == 3.0
        assert lerp_at(wt, wv, 99.0) == 7.0

    def test_hits_waypoints_exactly(self):
        # Arrange
        wt, wv = [0.0, 1.0, 2.0], [1.0, 4.0, 9.0]

        # Act / Assert
        assert lerp_at(wt, wv, 1.0) == 4.0


class TestSmoothColumns:
    def test_staircase_becomes_a_bounded_slope_ramp(self):
        # Arrange: 200 ms flat, then a 12 deg jump in one 10 ms frame (~1200 deg/s).
        dt = 0.01
        times = [i * dt for i in range(41)]
        col = [10.0] * 20 + [22.0] * 21  # jump at index 20

        # Act
        (smoothed,) = smooth_columns(times, [col])

        # Assert: the raw jump was 12 deg/frame; smoothing spreads it over the
        # whole dwell so no single frame step exceeds a fraction of that.
        raw_max = max(abs(col[i + 1] - col[i]) for i in range(len(col) - 1))
        out_max = max(abs(smoothed[i + 1] - smoothed[i]) for i in range(len(smoothed) - 1))
        assert raw_max == 12.0
        assert out_max < 1.0
        # Endpoints are preserved (no overshoot beyond recorded values).
        assert smoothed[0] == 10.0
        assert smoothed[-1] == 22.0
        assert min(smoothed) >= 10.0 - 1e-9
        assert max(smoothed) <= 22.0 + 1e-9

    def test_output_keeps_the_original_length_and_timeline(self):
        # Arrange
        times = [0.0, 0.1, 0.2, 0.3]
        cols = [[1.0, 1.0, 5.0, 5.0], [0.0, 0.0, 0.0, 2.0]]

        # Act
        out = smooth_columns(times, cols)

        # Assert: same shape - the sequencer plays it at the same fps/duration.
        assert len(out) == 2
        assert all(len(c) == len(times) for c in out)

    def test_channels_are_smoothed_independently(self):
        # Arrange: channel A jumps early, channel B jumps late.
        times = [0.0, 0.1, 0.2, 0.3, 0.4]
        a = [0.0, 6.0, 6.0, 6.0, 6.0]
        b = [0.0, 0.0, 0.0, 0.0, 8.0]

        # Act
        sa, sb = smooth_columns(times, [a, b])

        # Assert: A already ramped by mid-track; B is still near zero there.
        assert sa[2] == 6.0
        assert sb[2] == 0.0

    def test_no_times_returns_input_untouched(self):
        # Arrange
        cols = [[1.0, 9.0, 3.0]]

        # Act: without a usable timeline there is nothing to interpolate against.
        out = smooth_columns([], cols)

        # Assert
        assert out == cols

    def test_smoothing_a_pure_ramp_is_a_no_op(self):
        # Arrange: already-smooth data must pass through unchanged.
        times = [0.0, 0.1, 0.2, 0.3]
        col = [0.0, 1.0, 2.0, 3.0]

        # Act
        (out,) = smooth_columns(times, [col])

        # Assert
        assert all(math.isclose(o, c) for o, c in zip(out, col))
