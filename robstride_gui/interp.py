"""Smooth a coarse, zero-order-hold joint track into a continuous one.

Hand-teach recordings (``mujoco_teach.py --record``) are logged at ~100 Hz, but
the motor state behind them updates far slower - the GUI's status poll lands at
roughly 5 Hz. Each joint value therefore sits *exactly* flat for ~200 ms and then
jumps several degrees in a single frame (a ~1200 deg/s step). Replayed verbatim,
whether into the MuJoCo viewer or onto real motors, that reads as jerky, stepping
motion rather than the fluid hand movement that was actually performed.

This module reconstructs the underlying continuous motion:

1. :func:`dedupe_holds` collapses each run of held-flat samples to the single
   waypoint where the value first changed (the change point), keeping the final
   timestamp so the ending pose holds to the end.
2. :func:`lerp_at` / :func:`smooth_columns` re-evaluate a piecewise-linear track
   through those waypoints on the *original* timeline, so the smoothed track has
   the same length, duration, and frame rate - it just fills the plateaus with a
   gentle ramp instead of a flat-then-jump staircase.

Linear (not spline) interpolation is deliberate: it never overshoots a recorded
angle, so a smoothed track stays inside exactly the range the hand motion
covered. That matters when the target motors have software range limits.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

#: Samples whose value differs from the last kept waypoint by no more than this
#: (in the track's own units) are treated as a held repeat. The hold plateaus are
#: exact repeats, so any positive floor below one encoder quantum collapses them
#: while leaving genuine motion untouched.
DEFAULT_HOLD_EPS = 1e-6

#: Longest a single change point is allowed to ramp, in seconds. A zero-order-hold
#: transition really happened within about one poll period (~0.2 s here) before it
#: was observed, so the ramp is confined to that window: a joint that genuinely
#: sits still for seconds then moves makes a quick clean move at the end instead
#: of drifting across the whole hold. During active motion the change points are
#: closer than this, so each transition just fills its own dwell and the track is
#: continuous.
DEFAULT_MAX_TRANSITION_S = 0.2


def dedupe_holds(
    times: Sequence[float],
    values: Sequence[float],
    eps: float = DEFAULT_HOLD_EPS,
) -> Tuple[List[float], List[float]]:
    """Collapse zero-order-hold runs to their change points.

    Returns ``(waypoint_times, waypoint_values)``: the first sample, every sample
    that differs from the previously kept one by more than ``eps``, and the final
    sample (so the track spans the full timeline and holds its ending value).
    """
    n = len(values)
    if n == 0:
        return [], []
    wt = [times[0]]
    wv = [values[0]]
    for i in range(1, n):
        if abs(values[i] - wv[-1]) > eps:
            wt.append(times[i])
            wv.append(values[i])
    # Keep the final sample as an endpoint so a trailing plateau is held to the
    # end instead of ramping early toward the last change point.
    last_t = times[n - 1]
    if wt[-1] != last_t:
        wt.append(last_t)
        wv.append(wv[-1])
    return wt, wv


def lerp_at(wt: Sequence[float], wv: Sequence[float], t: float) -> float:
    """Piecewise-linear value at time ``t`` over waypoints ``(wt, wv)``.

    ``wt`` must be non-empty and non-decreasing. ``t`` outside ``[wt[0], wt[-1]]``
    is clamped to the nearest endpoint - the track is never extrapolated past a
    recorded angle.
    """
    n = len(wt)
    if n == 1 or t <= wt[0]:
        return wv[0]
    if t >= wt[-1]:
        return wv[-1]
    # Linear scan: waypoint lists are short (one entry per change point), so a
    # bisection buys nothing and costs clarity.
    for i in range(1, n):
        if t <= wt[i]:
            t0, t1 = wt[i - 1], wt[i]
            v0, v1 = wv[i - 1], wv[i]
            span = t1 - t0
            if span <= 0:
                return v1
            return v0 + (v1 - v0) * (t - t0) / span
    return wv[-1]


def ramp_waypoints(
    wt: Sequence[float],
    wv: Sequence[float],
    max_transition_s: float = DEFAULT_MAX_TRANSITION_S,
) -> Tuple[List[float], List[float]]:
    """Turn change points into bounded ramps around each transition.

    Given change-point waypoints ``(wt, wv)`` from :func:`dedupe_holds`, hold each
    value flat until ``max_transition_s`` before the next change (clamped so it
    never precedes the previous waypoint), then ramp to the new value over that
    window. The transition is confined to roughly one poll period, so a long
    genuine hold stays put and then moves quickly instead of drifting the whole
    time.
    """
    n = len(wt)
    if n <= 1:
        return list(wt), list(wv)
    out_t = [wt[0]]
    out_v = [wv[0]]
    for i in range(1, n):
        dwell = wt[i] - wt[i - 1]
        ramp = dwell if dwell < max_transition_s else max_transition_s
        ramp_start = wt[i] - ramp
        # Hold the previous value flat up to the start of the ramp. Skip the flat
        # anchor when the ramp already fills the whole gap (ramp_start == prev).
        if ramp_start > out_t[-1]:
            out_t.append(ramp_start)
            out_v.append(wv[i - 1])
        out_t.append(wt[i])
        out_v.append(wv[i])
    return out_t, out_v


def smooth_columns(
    times: Sequence[float],
    columns: Sequence[Sequence[float]],
    eps: float = DEFAULT_HOLD_EPS,
    max_transition_s: float = DEFAULT_MAX_TRANSITION_S,
) -> List[List[float]]:
    """Smooth each per-channel value column against a shared ``times`` axis.

    Every column is de-held independently (channels change at different moments),
    expanded into bounded ramps, and re-sampled on the original ``times`` grid, so
    the result has the same shape as ``columns`` and plays back at the same rate.
    With no usable timeline (empty ``times``) the columns are returned unchanged -
    there is nothing to interpolate against.
    """
    if not times:
        return [list(col) for col in columns]
    out: List[List[float]] = []
    for col in columns:
        wt, wv = dedupe_holds(times, col, eps)
        rt, rv = ramp_waypoints(wt, wv, max_transition_s)
        out.append([lerp_at(rt, rv, t) for t in times])
    return out
