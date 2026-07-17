"""Adapter: replay a wide *joint telemetry log* as a motor :class:`Sequence`.

The dashboard sequencer plays an animation *export* (one angle column per motor,
see :mod:`robstride_gui.sequence`). A recorded *joint log* is a different shape:
a ``time`` column, a ``mode`` column, then four columns per joint
(``cmd``/``pos``/``vel``/``torque``), e.g.::

    time,mode,cmd_revolute_3_deg,pos_revolute_3_deg,vel_revolute_3_degps,...
    0.002,manual,0.0,-0.001,-0.429,...

Feeding that straight into :func:`robstride_gui.sequence.parse_csv` fails - it
would treat ``mode`` ("manual"/"auto") as an angle and choke. This module picks
out the requested joints' angle column, converts degrees to radians, derives the
frame rate from the ``time`` column, and returns a plain :class:`Sequence` plus
the ``channel -> CAN id`` map - so playback reuses the existing player/worker path
unchanged.

Angles in the log are absolute joint angles in the user frame, the same
convention the worker applies, so a replayed setpoint lands at the logged angle
*relative to each motor's current zero*: zero-calibrate the target motors to the
log's frame before playing, or the motion is offset by the zero mismatch.
"""

from __future__ import annotations

import csv
import io
import math
import statistics
from pathlib import Path

from .interp import smooth_columns
from .sequence import DEFAULT_FPS, Sequence, SequenceError

#: Column that carries the per-frame timestamp (seconds), used to derive fps.
_TIME_HEADER = "time"

#: Angle sources selectable from the four-column-per-joint layout. ``pos`` is the
#: measured joint angle (faithful to what the robot did, includes sensor noise);
#: ``cmd`` is the clean commanded setpoint track.
_SOURCES = ("pos", "cmd")


def load_joint_log(
    path: str | Path,
    joints: tuple[int, ...] = (1, 2, 3, 4, 5, 6),
    source: str = "pos",
    smooth: bool = True,
) -> tuple[Sequence, dict[int, int]]:
    """Load a joint telemetry log as a playable sequence for ``joints``.

    ``joints`` are the ``revolute_<n>`` numbers to replay, in channel order; each
    also names the CAN id it drives, so the returned map is ``{i: joints[i]}``.
    ``source`` selects the ``pos`` (measured) or ``cmd`` (commanded) angle column.

    ``smooth`` (default on) reconstructs continuous motion from a coarse capture:
    hand-teach logs sample at ~100 Hz while the motor status behind them updates
    at ~5 Hz, so each joint sits flat then jumps several degrees in one frame.
    Replaying that verbatim drives the motors in jerky steps. Smoothing (see
    :mod:`robstride_gui.interp`) fills the plateaus with gentle ramps without
    overshooting any recorded angle. It needs the ``time`` column; without one the
    track is left as recorded. Pass ``smooth=False`` to play the raw samples.

    Returns ``(sequence, channel_map)``. Raises :class:`SequenceError` on a
    missing file, an unknown ``source``, a missing joint column, or malformed data.
    """
    if source not in _SOURCES:
        raise SequenceError(
            f"Unknown angle source {source!r} (use {' or '.join(_SOURCES)})")
    if not joints:
        raise SequenceError("No joints requested for the joint log")

    p = Path(path)
    try:
        text = p.read_text()
    except FileNotFoundError as e:
        raise SequenceError(f"Joint log not found: {p}") from e
    except OSError as e:
        raise SequenceError(f"Could not read joint log {p}: {e}") from e

    rows = list(csv.reader(io.StringIO(text)))
    # Skip leading blank rows: some captures start with a stray CRLF before the
    # header (e.g. a downloaded copy), which would otherwise be read as an empty
    # header with no columns.
    start = next((i for i, r in enumerate(rows)
                  if any(cell.strip() for cell in r)), len(rows))
    rows = rows[start:]
    if len(rows) < 2:
        raise SequenceError("Joint log has no data rows")

    header = [h.strip() for h in rows[0]]
    index = {name: i for i, name in enumerate(header)}

    angle_cols: list[int] = []
    for j in joints:
        col = f"{source}_revolute_{j}_deg"
        if col not in index:
            raise SequenceError(
                f"Joint log is missing column {col!r} "
                f"(available: {', '.join(header)})")
        angle_cols.append(index[col])
    time_col = index.get(_TIME_HEADER)

    frames: list[tuple[float, ...]] = []
    times: list[float] = []
    for i, row in enumerate(rows[1:]):
        if not any(cell.strip() for cell in row):
            continue  # skip blank rows
        frames.append(tuple(
            _angle_rad(row, c, header, i) for c in angle_cols))
        if time_col is not None:
            times.append(_timestamp(row, time_col))

    if not frames:
        raise SequenceError("Joint log has no non-blank data rows")

    # Reconstruct continuous motion from the zero-order-hold capture. Needs a
    # per-frame timeline; without a usable one (no `time` column or a length
    # mismatch) the frames play as recorded.
    if smooth and len(times) == len(frames):
        columns = [[frame[c] for frame in frames] for c in range(len(joints))]
        smoothed = smooth_columns(times, columns)
        frames = [tuple(col[i] for col in smoothed) for i in range(len(frames))]

    channels = tuple(str(j) for j in joints)
    channel_map = {i: j for i, j in enumerate(joints)}
    return Sequence(fps=_fps_from_times(times), channels=channels,
                    frames=tuple(frames)), channel_map


def _angle_rad(row: list[str], col: int, header: list[str], index: int) -> float:
    """Read ``row[col]`` as a finite degree value converted to radians."""
    if col >= len(row):
        raise SequenceError(
            f"Row {index} is short: no value for column {header[col]!r}")
    raw = row[col]
    try:
        deg = float(raw)
    except (TypeError, ValueError) as e:
        raise SequenceError(
            f"Row {index} has a non-numeric angle {raw!r} "
            f"in {header[col]!r}") from e
    if not math.isfinite(deg):
        raise SequenceError(
            f"Row {index} has a non-finite angle {raw!r} in {header[col]!r}")
    return math.radians(deg)


def _timestamp(row: list[str], col: int) -> float:
    """Read a finite timestamp; a bad one just drops out of the fps estimate."""
    if col >= len(row):
        return math.nan
    try:
        t = float(row[col])
    except (TypeError, ValueError):
        return math.nan
    return t if math.isfinite(t) else math.nan


def _fps_from_times(times: list[float]) -> float:
    """Derive a frame rate from the median frame interval in ``times``.

    Uses the median dt so a single logging hiccup does not skew the rate. Falls
    back to :data:`~robstride_gui.sequence.DEFAULT_FPS` when there is no usable
    timing (no ``time`` column, too few rows, or a non-positive interval).
    """
    deltas = [b - a for a, b in zip(times, times[1:])
              if math.isfinite(a) and math.isfinite(b) and b > a]
    if not deltas:
        return DEFAULT_FPS
    dt = statistics.median(deltas)
    return 1.0 / dt if dt > 0 else DEFAULT_FPS
