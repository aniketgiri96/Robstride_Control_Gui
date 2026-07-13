"""Animation-sequence import for the motor dashboard.

A *sequence* is a multi-channel angle track exported from Blender (or any
animation tool): one column per motor channel, one row per frame, played back
at a fixed frame rate. This module is pure data + parsing - no Qt, no IO beyond
reading the file - so the sequencer that drives motors from it stays trivial and
the parser is fully unit-testable.

Two on-disk formats are accepted, both carrying explicit metadata so the UI can
show what it is about to send *before* playing:

* **JSON**::

      {"fps": 30, "units": "rad", "channels": ["m1", "m2"],
       "frames": [[0.0, 0.1], [0.02, 0.12]]}

* **CSV** (header row names the channels; a leading ``frame``/``index``/``time``
  column is treated as an index and dropped)::

      frame,m1,m2
      0,0.0,0.1
      1,0.02,0.12

Angles are radians by default; ``units: "deg"`` (JSON) or a ``# units: deg``
comment (CSV) converts at load so the rest of the app only ever sees radians -
the same convention the worker and panels use.
"""

from __future__ import annotations

import csv
import io
import json
import math
from dataclasses import dataclass
from pathlib import Path

#: Frame rate used when an export omits one. 30 fps matches Blender's default.
DEFAULT_FPS: float = 30.0

#: Column headers that name an index/time axis rather than a motor channel.
_INDEX_HEADERS = frozenset({"frame", "index", "idx", "time", "t", "#"})


class SequenceError(ValueError):
    """Raised when a sequence file is missing, malformed, or inconsistent."""


@dataclass(frozen=True)
class Sequence:
    """An immutable multi-channel angle track, in radians.

    ``frames[i][j]`` is the angle (rad) of channel ``channels[j]`` at frame
    ``i``. Every frame has one value per channel (rectangular), validated at
    load so playback never indexes a ragged row.
    """

    fps: float
    channels: tuple[str, ...]
    frames: tuple[tuple[float, ...], ...]

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def channel_count(self) -> int:
        return len(self.channels)

    @property
    def duration_s(self) -> float:
        """Playback duration in seconds (0 for an empty track)."""
        if self.fps <= 0 or self.frame_count == 0:
            return 0.0
        return self.frame_count / self.fps

    def angle_at(self, frame: int, channel: int) -> float:
        """Angle (rad) of ``channel`` at ``frame``. Raises IndexError if out of range."""
        return self.frames[frame][channel]

    def describe(self) -> str:
        """One-line metadata summary for the UI to show once loaded."""
        return (f"{self.frame_count} frames × {self.channel_count} channels "
                f"@ {self.fps:g} fps ({self.duration_s:.1f}s)")


# --- loading --------------------------------------------------------------------


def load_sequence(path: str | Path) -> Sequence:
    """Load a sequence from ``path``, dispatching on file extension.

    ``.json`` uses the JSON schema; anything else is parsed as CSV. Raises
    :class:`SequenceError` on a missing or malformed file.
    """
    p = Path(path)
    try:
        text = p.read_text()
    except FileNotFoundError as e:
        raise SequenceError(f"Sequence file not found: {p}") from e
    except OSError as e:
        raise SequenceError(f"Could not read sequence file {p}: {e}") from e

    if p.suffix.lower() == ".json":
        return parse_json(text)
    return parse_csv(text)


def parse_json(text: str) -> Sequence:
    """Parse the JSON sequence schema into a radians :class:`Sequence`."""
    try:
        doc = json.loads(text)
    except json.JSONDecodeError as e:
        raise SequenceError(f"Invalid JSON sequence: {e}") from e
    if not isinstance(doc, dict):
        raise SequenceError("JSON sequence must be an object with 'channels'/'frames'")

    channels = doc.get("channels")
    if not isinstance(channels, list) or not channels:
        raise SequenceError("JSON sequence needs a non-empty 'channels' list")
    channels = tuple(str(c) for c in channels)

    raw_frames = doc.get("frames")
    if not isinstance(raw_frames, list):
        raise SequenceError("JSON sequence needs a 'frames' list")

    to_rad = _unit_converter(doc.get("units"))
    frames: list[tuple[float, ...]] = []
    for i, row in enumerate(raw_frames):
        if not isinstance(row, (list, tuple)):
            raise SequenceError(f"Frame {i} is not a list of angles")
        frames.append(_coerce_row(row, len(channels), i, to_rad))

    fps = _coerce_fps(doc.get("fps"))
    return Sequence(fps=fps, channels=channels, frames=tuple(frames))


def parse_csv(text: str) -> Sequence:
    """Parse the CSV sequence format into a radians :class:`Sequence`.

    The first non-empty, non-comment row is the header naming the channels. A
    leading index/time column (``frame``, ``index``, ``time``, ...) is dropped.
    Units default to radians; a ``# units: deg`` comment line switches to degrees.
    """
    units: str | None = None
    body_lines: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            # Support a "# units: deg" directive in a leading comment.
            after = stripped.lstrip("#").strip()
            if after.lower().startswith("units") and ":" in after:
                units = after.split(":", 1)[-1].strip()
            continue
        if stripped:
            body_lines.append(line)

    rows = list(csv.reader(io.StringIO("\n".join(body_lines))))
    if not rows:
        raise SequenceError("CSV sequence is empty")

    header = [h.strip() for h in rows[0]]
    drop_first = bool(header) and header[0].lower() in _INDEX_HEADERS
    channels = tuple(header[1:] if drop_first else header)
    if not channels:
        raise SequenceError("CSV sequence header names no channels")

    to_rad = _unit_converter(units)
    frames: list[tuple[float, ...]] = []
    for i, row in enumerate(rows[1:]):
        values = row[1:] if drop_first else row
        if not any(cell.strip() for cell in values):
            continue  # skip blank data rows
        frames.append(_coerce_row(values, len(channels), i, to_rad))

    return Sequence(fps=DEFAULT_FPS, channels=channels, frames=tuple(frames))


# --- helpers --------------------------------------------------------------------


def _unit_converter(units: str | None):
    """Return a callable mapping a stored angle to radians for ``units``."""
    if units is None:
        return lambda v: v
    u = str(units).strip().lower()
    if u in ("rad", "radian", "radians", ""):
        return lambda v: v
    if u in ("deg", "degree", "degrees"):
        return math.radians
    raise SequenceError(f"Unknown angle units '{units}' (use 'rad' or 'deg')")


def _coerce_row(row, channel_count: int, index: int, to_rad) -> tuple[float, ...]:
    """Validate one frame: right width, finite floats, converted to radians."""
    if len(row) != channel_count:
        raise SequenceError(
            f"Frame {index} has {len(row)} values but there are "
            f"{channel_count} channels")
    out: list[float] = []
    for value in row:
        try:
            angle = float(value)
        except (TypeError, ValueError) as e:
            raise SequenceError(f"Frame {index} has a non-numeric angle "
                                f"{value!r}") from e
        if not math.isfinite(angle):
            raise SequenceError(f"Frame {index} has a non-finite angle {value!r}")
        out.append(to_rad(angle))
    return tuple(out)


def _coerce_fps(value) -> float:
    """Validate an optional fps, falling back to :data:`DEFAULT_FPS`."""
    if value is None:
        return DEFAULT_FPS
    try:
        fps = float(value)
    except (TypeError, ValueError) as e:
        raise SequenceError(f"Invalid fps {value!r}") from e
    if not math.isfinite(fps) or fps <= 0:
        raise SequenceError(f"fps must be a positive number, got {value!r}")
    return fps
