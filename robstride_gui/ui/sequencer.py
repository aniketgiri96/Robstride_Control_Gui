"""Plays a loaded :class:`~robstride_gui.sequence.Sequence` onto motors.

The player advances one frame per timer tick at the sequence's frame rate and
posts each channel's angle as a position setpoint to the worker. It deliberately
knows nothing about the worker's command types: motion is delivered through an
injected ``post(device_id, position_rad)`` callback, so the stepping logic is
fully testable by driving :meth:`SequencePlayer.tick` with a capturing sink and
no Qt event loop.

Transport model:

* :meth:`play` starts (or resumes) ticking from the current frame.
* :meth:`stop` halts playback but keeps the frame, so play resumes where it left
  off.
* :meth:`abort` halts and rewinds to the start - the "get me out now" control.
"""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import QObject, QTimer, Signal

from ..sequence import Sequence

#: Floor on the tick period (ms) so a pathological fps can't peg the event loop.
_MIN_TICK_MS = 5


class SequencePlayer(QObject):
    """Steps a sequence's frames onto motors via an injected command sink."""

    progress = Signal(int, int)     # current_frame, total_frames
    finished = Signal()             # reached the end (natural stop)
    stateChanged = Signal(bool)     # playing?
    lastAction = Signal(str)        # human-readable status for the UI log

    def __init__(self, post: Callable[[int, float], None],
                 parent: QObject | None = None):
        super().__init__(parent)
        self._post = post
        self._seq: Optional[Sequence] = None
        self._channel_map: dict[int, int] = {}   # channel index -> device_id
        self._frame = 0
        self._playing = False
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)

    # -- loading -----------------------------------------------------------------

    def load(self, seq: Sequence, channel_map: dict[int, int]) -> None:
        """Load ``seq`` and the channel->device mapping, rewound to the start.

        ``channel_map`` maps a sequence channel index to a motor's CAN id; only
        mapped channels are driven, so a 6-channel export can play on however
        many motors are actually connected.
        """
        self.abort()
        self._seq = seq
        self._channel_map = dict(channel_map)
        self.progress.emit(0, seq.frame_count)
        self.lastAction.emit(f"Loaded sequence: {seq.describe()}")

    @property
    def is_playing(self) -> bool:
        return self._playing

    @property
    def frame(self) -> int:
        return self._frame

    # -- transport ---------------------------------------------------------------

    def play(self) -> None:
        if self._seq is None or self._seq.frame_count == 0:
            self.lastAction.emit("No sequence loaded")
            return
        if self._playing:
            return
        # A sequence already at its end restarts from the top on Play.
        if self._frame >= self._seq.frame_count:
            self._frame = 0
        self._playing = True
        period = max(int(round(1000.0 / self._seq.fps)), _MIN_TICK_MS)
        self._timer.start(period)
        self.stateChanged.emit(True)
        self.lastAction.emit("Sequence playing")

    def stop(self) -> None:
        """Halt playback, keeping the current frame so Play resumes from here."""
        if not self._playing:
            return
        self._timer.stop()
        self._playing = False
        self.stateChanged.emit(False)
        self.lastAction.emit(f"Sequence stopped at frame {self._frame}")

    def abort(self) -> None:
        """Halt immediately and rewind to the start."""
        was_playing = self._playing
        self._timer.stop()
        self._playing = False
        self._frame = 0
        if self._seq is not None:
            self.progress.emit(0, self._seq.frame_count)
        if was_playing:
            self.stateChanged.emit(False)
            self.lastAction.emit("Sequence aborted")

    # -- stepping ----------------------------------------------------------------

    def tick(self) -> None:
        """Emit the current frame's angles and advance; stop at the end.

        Public (not ``_tick``) so tests can step deterministically without a
        real timer.
        """
        seq = self._seq
        if seq is None:
            return
        if self._frame >= seq.frame_count:
            self._finish()
            return
        for channel, device_id in self._channel_map.items():
            if 0 <= channel < seq.channel_count:
                self._post(device_id, seq.angle_at(self._frame, channel))
        self._frame += 1
        self.progress.emit(self._frame, seq.frame_count)
        if self._frame >= seq.frame_count:
            self._finish()

    def _finish(self) -> None:
        self._timer.stop()
        if self._playing:
            self._playing = False
            self.stateChanged.emit(False)
        self.finished.emit()
        self.lastAction.emit("Sequence finished")
