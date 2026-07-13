"""Visual angle indicator: a horizontal bar showing where a motor sits between
its min and max travel limits, with distinct markers for the *commanded* and
*actual* angle and a directional arrow while it is moving.

The geometry is split out as pure functions (:func:`marker_fraction`,
:func:`direction`) so the placement logic is unit-testable without a running Qt
paint pass. The widget itself just maps those fractions onto pixels.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPolygonF
from PySide6.QtWidgets import QWidget

#: Below this |velocity| (rad/s) the motor is treated as stopped: no arrow.
MOVING_EPS: float = 0.05

# Marker colours. Actual is the bright, trustworthy reading; commanded is a
# ghosted target so the two are unmistakable at a glance.
_ACTUAL_COLOR = QColor("#42a5f5")
_COMMANDED_COLOR = QColor("#ffb74d")
_TRACK_COLOR = QColor("#37474f")
_WARN_TRACK_COLOR = QColor("#5d4037")


def marker_fraction(value: float, lo: float, hi: float) -> float:
    """Map ``value`` onto ``[0, 1]`` across ``[lo, hi]``, clamped to the ends.

    A degenerate range (``hi <= lo``) collapses to the middle so the marker is
    still drawn somewhere sensible rather than dividing by zero.
    """
    span = hi - lo
    if span <= 0:
        return 0.5
    frac = (value - lo) / span
    return 0.0 if frac < 0.0 else 1.0 if frac > 1.0 else frac


def direction(velocity: float, eps: float = MOVING_EPS) -> int:
    """Sign of motion: ``+1`` increasing, ``-1`` decreasing, ``0`` stopped."""
    if velocity > eps:
        return 1
    if velocity < -eps:
        return -1
    return 0


class PositionBar(QWidget):
    """Paints the commanded/actual angle between ``lo`` and ``hi`` (rad)."""

    def __init__(self, lo: float, hi: float, parent: QWidget | None = None):
        super().__init__(parent)
        self._lo = lo
        self._hi = hi
        self._actual = 0.0
        self._commanded = 0.0
        self._direction = 0
        self._warn = False
        self.setMinimumHeight(22)
        self.setMinimumWidth(120)

    def set_limits(self, lo: float, hi: float) -> None:
        self._lo, self._hi = lo, hi
        self.update()

    def set_actual(self, value: float, velocity: float = 0.0) -> None:
        self._actual = value
        self._direction = direction(velocity)
        self.update()

    def set_commanded(self, value: float) -> None:
        self._commanded = value
        self.update()

    def set_warning(self, warn: bool) -> None:
        if warn != self._warn:
            self._warn = warn
            self.update()

    # -- painting ----------------------------------------------------------------

    def paintEvent(self, _event) -> None:  # pragma: no cover - visual only
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        mid = h / 2

        track = _WARN_TRACK_COLOR if self._warn else _TRACK_COLOR
        painter.setPen(Qt.NoPen)
        painter.setBrush(track)
        painter.drawRoundedRect(QRectF(0, mid - 4, w, 8), 4, 4)

        # Commanded marker (ghost, drawn first so actual sits on top).
        cx = marker_fraction(self._commanded, self._lo, self._hi) * w
        painter.setBrush(_COMMANDED_COLOR)
        painter.setOpacity(0.55)
        painter.drawEllipse(QRectF(cx - 5, mid - 5, 10, 10))

        # Actual marker.
        ax = marker_fraction(self._actual, self._lo, self._hi) * w
        painter.setOpacity(1.0)
        painter.setBrush(_ACTUAL_COLOR)
        painter.drawEllipse(QRectF(ax - 6, mid - 6, 12, 12))

        if self._direction != 0:
            self._draw_arrow(painter, ax, mid, self._direction)
        painter.end()

    def _draw_arrow(self, painter: QPainter, x: float, y: float, sign: int) -> None:  # pragma: no cover
        tip = x + sign * 14
        back = x + sign * 6
        arrow = QPolygonF([QPointF(tip, y), QPointF(back, y - 5), QPointF(back, y + 5)])
        painter.setBrush(_ACTUAL_COLOR)
        painter.drawPolygon(arrow)
