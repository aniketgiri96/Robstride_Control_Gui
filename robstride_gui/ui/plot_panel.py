"""Real-time rolling plots for one motor's telemetry.

A :class:`LivePlot` keeps a fixed-length ring buffer per trace and redraws on a
timer rather than on every sample, so a 100 Hz feedback stream never floods the
GUI thread. Falls back gracefully to a text label if pyqtgraph is unavailable.
"""

from __future__ import annotations

import collections
import time
from typing import Deque

from PySide6.QtWidgets import QVBoxLayout, QWidget, QLabel

try:
    import pyqtgraph as pg
    _HAVE_PG = True
except ImportError:  # pragma: no cover - optional dependency
    _HAVE_PG = False


TRACES = (
    ("position", "Position (rad)", (0, 200, 255)),
    ("velocity", "Velocity (rad/s)", (0, 230, 118)),
    ("torque", "Torque (Nm)", (255, 167, 38)),
    ("temperature", "Temp (C)", (239, 83, 80)),
)


class LivePlot(QWidget):
    """Stacked rolling plots of position/velocity/torque/temperature."""

    def __init__(self, window_seconds: float = 10.0, sample_hz: float = 100.0,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._maxlen = max(10, int(window_seconds * sample_hz))
        self._t0 = time.monotonic()
        self._t: Deque[float] = collections.deque(maxlen=self._maxlen)
        self._data: dict[str, Deque[float]] = {
            key: collections.deque(maxlen=self._maxlen) for key, _, _ in TRACES
        }
        self._curves: dict[str, object] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not _HAVE_PG:
            fallback = QLabel("Install pyqtgraph for live plots")
            fallback.setStyleSheet("color: #888; padding: 12px;")
            layout.addWidget(fallback)
            return

        pg.setConfigOptions(antialias=True)
        for key, label, color in TRACES:
            plot = pg.PlotWidget()
            plot.setBackground("#1e1e1e")
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setLabel("left", label)
            plot.setMouseEnabled(x=False, y=True)
            curve = plot.plot(pen=pg.mkPen(color=color, width=2))
            self._curves[key] = curve
            layout.addWidget(plot)

    def add_sample(self, position: float, velocity: float, torque: float,
                   temperature: float) -> None:
        """Append one feedback sample (cheap; drawing happens in :meth:`refresh`)."""
        self._t.append(time.monotonic() - self._t0)
        self._data["position"].append(position)
        self._data["velocity"].append(velocity)
        self._data["torque"].append(torque)
        self._data["temperature"].append(temperature)

    def refresh(self) -> None:
        """Redraw all traces from the current ring buffers."""
        if not _HAVE_PG or not self._t:
            return
        xs = list(self._t)
        for key, curve in self._curves.items():
            curve.setData(xs, list(self._data[key]))

    def clear(self) -> None:
        self._t.clear()
        for buf in self._data.values():
            buf.clear()
        self.refresh()
