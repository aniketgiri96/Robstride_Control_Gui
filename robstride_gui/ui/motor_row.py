"""Compact per-motor card for the 6-motor overview dashboard.

Unlike :class:`~robstride_gui.ui.motor_panel.MotorPanel` (the full single-motor
tab), this widget is built to sit six-up on one screen: a single always-visible
status line - current, torque, commanded-vs-actual angle, a min/max position bar,
and a warning border when current or torque nears its safety limit - over a
*Setup* section that stays collapsed until the operator expands it.

Setup holds the calibration (CV1/CV2 min/max, capture-current-position,
lock/unlock, explicit Save) and the position controls (manual target, two-point
A/B jog). It emits the same high-level signals as ``MotorPanel``
(``enableToggled``/``targetChanged``/``rangeLimitsEdited``/``modeChanged``) so
``MainWindow`` wires it to the worker with the existing handlers, plus
``lockChanged`` so the lock state can be persisted. The card never touches the
bus.
"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QPushButton, QToolButton, QVBoxLayout, QWidget,
)

from .. import protocol as proto
from ..protocol import MotorStatus, RunMode
from ..safety import SafetyLimits
from .position_bar import PositionBar, direction

#: Warn once current or torque reaches this fraction of its safety limit.
WARN_FRACTION: float = 0.8


def is_warning(current: float, torque: float,
               current_max: float | None, torque_max: float | None,
               fraction: float = WARN_FRACTION) -> bool:
    """True if |current| or |torque| has reached ``fraction`` of its limit.

    Pure so the threshold logic is unit-testable. A ``None`` limit disables that
    side's check (some models leave a cap unset).
    """
    if current_max is not None and abs(current) >= fraction * current_max:
        return True
    if torque_max is not None and abs(torque) >= fraction * torque_max:
        return True
    return False


class MotorRow(QFrame):
    """One motor's compact status + collapsible setup on the dashboard."""

    enableToggled = Signal(int, bool)          # device_id, enable
    targetChanged = Signal(int, object)        # device_id, dict of changed fields
    modeChanged = Signal(int, int)             # device_id, RunMode
    rangeLimitsEdited = Signal(int, object, object)  # device_id, min, max (rad|None)
    lockChanged = Signal(int, bool)            # device_id, locked

    def __init__(self, device_id: int, model: str = proto.DEFAULT_MODEL,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.device_id = device_id
        self.model = model
        self._limits = proto.model_limits(model)
        safety = SafetyLimits.for_model(model)
        self._current_max = safety.current_max
        self._torque_max = safety.torque_max

        span = self._limits["position"]
        self._pos_lo = -span
        self._pos_hi = span

        self._enabled = False
        self._locked = True          # calibration starts locked to avoid nudges
        self._actual = 0.0
        self._commanded = 0.0
        self._velocity = 0.0
        self._current = 0.0
        self._torque = 0.0
        self._warn = False

        # A/B two-point jog: a timer flips the target between A and B.
        self._ab_timer = QTimer(self)
        self._ab_timer.timeout.connect(self._ab_step)
        self._ab_at_a = True

        self.setFrameShape(QFrame.StyledPanel)
        self._build()
        self._apply_lock()
        self._refresh_border()

    # -- construction ------------------------------------------------------------

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(4)
        root.addLayout(self._build_status_line())
        root.addWidget(self._build_setup())

        self.action_lbl = QLabel("")
        self.action_lbl.setStyleSheet("color:#9e9e9e; font-size:11px;")
        root.addWidget(self.action_lbl)

    def _build_status_line(self) -> QHBoxLayout:
        lay = QHBoxLayout()
        lay.setSpacing(10)

        self.state_dot = QLabel("●")
        self.state_dot.setStyleSheet("color:#b71c1c; font-size:16px;")
        self.name_lbl = QLabel(f"<b>Motor {self.device_id}</b>")
        self.name_lbl.setMinimumWidth(70)

        self.current_lbl = QLabel("I: – A")
        self.current_lbl.setMinimumWidth(78)
        self.torque_lbl = QLabel("τ: – Nm")
        self.torque_lbl.setMinimumWidth(90)
        self.angle_lbl = QLabel("∠ –")
        self.angle_lbl.setMinimumWidth(150)
        self.angle_lbl.setToolTip("commanded → actual angle")

        self.bar = PositionBar(self._pos_lo, self._pos_hi)

        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setCheckable(True)
        self.enable_btn.setFixedWidth(72)
        self.enable_btn.clicked.connect(
            lambda checked: self.enableToggled.emit(self.device_id, checked))

        self.limp_btn = QPushButton("LIMP")
        self.limp_btn.setFixedWidth(56)
        self.limp_btn.setToolTip(
            "Back-drivable bring-up: MIT mode, zero Kp/Kd and assist torque, then "
            "enable. The motor reports its encoder while free to move by hand - "
            "for hand-teaching (the sim mirrors it) or moving through the travel "
            "during calibration.")
        self.limp_btn.clicked.connect(self._on_make_limp)

        self.setup_btn = QToolButton()
        self.setup_btn.setText("Setup")
        self.setup_btn.setCheckable(True)
        self.setup_btn.setArrowType(Qt.RightArrow)
        self.setup_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.setup_btn.toggled.connect(self._on_setup_toggled)

        lay.addWidget(self.state_dot)
        lay.addWidget(self.name_lbl)
        lay.addWidget(self.current_lbl)
        lay.addWidget(self.torque_lbl)
        lay.addWidget(self.angle_lbl)
        lay.addWidget(self.bar, 1)
        lay.addWidget(self.enable_btn)
        lay.addWidget(self.limp_btn)
        lay.addWidget(self.setup_btn)
        return lay

    def _build_setup(self) -> QWidget:
        self.setup_panel = QWidget()
        lay = QHBoxLayout(self.setup_panel)
        lay.setContentsMargins(0, 4, 0, 0)
        lay.addWidget(self._build_calibration_box(), 1)
        lay.addWidget(self._build_position_box(), 1)
        self.setup_panel.setVisible(False)  # collapsed by default
        return self.setup_panel

    def _build_calibration_box(self) -> QWidget:
        box = QFrame()
        box.setFrameShape(QFrame.StyledPanel)
        grid = QGridLayout(box)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.addWidget(QLabel("<b>Calibration</b>"), 0, 0, 1, 2)

        span_deg = math.degrees(self._limits["position"])
        self.min_spin = self._spin(-span_deg, span_deg, 1.0, " deg")
        self.min_spin.setValue(-span_deg)
        self.max_spin = self._spin(-span_deg, span_deg, 1.0, " deg")
        self.max_spin.setValue(span_deg)
        grid.addWidget(QLabel("Min (CV1)"), 1, 0)
        grid.addWidget(self.min_spin, 1, 1)
        grid.addWidget(QLabel("Max (CV2)"), 2, 0)
        grid.addWidget(self.max_spin, 2, 1)

        self.capture_min_btn = QPushButton("Capture → Min")
        self.capture_min_btn.setToolTip("Set Min to the current actual angle")
        self.capture_min_btn.clicked.connect(lambda: self._capture(self.min_spin))
        self.capture_max_btn = QPushButton("Capture → Max")
        self.capture_max_btn.setToolTip("Set Max to the current actual angle")
        self.capture_max_btn.clicked.connect(lambda: self._capture(self.max_spin))
        grid.addWidget(self.capture_min_btn, 3, 0)
        grid.addWidget(self.capture_max_btn, 3, 1)

        self.lock_btn = QPushButton("🔒 Locked")
        self.lock_btn.setCheckable(True)
        self.lock_btn.setChecked(True)
        self.lock_btn.setToolTip("Unlock to edit calibration; locked prevents "
                                 "accidental nudges during operation")
        self.lock_btn.toggled.connect(self._on_lock_toggled)
        self.save_btn = QPushButton("Save limits")
        self.save_btn.setToolTip("Apply the Min/Max above as the travel range")
        self.save_btn.clicked.connect(self._on_save_limits)
        grid.addWidget(self.lock_btn, 4, 0)
        grid.addWidget(self.save_btn, 4, 1)
        return box

    def _build_position_box(self) -> QWidget:
        box = QFrame()
        box.setFrameShape(QFrame.StyledPanel)
        form = QFormLayout(box)
        form.setContentsMargins(6, 6, 6, 6)
        form.addRow(QLabel("<b>Position</b>"))

        span_deg = math.degrees(self._limits["position"])
        self.target_spin = self._spin(-span_deg, span_deg, 1.0, " deg")
        send_btn = QPushButton("Send")
        send_btn.clicked.connect(self._on_send_target)
        target_row = QHBoxLayout()
        target_row.addWidget(self.target_spin, 1)
        target_row.addWidget(send_btn)
        target_w = QWidget()
        target_w.setLayout(target_row)
        form.addRow("Target", target_w)

        self.a_spin = self._spin(-span_deg, span_deg, 1.0, " deg")
        self.a_spin.setValue(-30.0)
        self.b_spin = self._spin(-span_deg, span_deg, 1.0, " deg")
        self.b_spin.setValue(30.0)
        self.interval_spin = self._spin(0.1, 60.0, 0.1, " s")
        self.interval_spin.setValue(1.0)
        form.addRow("A", self.a_spin)
        form.addRow("B", self.b_spin)
        form.addRow("Interval", self.interval_spin)

        self.ab_btn = QPushButton("Start A/B")
        self.ab_btn.setCheckable(True)
        self.ab_btn.setToolTip("Jog between A and B every interval (position mode)")
        self.ab_btn.toggled.connect(self._on_ab_toggled)
        form.addRow(self.ab_btn)
        return box

    def _spin(self, lo: float, hi: float, step: float, suffix: str) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(2)
        if suffix:
            s.setSuffix(suffix)
        s.setKeyboardTracking(False)
        return s

    # -- setup expand / lock -----------------------------------------------------

    def _on_setup_toggled(self, expanded: bool) -> None:
        self.setup_panel.setVisible(expanded)
        self.setup_btn.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)

    def _on_lock_toggled(self, locked: bool) -> None:
        self._locked = locked
        self._apply_lock()
        self.lockChanged.emit(self.device_id, locked)

    def _apply_lock(self) -> None:
        """Enable/disable the calibration edit widgets to match the lock."""
        editable = not self._locked
        for w in (self.min_spin, self.max_spin, self.capture_min_btn,
                  self.capture_max_btn, self.save_btn):
            w.setEnabled(editable)
        self.lock_btn.setText("🔒 Locked" if self._locked else "🔓 Unlocked")

    def set_locked(self, locked: bool) -> None:
        """Set the lock without re-emitting (e.g. restoring a persisted state)."""
        self.lock_btn.blockSignals(True)
        self.lock_btn.setChecked(locked)
        self.lock_btn.blockSignals(False)
        self._locked = locked
        self._apply_lock()

    # -- calibration -------------------------------------------------------------

    def _capture(self, spin: QDoubleSpinBox) -> None:
        """Copy the live actual angle into a min/max spin (no apply until Save)."""
        spin.setValue(math.degrees(self._actual))
        which = "Min" if spin is self.min_spin else "Max"
        self.set_action(f"Captured {which} = {math.degrees(self._actual):+.1f}°")

    def _on_save_limits(self) -> None:
        lo = math.radians(self.min_spin.value())
        hi = math.radians(self.max_spin.value())
        if lo > hi:
            lo, hi = hi, lo
        self.rangeLimitsEdited.emit(self.device_id, lo, hi)
        self.set_action(f"Saved limits [{math.degrees(lo):+.1f}, "
                        f"{math.degrees(hi):+.1f}]°")

    # -- position control --------------------------------------------------------

    def _on_send_target(self) -> None:
        self._command_position(math.radians(self.target_spin.value()))
        self.set_action(f"Target → {self.target_spin.value():+.1f}°")

    def _on_ab_toggled(self, active: bool) -> None:
        if active and not self._enabled:
            # No point cycling a disabled motor - the worker ignores it and the
            # timer would just fire into the void. Make the operator enable first.
            self.set_action("Enable the motor before A/B")
            self.set_ab_active(False)
            return
        self.ab_btn.setText("Stop A/B" if active else "Start A/B")
        if active:
            # Assert position mode once so the setpoints are acted on.
            self.modeChanged.emit(self.device_id, RunMode.POSITION_PP)
            self._ab_at_a = True
            self._ab_step()  # move immediately, then on the interval
            self._ab_timer.start(max(int(self.interval_spin.value() * 1000), 100))
            self.set_action("A/B cycling")
        else:
            self._ab_timer.stop()
            self.set_action("A/B stopped")

    def _ab_step(self) -> None:
        target_deg = self.a_spin.value() if self._ab_at_a else self.b_spin.value()
        self._command_position(math.radians(target_deg))
        self._ab_at_a = not self._ab_at_a

    def _command_position(self, rad: float) -> None:
        """Emit a position setpoint and record it as the commanded angle."""
        self.set_commanded(rad)
        self.targetChanged.emit(self.device_id, {"position": rad})

    def _on_make_limp(self) -> None:
        """One-click back-drivable bring-up for hand-teaching / calibration.

        Puts the motor into MIT mode with zero stiffness, damping and assist
        torque, then enables it, so it holds no position and can be moved freely
        by hand while still reporting its encoder. Emits the same signals as
        :meth:`MotorPanel._on_make_limp`; the dashboard forwards them to
        ``MainWindow``, so the command path and its guards (bus connected,
        zeroed-since-enable confirmation) are exactly the normal ones. The row
        has no gain widgets, so the zeroed gains are sent straight as a target.
        """
        self.modeChanged.emit(self.device_id, RunMode.MIT)
        self.targetChanged.emit(
            self.device_id, {"kp": 0.0, "kd": 0.0, "torque_ff": 0.0})
        # Enable last so the first MIT frame is already limp. Mirror a real Enable
        # click (setChecked doesn't emit clicked); MainWindow resets the button via
        # set_enabled_state if it rejects or the operator backs out.
        if not self.enable_btn.isChecked():
            self.enable_btn.setChecked(True)
            self.enableToggled.emit(self.device_id, True)
        self.set_action("LIMP: MIT Kp=Kd=0, enabled - free to move by hand")

    def set_ab_active(self, active: bool) -> None:
        """Reflect A/B on/off from outside without re-triggering the toggle."""
        self.ab_btn.blockSignals(True)
        self.ab_btn.setChecked(active)
        self.ab_btn.setText("Stop A/B" if active else "Start A/B")
        self.ab_btn.blockSignals(False)
        if not active:
            self._ab_timer.stop()

    @property
    def ab_active(self) -> bool:
        return self._ab_timer.isActive()

    # -- external updates --------------------------------------------------------

    def set_commanded(self, rad: float) -> None:
        self._commanded = rad
        self.bar.set_commanded(rad)
        self._refresh_angle_label()

    def update_status(self, status: MotorStatus) -> None:
        self._actual = status.position
        self._velocity = status.velocity
        self._torque = status.torque
        self.bar.set_actual(status.position, status.velocity)
        self.torque_lbl.setText(f"τ: {status.torque:+.2f} Nm")
        self._refresh_angle_label()
        self._refresh_warning()

    def update_power(self, power) -> None:
        self._current = power.iq
        self.current_lbl.setText(f"I: {power.iq:+.2f} A")
        self._refresh_warning()

    def set_enabled_state(self, enabled: bool) -> None:
        self._enabled = enabled
        self.enable_btn.blockSignals(True)
        self.enable_btn.setChecked(enabled)
        self.enable_btn.setText("Disable" if enabled else "Enable")
        self.enable_btn.blockSignals(False)
        self.state_dot.setStyleSheet(
            f"color:{'#66bb6a' if enabled else '#b71c1c'}; font-size:16px;")
        if enabled:
            # Come up holding the current spot: align the commanded ghost with it.
            self.set_commanded(self._actual)
        else:
            # A disabled motor is not cycling.
            self.set_ab_active(False)

    def set_position_limits(self, lo: float | None, hi: float | None) -> None:
        span = self._limits["position"]
        self._pos_lo = -span if lo is None else lo
        self._pos_hi = span if hi is None else hi
        if self._pos_lo > self._pos_hi:
            self._pos_lo, self._pos_hi = self._pos_hi, self._pos_lo
        self.bar.set_limits(self._pos_lo, self._pos_hi)
        lo_deg, hi_deg = math.degrees(self._pos_lo), math.degrees(self._pos_hi)
        # Reflect the calibrated limits into the CV1/CV2 editors.
        for spin, val in ((self.min_spin, lo_deg), (self.max_spin, hi_deg)):
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)
        # Constrain the motion inputs to the calibrated travel too, so the UI
        # cannot even request an angle past the ends (mirrors MotorPanel). The
        # worker still clamps as the authoritative backstop; this just keeps the
        # inputs honest and clamps any now-out-of-range value.
        for spin in (self.target_spin, self.a_spin, self.b_spin):
            spin.blockSignals(True)
            spin.setRange(lo_deg, hi_deg)
            spin.blockSignals(False)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def current_draw(self) -> float:
        """Magnitude of current draw (A), for the totals bar."""
        return abs(self._current)

    def torque(self) -> float:
        """Magnitude of torque (Nm), for the totals bar."""
        return abs(self._torque)

    def set_action(self, message: str) -> None:
        self.action_lbl.setText(message)

    # -- rendering helpers -------------------------------------------------------

    def _refresh_angle_label(self) -> None:
        arrow = {1: " →", -1: " ←", 0: ""}[direction(self._velocity)]
        self.angle_lbl.setText(
            f"∠ {math.degrees(self._commanded):+.1f}° → "
            f"{math.degrees(self._actual):+.1f}°{arrow}")

    def _refresh_warning(self) -> None:
        warn = is_warning(self._current, self._torque,
                          self._current_max, self._torque_max)
        if warn != self._warn:
            self._warn = warn
            self.bar.set_warning(warn)
            self._refresh_border()

    def _refresh_border(self) -> None:
        if self._warn:
            self.setStyleSheet(
                "MotorRow{border:2px solid #ef5350; border-radius:6px;}")
        else:
            self.setStyleSheet(
                "MotorRow{border:1px solid #455a64; border-radius:6px;}")
