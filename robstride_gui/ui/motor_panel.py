"""Per-motor control + telemetry widget.

Emits high-level Qt signals (enable/disable, zero, mode change, target change)
that :class:`~robstride_gui.ui.main_window.MainWindow` wires to the worker. The
panel never touches the bus directly, keeping IO off the GUI thread.
"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QScrollArea, QSlider, QSplitter,
    QVBoxLayout, QWidget,
)

from .. import protocol as proto
from ..protocol import MotorStatus, RunMode
from .plot_panel import LivePlot

_POS_SLIDER_STEPS = 1000  # slider integer resolution across the position span

# Fixed display units: position in radians, velocity in RPM. The motor protocol
# speaks rad/s on the wire (see protocol.MODEL_VELOCITY_MAX), so velocity is
# converted only at the display boundary; commands stay in rad/s.
_RAD_S_TO_RPM = proto.RAD_S_TO_RPM
_RPM_TO_RAD_S = proto.RPM_TO_RAD_S


class MotorPanel(QWidget):
    """Controls and live readout for a single motor."""

    enableToggled = Signal(int, bool)     # device_id, enable
    zeroRequested = Signal(int)
    modeChanged = Signal(int, int)        # device_id, RunMode
    targetChanged = Signal(int, object)   # device_id, dict of changed fields
    calibrationChanged = Signal(int, int, float)  # device_id, direction, offset
    captureZeroRequested = Signal(int)    # device_id
    zeroStateRequested = Signal(int)      # device_id

    def __init__(self, device_id: int, model: str = proto.DEFAULT_MODEL,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.device_id = device_id
        self.model = model
        self._limits = proto.model_limits(model)
        self._enabled = False
        self._build()

    # -- construction ------------------------------------------------------------

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        left = QVBoxLayout()
        left.addWidget(self._build_header())
        left.addWidget(self._build_mode_box())
        left.addWidget(self._build_calibration_box())
        left.addWidget(self._build_target_box())
        left.addWidget(self._build_jog_box())
        left.addWidget(self._build_readout_box())
        left.addStretch(1)
        left_w = QWidget()
        left_w.setLayout(left)

        # Scroll the control column so it stays usable on small / short screens
        # instead of forcing a fixed width and clipping when the window shrinks.
        controls = QScrollArea()
        controls.setWidgetResizable(True)
        controls.setWidget(left_w)
        controls.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        controls.setMinimumWidth(240)

        self.plot = LivePlot(ranges=self._plot_ranges())

        # A splitter lets the user re-balance controls vs. graph on any device.
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(controls)
        splitter.addWidget(self.plot)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([360, 820])
        root.addWidget(splitter)

    def _plot_ranges(self) -> dict[str, tuple[float, float]]:
        """Fixed Y-axis ranges so the graph scales (and units) stay steady."""
        lim = self._limits
        pos = lim["position"]
        vel_rpm = lim["velocity"] * _RAD_S_TO_RPM
        tq = lim["torque"]
        return {
            "position": (-pos, pos),
            "velocity": (-vel_rpm, vel_rpm),
            "torque": (-tq, tq),
            "temperature": (0.0, 100.0),
        }

    def _build_header(self) -> QWidget:
        box = QGroupBox(f"Motor {self.device_id}  -  {self.model}")
        lay = QHBoxLayout(box)
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setCheckable(True)
        self.enable_btn.clicked.connect(self._on_enable_clicked)
        self.zero_btn = QPushButton("Set Zero")
        self.zero_btn.clicked.connect(lambda: self.zeroRequested.emit(self.device_id))
        self.state_dot = QLabel("●")
        self.state_dot.setStyleSheet("color: #b71c1c; font-size: 18px;")
        lay.addWidget(self.state_dot)
        lay.addWidget(self.enable_btn)
        lay.addWidget(self.zero_btn)
        lay.addStretch(1)
        return box

    def _build_mode_box(self) -> QWidget:
        box = QGroupBox("Mode")
        lay = QHBoxLayout(box)
        self.mode_combo = QComboBox()
        self._mode_values = [RunMode.MIT, RunMode.POSITION_PP,
                             RunMode.VELOCITY, RunMode.CURRENT]
        for m in self._mode_values:
            self.mode_combo.addItem(RunMode.NAMES[m], m)
        self.mode_combo.setCurrentIndex(1)  # Position
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        lay.addWidget(self.mode_combo)
        return box

    def _build_calibration_box(self) -> QWidget:
        box = QGroupBox("Calibration")
        form = QFormLayout(box)

        self.invert_check = QCheckBox("Invert direction")
        self.invert_check.toggled.connect(self._emit_calibration)
        form.addRow(self.invert_check)

        span = self._limits["position"]
        self.offset_spin = self._spin(-span, span, 0.01, " rad")
        self.offset_spin.valueChanged.connect(self._emit_calibration)
        form.addRow("Zero offset", self.offset_spin)

        self.capture_btn = QPushButton("Set zero here (current position)")
        self.capture_btn.clicked.connect(
            lambda: self.captureZeroRequested.emit(self.device_id))
        form.addRow(self.capture_btn)

        # "Does the motor remember its absolute zero?" - reads the motor's own
        # stored zero (mechOffset/zero_sta) back from its registers.
        self.check_zero_btn = QPushButton("Check motor's saved zero")
        self.check_zero_btn.setToolTip(
            "Read the motor's own stored zero (mechOffset / zero_sta) to confirm "
            "it remembers its absolute zero across power cycles.")
        self.check_zero_btn.clicked.connect(
            lambda: self.zeroStateRequested.emit(self.device_id))
        form.addRow(self.check_zero_btn)

        self.zero_state_lbl = QLabel("motor zero: unknown")
        self.zero_state_lbl.setStyleSheet("color: #9e9e9e;")
        form.addRow(self.zero_state_lbl)
        return box

    def _build_target_box(self) -> QWidget:
        box = QGroupBox("Targets")
        form = QFormLayout(box)

        # Position: slider + spinbox
        span = self._limits["position"]
        self.pos_spin = self._spin(-span, span, 0.01, " rad")
        self.pos_slider = QSlider(Qt.Horizontal)
        self.pos_slider.setRange(-_POS_SLIDER_STEPS, _POS_SLIDER_STEPS)
        self.pos_slider.valueChanged.connect(self._on_pos_slider)
        self.pos_spin.valueChanged.connect(self._on_pos_spin)
        pos_row = QVBoxLayout()
        pos_row.addWidget(self.pos_spin)
        pos_row.addWidget(self.pos_slider)
        pos_w = QWidget()
        pos_w.setLayout(pos_row)
        form.addRow("Position", pos_w)

        vmax_rpm = self._limits["velocity"] * _RAD_S_TO_RPM
        self.vel_spin = self._spin(-vmax_rpm, vmax_rpm, 0.5, " rpm")
        self.vel_spin.valueChanged.connect(
            lambda v: self.targetChanged.emit(
                self.device_id, {"velocity": v * _RPM_TO_RAD_S}))
        form.addRow("Velocity", self.vel_spin)

        self.cur_spin = self._spin(-30.0, 30.0, 0.1, " A")
        self.cur_spin.valueChanged.connect(
            lambda v: self.targetChanged.emit(self.device_id, {"current": v}))
        form.addRow("Current", self.cur_spin)

        self.kp_spin = self._spin(0.0, self._limits["kp"], 0.5, "")
        self.kp_spin.setValue(28.0)
        self.kp_spin.valueChanged.connect(
            lambda v: self.targetChanged.emit(self.device_id, {"kp": v}))
        form.addRow("Kp", self.kp_spin)

        self.kd_spin = self._spin(0.0, self._limits["kd"], 0.1, "")
        self.kd_spin.setValue(6.0)
        self.kd_spin.valueChanged.connect(
            lambda v: self.targetChanged.emit(self.device_id, {"kd": v}))
        form.addRow("Kd", self.kd_spin)

        # Assist / feed-forward torque (MIT only). Injects a constant torque so a
        # geared motor can be made to *feel* backdrivable at Kp=Kd=0; safety caps
        # it at torque_max regardless of the slider range.
        tq = self._limits["torque"]
        self.tq_spin = self._spin(-tq, tq, 0.1, " Nm")
        self.tq_slider = QSlider(Qt.Horizontal)
        self.tq_slider.setRange(-_POS_SLIDER_STEPS, _POS_SLIDER_STEPS)
        self.tq_slider.valueChanged.connect(self._on_tq_slider)
        self.tq_spin.valueChanged.connect(self._on_tq_spin)
        tq_row = QVBoxLayout()
        tq_row.addWidget(self.tq_spin)
        tq_row.addWidget(self.tq_slider)
        tq_w = QWidget()
        tq_w.setLayout(tq_row)
        form.addRow("Assist τ", tq_w)

        self._update_field_enablement(RunMode.POSITION_PP)
        return box

    def _build_jog_box(self) -> QWidget:
        """Press-and-hold rotate control.

        Holding a button switches the motor to velocity mode and drives it at
        the jog speed; CCW is positive velocity, CW is negative. Releasing the
        button commands zero velocity so the motor coasts to a stop. The motor
        still has to be enabled for the worker to act on these commands.
        """
        box = QGroupBox("Jog (rotate)")
        lay = QHBoxLayout(box)

        self.jog_speed_spin = self._spin(
            0.0, self._limits["velocity"] * _RAD_S_TO_RPM, 1.0, " rpm")
        self.jog_speed_spin.setValue(10.0)

        self.jog_ccw_btn = QPushButton("↺ CCW")
        self.jog_ccw_btn.setToolTip("Hold to rotate counter-clockwise (+velocity)")
        self.jog_cw_btn = QPushButton("CW ↻")
        self.jog_cw_btn.setToolTip("Hold to rotate clockwise (-velocity)")
        self.jog_ccw_btn.pressed.connect(lambda: self._start_jog(1))
        self.jog_cw_btn.pressed.connect(lambda: self._start_jog(-1))
        self.jog_ccw_btn.released.connect(self._stop_jog)
        self.jog_cw_btn.released.connect(self._stop_jog)

        lay.addWidget(QLabel("Speed"))
        lay.addWidget(self.jog_speed_spin)
        lay.addWidget(self.jog_cw_btn)
        lay.addWidget(self.jog_ccw_btn)
        return box

    def _build_readout_box(self) -> QWidget:
        box = QGroupBox("Feedback")
        grid = QGridLayout(box)
        self.read_pos = QLabel("-")
        self.read_vel = QLabel("-")
        self.read_tq = QLabel("-")
        self.read_temp = QLabel("-")
        self.read_volt = QLabel("-")
        self.read_cur = QLabel("-")
        self.read_pwr = QLabel("-")
        self.read_fault = QLabel("OK")
        self.read_fault.setStyleSheet("color: #66bb6a;")
        rows = [("Position", self.read_pos), ("Velocity", self.read_vel),
                ("Torque", self.read_tq), ("Temp", self.read_temp),
                ("Voltage", self.read_volt), ("Current", self.read_cur),
                ("Power", self.read_pwr), ("Status", self.read_fault)]
        for i, (label, w) in enumerate(rows):
            grid.addWidget(QLabel(label), i, 0)
            w.setAlignment(Qt.AlignRight)
            grid.addWidget(w, i, 1)
        return box

    # -- small factory -----------------------------------------------------------

    def _spin(self, lo: float, hi: float, step: float, suffix: str) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(3)
        if suffix:
            s.setSuffix(suffix)
        s.setKeyboardTracking(False)
        return s

    # -- signal handlers ---------------------------------------------------------

    def _on_enable_clicked(self, checked: bool) -> None:
        self.enableToggled.emit(self.device_id, checked)

    def _emit_calibration(self, *_args) -> None:
        direction = -1 if self.invert_check.isChecked() else 1
        self.calibrationChanged.emit(self.device_id, direction, self.offset_spin.value())

    def set_calibration_display(self, direction: int, offset: float) -> None:
        """Reflect a calibration update (e.g. from 'set zero here') without
        re-emitting the change signal."""
        self.invert_check.blockSignals(True)
        self.offset_spin.blockSignals(True)
        self.invert_check.setChecked(direction < 0)
        self.offset_spin.setValue(offset)
        self.invert_check.blockSignals(False)
        self.offset_spin.blockSignals(False)

    def set_zero_state_display(self, info) -> None:
        """Show the motor's own persisted zero markers (worker.ZeroStateInfo)."""
        offset = "?" if info.mech_offset is None else f"{info.mech_offset:+.3f} rad"
        sta = "?" if info.zero_sta is None else str(info.zero_sta)
        self.zero_state_lbl.setText(f"motor zero: offset {offset} (zero_sta {sta})")
        self.zero_state_lbl.setStyleSheet("color: #66bb6a;")

    def _on_mode_changed(self, _index: int) -> None:
        mode = self.mode_combo.currentData()
        self._update_field_enablement(mode)
        self.modeChanged.emit(self.device_id, mode)

    def _on_pos_slider(self, value: int) -> None:
        span = self._limits["position"]
        rad = (value / _POS_SLIDER_STEPS) * span
        if abs(rad - self.pos_spin.value()) > 1e-6:
            self.pos_spin.blockSignals(True)
            self.pos_spin.setValue(rad)
            self.pos_spin.blockSignals(False)
        self.targetChanged.emit(self.device_id, {"position": rad})

    def _on_pos_spin(self, value: float) -> None:
        span = self._limits["position"]
        slider_val = int((value / span) * _POS_SLIDER_STEPS) if span else 0
        if slider_val != self.pos_slider.value():
            self.pos_slider.blockSignals(True)
            self.pos_slider.setValue(slider_val)
            self.pos_slider.blockSignals(False)
        self.targetChanged.emit(self.device_id, {"position": value})

    def _on_tq_slider(self, value: int) -> None:
        span = self._limits["torque"]
        nm = (value / _POS_SLIDER_STEPS) * span
        if abs(nm - self.tq_spin.value()) > 1e-6:
            self.tq_spin.blockSignals(True)
            self.tq_spin.setValue(nm)
            self.tq_spin.blockSignals(False)
        self.targetChanged.emit(self.device_id, {"torque_ff": nm})

    def _on_tq_spin(self, value: float) -> None:
        span = self._limits["torque"]
        slider_val = int((value / span) * _POS_SLIDER_STEPS) if span else 0
        if slider_val != self.tq_slider.value():
            self.tq_slider.blockSignals(True)
            self.tq_slider.setValue(slider_val)
            self.tq_slider.blockSignals(False)
        self.targetChanged.emit(self.device_id, {"torque_ff": value})

    def _start_jog(self, sign: int) -> None:
        """Switch to velocity mode (so UI and worker stay in sync) and drive
        the motor at the jog speed in the requested direction."""
        idx = self.mode_combo.findData(RunMode.VELOCITY)
        if idx >= 0 and self.mode_combo.currentIndex() != idx:
            self.mode_combo.setCurrentIndex(idx)  # emits modeChanged
        self.vel_spin.setValue(sign * self.jog_speed_spin.value())  # emits targetChanged

    def _stop_jog(self) -> None:
        self.vel_spin.setValue(0.0)

    def _update_field_enablement(self, mode: int) -> None:
        is_pos = mode in (RunMode.POSITION_PP, RunMode.POSITION_CSP, RunMode.MIT)
        is_vel = mode in (RunMode.VELOCITY, RunMode.MIT)
        is_cur = mode == RunMode.CURRENT
        is_mit = mode == RunMode.MIT
        self.pos_spin.setEnabled(is_pos)
        self.pos_slider.setEnabled(is_pos)
        self.vel_spin.setEnabled(is_vel)
        self.cur_spin.setEnabled(is_cur)
        self.kp_spin.setEnabled(is_mit)
        self.kd_spin.setEnabled(is_mit)
        self.tq_spin.setEnabled(is_mit)
        self.tq_slider.setEnabled(is_mit)

    # -- external updates --------------------------------------------------------

    def set_enabled_state(self, enabled: bool) -> None:
        self._enabled = enabled
        self.enable_btn.blockSignals(True)
        self.enable_btn.setChecked(enabled)
        self.enable_btn.setText("Disable" if enabled else "Enable")
        self.enable_btn.blockSignals(False)
        self.state_dot.setStyleSheet(
            "color: #66bb6a; font-size: 18px;" if enabled
            else "color: #b71c1c; font-size: 18px;")

    def update_status(self, status: MotorStatus) -> None:
        self.read_pos.setText(
            f"{status.position:+.3f} rad ({math.degrees(status.position):+.1f} deg)")
        self.read_vel.setText(
            f"{status.velocity * _RAD_S_TO_RPM:+.1f} rpm "
            f"({status.velocity:+.3f} rad/s)")
        self.read_tq.setText(f"{status.torque:+.3f} Nm")
        self.read_temp.setText(f"{status.temperature:.1f} C")
        if status.has_fault:
            faults = [name for name, flag in (
                ("stall", status.stalled), ("encoder", status.encoder_fault),
                ("overtemp", status.overtemperature), ("overcur", status.overcurrent),
                ("undervolt", status.undervoltage)) if flag]
            self.read_fault.setText("FAULT: " + ", ".join(faults))
            self.read_fault.setStyleSheet("color: #ef5350; font-weight: bold;")
        else:
            self.read_fault.setText("OK")
            self.read_fault.setStyleSheet("color: #66bb6a;")
        self.plot.add_sample(status.position, status.velocity * _RAD_S_TO_RPM,
                             status.torque, status.temperature)

    def update_power(self, power) -> None:
        """Show the board's electrical telemetry (worker.PowerInfo)."""
        self.read_volt.setText(f"{power.vbus:.2f} V")
        self.read_cur.setText(f"{power.iq:+.2f} A")
        self.read_pwr.setText(f"{power.power:+.2f} W")
