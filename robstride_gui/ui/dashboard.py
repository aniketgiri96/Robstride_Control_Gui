"""The 6-motor overview screen.

Holds a persistent totals bar (Σ current, Σ torque, # active, # cycling) fixed at
the top, a scrollable column of :class:`~robstride_gui.ui.motor_row.MotorRow`
cards below it (six fit collapsed; the scroll only bites once a card's Setup is
expanded), and a screen-level sequence transport: import a Blender/animation
export, see its metadata, then Play / Stop / Abort it across the connected motors.

The dashboard re-emits each row's high-level signals unchanged, so
``MainWindow`` connects to the dashboard exactly as it does to a ``MotorPanel``.
Sequence playback reuses the same ``targetChanged`` path - the player posts a
position per mapped channel and the dashboard forwards it - so nothing here
talks to the bus directly.
"""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox, QFileDialog, QFrame, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QVBoxLayout, QWidget,
)

from .. import protocol as proto
from ..jointlog import load_joint_log
from ..protocol import RunMode
from ..sequence import SequenceError, load_sequence
from .motor_row import MotorRow
from .sequencer import SequencePlayer


class MotorDashboard(QWidget):
    """Single-screen overview + control for all connected motors."""

    enableToggled = Signal(int, bool)
    targetChanged = Signal(int, object)
    modeChanged = Signal(int, int)
    rangeLimitsEdited = Signal(int, object, object)
    lockChanged = Signal(int, bool)
    log = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.rows: dict[int, MotorRow] = {}
        self._seq = None
        self._channel_map: dict[int, int] = {}
        # Set by a joint-log import (channel -> fixed CAN id). When present it
        # overrides the ascending-id auto-map so joint 3 always drives id 3, etc.
        self._explicit_channel_map: dict[int, int] | None = None
        self.player = SequencePlayer(post=self._on_sequence_frame)
        self.player.lastAction.connect(self.log)
        self.player.progress.connect(self._on_seq_progress)
        self.player.stateChanged.connect(self._on_seq_state)
        self.player.finished.connect(self._recompute_totals)
        self._build()

    # -- construction ------------------------------------------------------------

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.addWidget(self._build_totals_bar())
        root.addWidget(self._build_sequence_bar())

        self._rows_container = QWidget()
        self._rows_layout = QVBoxLayout(self._rows_container)
        self._rows_layout.setContentsMargins(0, 0, 0, 0)
        self._rows_layout.setSpacing(6)
        self._rows_layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(self._rows_container)
        root.addWidget(scroll, 1)

    def _build_totals_bar(self) -> QWidget:
        bar = QFrame()
        bar.setFrameShape(QFrame.StyledPanel)
        bar.setStyleSheet("QFrame{background:#263238; border-radius:6px;}"
                          "QLabel{color:#eceff1;}")
        lay = QHBoxLayout(bar)
        self.total_current_lbl = QLabel("Σ current: – A")
        self.total_torque_lbl = QLabel("Σ torque: – Nm")
        self.active_lbl = QLabel("active: 0")
        self.cycling_lbl = QLabel("cycling: 0")
        for w in (self.total_current_lbl, self.total_torque_lbl,
                  self.active_lbl, self.cycling_lbl):
            w.setStyleSheet("font-weight:bold; padding:2px 12px;")
            lay.addWidget(w)
        lay.addStretch(1)
        return bar

    def _build_sequence_bar(self) -> QWidget:
        bar = QFrame()
        bar.setFrameShape(QFrame.StyledPanel)
        lay = QHBoxLayout(bar)
        lay.addWidget(QLabel("<b>Sequence</b>"))

        import_btn = QPushButton("Import…")
        import_btn.setToolTip("Load a Blender/animation export (JSON or CSV)")
        import_btn.clicked.connect(self._on_import_clicked)
        lay.addWidget(import_btn)

        joint_btn = QPushButton("Import joint log…")
        joint_btn.setToolTip("Replay a recorded joint telemetry log "
                             "(joints 1–6 → motor ids 1–6)")
        joint_btn.clicked.connect(self._on_import_joint_log_clicked)
        lay.addWidget(joint_btn)

        self.smooth_chk = QCheckBox("Smooth")
        self.smooth_chk.setChecked(True)
        self.smooth_chk.setToolTip(
            "Reconstruct continuous motion from a coarse (~5 Hz) hand-teach "
            "capture so the motors ramp smoothly instead of stepping. Applies "
            "to the next joint-log import; uncheck to play raw samples.")
        lay.addWidget(self.smooth_chk)

        self.seq_meta_lbl = QLabel("no sequence loaded")
        self.seq_meta_lbl.setStyleSheet("color:#9e9e9e;")
        lay.addWidget(self.seq_meta_lbl, 1)

        self.play_btn = QPushButton("▶ Play")
        self.play_btn.clicked.connect(self._on_play_clicked)
        self.stop_btn = QPushButton("⏸ Stop")
        self.stop_btn.clicked.connect(self.player.stop)
        self.abort_btn = QPushButton("⏹ Abort")
        self.abort_btn.setToolTip("Stop immediately and rewind to the start")
        self.abort_btn.clicked.connect(self.player.abort)
        for b in (self.play_btn, self.stop_btn, self.abort_btn):
            b.setEnabled(False)
            lay.addWidget(b)
        return bar

    # -- motors ------------------------------------------------------------------

    def add_motor(self, device_id: int, model: str = proto.DEFAULT_MODEL) -> MotorRow:
        if device_id in self.rows:
            return self.rows[device_id]
        row = MotorRow(device_id, model)
        row.enableToggled.connect(self.enableToggled)
        row.targetChanged.connect(self.targetChanged)
        row.modeChanged.connect(self.modeChanged)
        row.rangeLimitsEdited.connect(self.rangeLimitsEdited)
        row.lockChanged.connect(self.lockChanged)
        # Insert before the trailing stretch so rows stack top-down.
        self._rows_layout.insertWidget(self._rows_layout.count() - 1, row)
        self.rows[device_id] = row
        self._rebuild_channel_map()
        return row

    def remove_motor(self, device_id: int) -> None:
        row = self.rows.pop(device_id, None)
        if row is None:
            return
        self._rows_layout.removeWidget(row)
        row.deleteLater()
        self._rebuild_channel_map()
        self._recompute_totals()

    def _rebuild_channel_map(self) -> None:
        """Map sequence channel *i* to the i-th motor by ascending CAN id.

        A joint-log import pins channels to fixed CAN ids
        (:attr:`_explicit_channel_map`); that binding wins so joint 3 stays on
        motor id 3 no matter which other motors are connected.
        """
        if self._explicit_channel_map is not None:
            self._channel_map = dict(self._explicit_channel_map)
            return
        self._channel_map = {i: did for i, did in enumerate(sorted(self.rows))}

    # -- worker signal fan-in (called by MainWindow) -----------------------------

    def update_status(self, device_id: int, status) -> None:
        row = self.rows.get(device_id)
        if row is not None:
            row.update_status(status)
            self._recompute_totals()

    def update_power(self, device_id: int, power) -> None:
        row = self.rows.get(device_id)
        if row is not None:
            row.update_power(power)
            self._recompute_totals()

    def set_enabled_state(self, device_id: int, enabled: bool) -> None:
        row = self.rows.get(device_id)
        if row is not None:
            row.set_enabled_state(enabled)
            self._recompute_totals()

    def set_position_limits(self, device_id: int, lo, hi) -> None:
        row = self.rows.get(device_id)
        if row is not None:
            row.set_position_limits(lo, hi)

    def set_locked(self, device_id: int, locked: bool) -> None:
        row = self.rows.get(device_id)
        if row is not None:
            row.set_locked(locked)

    # -- totals ------------------------------------------------------------------

    def totals(self) -> dict:
        """Aggregate current/torque/active/cycling across the rows (testable)."""
        total_current = sum(r.current_draw() for r in self.rows.values())
        total_torque = sum(r.torque() for r in self.rows.values())
        active = sum(1 for r in self.rows.values() if r.enabled)
        cycling_ids = {did for did, r in self.rows.items() if r.ab_active}
        if self.player.is_playing:
            cycling_ids |= set(self._channel_map.values())
        return {"current": total_current, "torque": total_torque,
                "active": active, "cycling": len(cycling_ids)}

    def _recompute_totals(self) -> None:
        t = self.totals()
        self.total_current_lbl.setText(f"Σ current: {t['current']:.2f} A")
        self.total_torque_lbl.setText(f"Σ torque: {t['torque']:.2f} Nm")
        self.active_lbl.setText(f"active: {t['active']}")
        self.cycling_lbl.setText(f"cycling: {t['cycling']}")

    # -- sequence ----------------------------------------------------------------

    def _on_import_clicked(self) -> None:  # pragma: no cover - file dialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Import sequence", "",
            "Sequences (*.json *.csv);;All files (*)")
        if path:
            self.load_sequence_file(path)

    def load_sequence_file(self, path: str) -> None:
        """Load a sequence file, updating metadata and enabling the transport."""
        try:
            seq = load_sequence(path)
        except SequenceError as e:
            self._show_seq_error(f"Sequence import failed: {e}")
            return
        # An animation export maps to whatever motors are connected (ascending
        # id); drop any joint-log pin so it does not linger from a prior import.
        self._explicit_channel_map = None
        self._rebuild_channel_map()
        self._activate_sequence(seq, seq.describe())

    def _on_import_joint_log_clicked(self) -> None:  # pragma: no cover - dialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Import joint log", "", "Joint logs (*.csv);;All files (*)")
        if path:
            self.load_joint_log_file(path, smooth=self.smooth_chk.isChecked())

    def load_joint_log_file(self, path: str,
                            joints: tuple[int, ...] = (1, 2, 3, 4, 5, 6),
                            source: str = "pos", smooth: bool = True) -> None:
        """Load a recorded joint telemetry log, pinning joints to their CAN ids.

        Each ``revolute_<n>`` joint drives the motor with CAN id ``n``. The
        binding is stored as the explicit channel map so Play cannot re-order it.
        ``smooth`` reconstructs continuous motion from a coarse capture (see
        :func:`~robstride_gui.jointlog.load_joint_log`).
        """
        try:
            seq, channel_map = load_joint_log(
                path, joints=joints, source=source, smooth=smooth)
        except SequenceError as e:
            self._show_seq_error(f"Joint log import failed: {e}")
            return
        self._explicit_channel_map = channel_map
        self._channel_map = dict(channel_map)
        mapping = ", ".join(f"j{ch}→id{did}"
                            for ch, did in zip(seq.channels,
                                               channel_map.values()))
        smooth_tag = "smoothed" if smooth else "raw"
        self._activate_sequence(
            seq, f"{seq.describe()}  ({source}, {smooth_tag}: {mapping})")

    def _activate_sequence(self, seq, meta_text: str) -> None:
        """Common tail for both importers: load the player and light the transport."""
        self._seq = seq
        self.player.load(seq, self._channel_map)
        self.seq_meta_lbl.setText(meta_text)
        self.seq_meta_lbl.setStyleSheet("color:#66bb6a;")
        for b in (self.play_btn, self.stop_btn, self.abort_btn):
            b.setEnabled(True)

    def _show_seq_error(self, message: str) -> None:
        self.seq_meta_lbl.setText(message)
        self.seq_meta_lbl.setStyleSheet("color:#ef5350;")
        self.log.emit(message)

    def _on_play_clicked(self) -> None:
        if self._seq is None:
            return
        # Re-bind the mapping in case motors were added since import.
        self._rebuild_channel_map()
        # Only enabled motors can move; refuse to "play" into a set of disabled
        # motors, which would look like it is running while nothing happens.
        enabled_ids = [d for d in self._channel_map.values()
                       if d in self.rows and self.rows[d].enabled]
        if not enabled_ids:
            self.log.emit("Sequence: no enabled motors mapped — enable a motor "
                          "first, then Play")
            return
        self.player.load(self._seq, self._channel_map)
        # Assert position mode on the motors that will actually move.
        for device_id in enabled_ids:
            self.modeChanged.emit(device_id, RunMode.POSITION_PP)
        self.player.play()

    def _on_sequence_frame(self, device_id: int, position_rad: float) -> None:
        """Player callback: forward one channel's angle as a position target."""
        row = self.rows.get(device_id)
        if row is not None:
            row.set_commanded(position_rad)
            row.set_action("sequence")
        self.targetChanged.emit(device_id, {"position": position_rad})

    def _on_seq_progress(self, frame: int, total: int) -> None:
        if self._seq is not None:
            self.seq_meta_lbl.setText(f"{self._seq.describe()}  —  frame {frame}/{total}")

    def _on_seq_state(self, playing: bool) -> None:
        self.play_btn.setText("▶ Playing…" if playing else "▶ Play")
        self._recompute_totals()
