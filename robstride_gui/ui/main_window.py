"""Top-level application window.

Owns the worker thread, the connection bar, the per-motor tabs, the global
E-stop, presets, and the log. The window only ever *posts commands* to the
worker and *reacts to signals* from it - it never performs IO itself.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QThread, QTimer, Slot
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QComboBox, QDockWidget, QHBoxLayout, QInputDialog, QLabel,
    QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSpinBox, QTabWidget,
    QVBoxLayout, QWidget,
)

from .. import protocol as proto
from ..bus import Motor
from ..presets import Preset, PresetStore
from ..protocol import MotorStatus
from ..transport import (
    SerialATTransport, SocketCANTransport,
    auto_detect_serial_port, list_serial_port_details,
)
from .. import worker as wk
from .device_dialog import DeviceDialog
from .motor_panel import MotorPanel

PLOT_REFRESH_MS = 33  # ~30 FPS


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RobStride Control")
        self.resize(1180, 780)

        self.panels: dict[int, MotorPanel] = {}
        self.presets = PresetStore().load()
        self._connected = False
        self.device_dialog: DeviceDialog | None = None

        self._start_worker()
        self._build_ui()
        self._build_shortcuts()
        self._refresh_serial_ports()
        self._reload_preset_combo()

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start(PLOT_REFRESH_MS)

    # -- worker / thread ---------------------------------------------------------

    def _start_worker(self) -> None:
        self.thread = QThread(self)
        self.worker = wk.ControlWorker()
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)

        self.worker.statusUpdated.connect(self._on_status)
        self.worker.connectionChanged.connect(self._on_connection_changed)
        self.worker.scanFinished.connect(self._on_scan_finished)
        self.worker.busCollision.connect(self._on_bus_collision)
        self.worker.inventoryReady.connect(self._on_inventory_ready)
        self.worker.motorEnabledChanged.connect(self._on_motor_enabled)
        self.worker.calibrationChanged.connect(self._on_calibration_changed)
        self.worker.motorIdChanged.connect(self._on_motor_id_changed)
        self.worker.log.connect(self._append_log)
        self.worker.error.connect(self._on_error)

        self.thread.start()

    # -- UI construction ---------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.addWidget(self._build_connection_bar())

        self.tabs = QTabWidget()
        self.tabs.setMovable(True)
        root.addWidget(self.tabs, 1)

        self.setCentralWidget(central)
        self._build_log_dock()
        self._build_preset_dock()

    def _build_shortcuts(self) -> None:
        # F11 toggles full screen; Esc leaves it. Both are no-ops otherwise.
        QShortcut(QKeySequence(Qt.Key_F11), self, activated=self._toggle_fullscreen)
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._exit_fullscreen)

    def _toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def _exit_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()

    def _build_connection_bar(self) -> QWidget:
        bar = QWidget()
        lay = QHBoxLayout(bar)

        self.transport_combo = QComboBox()
        self.transport_combo.addItems(["Serial (AT)", "SocketCAN"])
        self.transport_combo.currentIndexChanged.connect(self._on_transport_changed)
        lay.addWidget(QLabel("Transport"))
        lay.addWidget(self.transport_combo)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(160)
        lay.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setToolTip("Re-scan for available serial ports")
        self.refresh_btn.clicked.connect(self._refresh_serial_ports)
        lay.addWidget(self.refresh_btn)

        self.port_status = QLabel()
        self.port_status.setToolTip("Whether a usable port was auto-detected")
        lay.addWidget(self.port_status)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connection)
        lay.addWidget(self.connect_btn)

        lay.addSpacing(16)
        lay.addWidget(QLabel("Add motor id"))
        self.add_id_spin = QSpinBox()
        self.add_id_spin.setRange(0, 127)  # id 0 is valid; motors ship/reset to it
        self.add_id_spin.setValue(0)
        lay.addWidget(self.add_id_spin)
        self.model_combo = QComboBox()
        self.model_combo.addItems(list(proto.MODELS))
        self.model_combo.setCurrentText(proto.DEFAULT_MODEL)
        lay.addWidget(self.model_combo)
        self.add_btn = QPushButton("Add")
        self.add_btn.clicked.connect(self._on_add_motor_clicked)
        lay.addWidget(self.add_btn)

        lay.addWidget(QLabel("Scan"))
        self.scan_start_spin = QSpinBox()
        self.scan_start_spin.setRange(0, 127)  # id 0 is valid; some motors ship/reset to it
        self.scan_start_spin.setValue(0)
        self.scan_end_spin = QSpinBox()
        self.scan_end_spin.setRange(0, 127)
        self.scan_end_spin.setValue(127)  # cover the full id space; motors ship/reset to 127
        lay.addWidget(self.scan_start_spin)
        lay.addWidget(QLabel("to"))
        lay.addWidget(self.scan_end_spin)
        self.scan_btn = QPushButton("Scan")
        self.scan_btn.clicked.connect(self._on_scan_clicked)
        lay.addWidget(self.scan_btn)

        self.setid_btn = QPushButton("Set ID...")
        self.setid_btn.setToolTip("Reassign a motor's CAN id (one motor on the bus)")
        self.setid_btn.clicked.connect(self._on_set_id_clicked)
        lay.addWidget(self.setid_btn)

        self.devices_btn = QPushButton("Devices...")
        self.devices_btn.setToolTip("List motors with their unique ids and assign CAN ids")
        self.devices_btn.clicked.connect(self._on_devices_clicked)
        lay.addWidget(self.devices_btn)

        lay.addStretch(1)

        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setCheckable(True)
        self.estop_btn.setMinimumWidth(120)
        self.estop_btn.setStyleSheet(
            "QPushButton{background:#b71c1c;color:white;font-weight:bold;}"
            "QPushButton:checked{background:#ff5252;}")
        self.estop_btn.clicked.connect(self._on_estop)
        lay.addWidget(self.estop_btn)
        return bar

    def _build_log_dock(self) -> None:
        self.log_dock = QDockWidget("Log", self)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        # Without a minimum height Qt collapses the bottom dock to a thin sliver,
        # so the log is effectively invisible. Give it a usable floor and a
        # sensible initial size.
        self.log_view.setMinimumHeight(120)
        self.log_dock.setWidget(self.log_view)
        self.log_dock.setMinimumHeight(140)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        self.resizeDocks([self.log_dock], [220], Qt.Vertical)

    def _build_preset_dock(self) -> None:
        dock = QDockWidget("Presets", self)
        w = QWidget()
        lay = QVBoxLayout(w)
        self.preset_combo = QComboBox()
        lay.addWidget(self.preset_combo)
        apply_btn = QPushButton("Apply to current motor")
        apply_btn.clicked.connect(self._apply_preset)
        save_btn = QPushButton("Save current as preset...")
        save_btn.clicked.connect(self._save_preset)
        del_btn = QPushButton("Delete preset")
        del_btn.clicked.connect(self._delete_preset)
        for b in (apply_btn, save_btn, del_btn):
            lay.addWidget(b)
        lay.addStretch(1)
        dock.setWidget(w)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

    # -- connection bar logic ----------------------------------------------------

    def _is_serial(self) -> bool:
        return self.transport_combo.currentIndex() == 0

    def _on_transport_changed(self, _index: int) -> None:
        self.port_combo.clear()
        if self._is_serial():
            self._refresh_serial_ports()
        else:
            self.port_combo.addItems(["can0", "can1"])
            self.port_status.setText("")
            self._set_connect_enabled(True)

    def _refresh_serial_ports(self) -> None:
        if not self._is_serial():
            return
        self.port_combo.clear()
        ports = list_serial_port_details()
        for info in ports:
            # show a friendly label, but keep the raw device path as the value
            self.port_combo.addItem(info.label(), info.device)

        if ports:
            auto = auto_detect_serial_port()
            idx = self.port_combo.findData(auto)
            self.port_combo.setCurrentIndex(max(idx, 0))
            self._set_port_status(available=True, count=len(ports))
            self._set_connect_enabled(True)
        else:
            # No fabricated default: be honest that nothing is plugged in.
            self._set_port_status(available=False, count=0)
            self._set_connect_enabled(False)

    def _set_port_status(self, *, available: bool, count: int) -> None:
        if available:
            self.port_status.setText(f"✓ {count} port{'s' if count != 1 else ''}")
            self.port_status.setStyleSheet("color:#2e7d32;")  # green
        else:
            self.port_status.setText("✗ no port")
            self.port_status.setStyleSheet("color:#b71c1c;")  # red

    def _set_connect_enabled(self, enabled: bool) -> None:
        # Never block a Disconnect; only gate the initial Connect.
        if not self._connected:
            self.connect_btn.setEnabled(enabled)

    def _current_port(self) -> str:
        data = self.port_combo.currentData()
        if data:
            return str(data)
        return self.port_combo.currentText().strip()

    def _build_transport(self):
        target = self._current_port()
        if self._is_serial():
            if not target:
                raise RuntimeError(
                    "No serial port available. Plug in the USB-CAN adapter and "
                    "press Refresh.")
            return SerialATTransport(target)
        return SocketCANTransport(target or "can0")

    def _toggle_connection(self) -> None:
        if self._connected:
            self.worker.post(wk.Disconnect())
            return
        if not self.panels:
            self._add_motor(self.add_id_spin.value(), self.model_combo.currentText())
        motors = [Motor(device_id=did, model=p.model) for did, p in self.panels.items()]
        try:
            transport = self._build_transport()
        except Exception as e:
            self._on_error(str(e))
            return
        self.worker.post(wk.Connect(transport=transport, motors=motors))

    # -- motors ------------------------------------------------------------------

    def _on_add_motor_clicked(self) -> None:
        self._add_motor(self.add_id_spin.value(), self.model_combo.currentText())

    def _on_scan_clicked(self) -> None:
        start = self.scan_start_spin.value()
        end = max(start, self.scan_end_spin.value())
        self.worker.post(wk.Scan(start=start, end=end))

    def _on_devices_clicked(self) -> None:
        if not self._connected:
            self._on_error("Connect to the bus before listing devices")
            return
        if self.device_dialog is None:
            dialog = DeviceDialog(self)
            dialog.detectRequested.connect(self._request_inventory)
            dialog.assignRequested.connect(self._on_device_assign)
            self.device_dialog = dialog
        self.device_dialog.show()
        self.device_dialog.raise_()
        self.device_dialog.activateWindow()
        self._request_inventory()

    def _request_inventory(self) -> None:
        start = self.scan_start_spin.value()
        end = max(start, self.scan_end_spin.value())
        self.worker.post(wk.Inventory(start=start, end=end))

    def _on_device_assign(self, current_id: int, new_id: int) -> None:
        self.worker.post(wk.SetMotorId(current_id=current_id, new_id=new_id))
        # Re-detect so the table reflects the new id after the motor acks.
        self._request_inventory()

    @Slot(list)
    def _on_inventory_ready(self, items: list) -> None:
        if self.device_dialog is not None:
            self.device_dialog.populate(items)
        # Detect alone now surfaces every motor: add a tab per responding CAN id.
        # _add_motor is idempotent (an already-tabbed id is re-focused, not
        # duplicated), so re-running Detect never creates duplicate tabs.
        model = self.model_combo.currentText()
        for entry in items:
            try:
                can_id = int(entry["can_id"])
            except (KeyError, TypeError, ValueError):
                continue
            self._add_motor(can_id, model)

    def _on_set_id_clicked(self) -> None:
        if not self._connected:
            self._on_error("Connect to the bus before changing a motor id")
            return
        cur, ok = QInputDialog.getInt(self, "Set Motor ID", "Current CAN id:",
                                      1, 1, 127)
        if not ok:
            return
        new, ok = QInputDialog.getInt(self, "Set Motor ID", "New CAN id:",
                                      cur, 1, 127)
        if not ok or new == cur:
            return
        confirm = QMessageBox.question(
            self, "Set Motor ID",
            f"Reassign motor {cur} -> {new}?\n\n"
            "Only ONE motor should be on the bus, or every motor sharing id "
            f"{cur} will be changed. Power-cycle afterwards to confirm it sticks.")
        if confirm == QMessageBox.Yes:
            self.worker.post(wk.SetMotorId(current_id=cur, new_id=new))

    def _remove_panel(self, device_id: int) -> None:
        panel = self.panels.pop(device_id, None)
        if panel is None:
            return
        idx = self.tabs.indexOf(panel)
        if idx >= 0:
            self.tabs.removeTab(idx)
        panel.deleteLater()

    def _add_motor(self, device_id: int, model: str) -> MotorPanel:
        if device_id in self.panels:
            self.tabs.setCurrentWidget(self.panels[device_id])
            return self.panels[device_id]
        panel = MotorPanel(device_id, model)
        panel.enableToggled.connect(self._on_enable_toggled)
        panel.zeroRequested.connect(lambda did: self.worker.post(wk.SetZero(did)))
        panel.modeChanged.connect(lambda did, m: self.worker.post(wk.SetMode(did, m)))
        panel.targetChanged.connect(self._on_target_changed)
        panel.calibrationChanged.connect(
            lambda did, d, o: self.worker.post(wk.SetCalibration(did, d, o)))
        panel.captureZeroRequested.connect(
            lambda did: self.worker.post(wk.CaptureZero(did)))
        self.panels[device_id] = panel
        self.tabs.addTab(panel, f"Motor {device_id}")
        self.tabs.setCurrentWidget(panel)
        self._append_log(f"Added motor {device_id} ({model})")
        # A motor added after Connect must be registered with the worker too,
        # else it gets a tab but is never polled/driven (Connect only seeds the
        # motors open at connect time).
        if self._connected:
            self.worker.post(wk.AddMotor(device_id=device_id, model=model))
        return panel

    @Slot(int, bool)
    def _on_enable_toggled(self, device_id: int, enable: bool) -> None:
        if not self._connected:
            self._on_error("Connect to the bus before enabling a motor")
            self.panels[device_id].set_enabled_state(False)
            return
        self.worker.post(wk.Enable(device_id) if enable else wk.Disable(device_id))

    @Slot(int, object)
    def _on_target_changed(self, device_id: int, changes: dict) -> None:
        self.worker.post(wk.SetTarget(device_id=device_id, **changes))

    # -- worker signal slots -----------------------------------------------------

    @Slot(int, object)
    def _on_status(self, device_id: int, status: MotorStatus) -> None:
        panel = self.panels.get(device_id)
        if panel is not None:
            panel.update_status(status)

    @Slot(bool)
    def _on_connection_changed(self, connected: bool) -> None:
        self._connected = connected
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.connect_btn.setEnabled(True)
        self.transport_combo.setEnabled(not connected)
        self.port_combo.setEnabled(not connected)
        self.refresh_btn.setEnabled(not connected)
        if not connected:
            # Re-check availability now that the port is free again.
            self._refresh_serial_ports()

    @Slot(list)
    def _on_scan_finished(self, ids: list) -> None:
        for device_id in ids:
            self._add_motor(int(device_id), proto.DEFAULT_MODEL)

    @Slot(list)
    def _on_bus_collision(self, ids: list) -> None:
        id_str = ", ".join(str(i) for i in ids)
        msg = (
            f"CAN id collision detected on id {id_str}.\n\n"
            "More than one motor is answering on the same CAN id, so every "
            "command drives them together — they cannot be controlled "
            "independently.\n\n"
            "To fix:\n"
            "1. Leave only ONE of the colliding motors powered on the bus.\n"
            "2. Use 'Set ID...' to give it a unique CAN id.\n"
            "3. Power-cycle that motor, then reconnect the others.")
        self._append_log(f"WARNING: CAN id collision on {id_str} - "
                         "multiple motors share an id and move together")
        QMessageBox.warning(self, "Bus ID Collision", msg)

    @Slot(int, bool)
    def _on_motor_enabled(self, device_id: int, enabled: bool) -> None:
        panel = self.panels.get(device_id)
        if panel is not None:
            panel.set_enabled_state(enabled)

    @Slot(int, int, float)
    def _on_calibration_changed(self, device_id: int, direction: int, offset: float) -> None:
        panel = self.panels.get(device_id)
        if panel is not None:
            panel.set_calibration_display(direction, offset)

    @Slot(int, int)
    def _on_motor_id_changed(self, old_id: int, new_id: int) -> None:
        model = self.panels[old_id].model if old_id in self.panels else proto.DEFAULT_MODEL
        self._remove_panel(old_id)
        self._add_motor(new_id, model)
        self._append_log(f"Motor {old_id} is now id {new_id}")

    @Slot(str)
    def _on_error(self, message: str) -> None:
        self._append_log("ERROR: " + message)
        QMessageBox.warning(self, "RobStride Control", message)

    def _append_log(self, message: str) -> None:
        self.log_view.appendPlainText(message)

    # -- E-stop ------------------------------------------------------------------

    def _on_estop(self, checked: bool) -> None:
        self.worker.post(wk.EStop(engage=checked))
        self.estop_btn.setText("E-STOP (engaged)" if checked else "E-STOP")
        if checked:
            for panel in self.panels.values():
                panel.set_enabled_state(False)

    # -- presets -----------------------------------------------------------------

    def _reload_preset_combo(self) -> None:
        self.preset_combo.clear()
        self.preset_combo.addItems(self.presets.names())

    def _current_panel(self) -> MotorPanel | None:
        w = self.tabs.currentWidget()
        return w if isinstance(w, MotorPanel) else None

    def _save_preset(self) -> None:
        panel = self._current_panel()
        if panel is None:
            return
        name, ok = QInputDialog.getText(self, "Save preset", "Preset name:")
        if not ok or not name.strip():
            return
        preset = Preset(
            name=name.strip(),
            device_id=panel.device_id,
            mode=panel.mode_combo.currentData(),
            position=panel.pos_spin.value(),
            velocity=panel.vel_spin.value(),
            current=panel.cur_spin.value(),
            kp=panel.kp_spin.value(),
            kd=panel.kd_spin.value(),
        )
        self.presets.upsert(preset)
        self.presets.save()
        self._reload_preset_combo()
        self._append_log(f"Saved preset '{preset.name}'")

    def _apply_preset(self) -> None:
        panel = self._current_panel()
        name = self.preset_combo.currentText()
        if panel is None or not name:
            return
        preset = self.presets.get(name)
        if preset is None:
            return
        idx = panel.mode_combo.findData(preset.mode)
        if idx >= 0:
            panel.mode_combo.setCurrentIndex(idx)
        panel.pos_spin.setValue(preset.position)
        panel.vel_spin.setValue(preset.velocity)
        panel.cur_spin.setValue(preset.current)
        panel.kp_spin.setValue(preset.kp)
        panel.kd_spin.setValue(preset.kd)
        self._append_log(f"Applied preset '{name}' to motor {panel.device_id}")

    def _delete_preset(self) -> None:
        name = self.preset_combo.currentText()
        if name and self.presets.remove(name):
            self.presets.save()
            self._reload_preset_combo()
            self._append_log(f"Deleted preset '{name}'")

    # -- plot refresh / shutdown -------------------------------------------------

    def _refresh_plots(self) -> None:
        for panel in self.panels.values():
            panel.plot.refresh()

    def closeEvent(self, event) -> None:
        try:
            self.worker.stop()
            self.thread.quit()
            self.thread.wait(2000)
        finally:
            super().closeEvent(event)
