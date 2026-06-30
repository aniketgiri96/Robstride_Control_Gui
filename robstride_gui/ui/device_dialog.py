"""Device inventory dialog: list motors on the bus and assign CAN ids.

Read-only discovery plus a single assign action. The dialog owns no IO: it
emits :attr:`detectRequested` / :attr:`assignRequested` and is fed results via
:meth:`populate`, so all bus access stays on the worker thread.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QDialog, QHBoxLayout, QHeaderView, QLabel, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)


class DeviceDialog(QDialog):
    """Lists responding CAN ids with each motor's unique MCU id; assigns a new id."""

    detectRequested = Signal()
    assignRequested = Signal(int, int)   # current_id, new_id

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setWindowTitle("Devices")
        self.resize(560, 360)
        # can_id of the currently selected row, or None.
        self._selected_id: int | None = None
        self._selected_collision = False
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        top = QHBoxLayout()
        self.detect_btn = QPushButton("Detect")
        self.detect_btn.setToolTip("Scan the bus and list every responding motor")
        self.detect_btn.clicked.connect(self.detectRequested)
        top.addWidget(self.detect_btn)
        top.addStretch(1)
        root.addLayout(top)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["CAN id", "Motors", "Unique MCU id(s)"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self._on_selection_changed)
        root.addWidget(self.table, 1)

        assign = QHBoxLayout()
        assign.addWidget(QLabel("Assign selected →  new CAN id"))
        self.new_id_spin = QSpinBox()
        self.new_id_spin.setRange(1, 127)
        assign.addWidget(self.new_id_spin)
        self.assign_btn = QPushButton("Assign")
        self.assign_btn.setEnabled(False)
        self.assign_btn.clicked.connect(self._on_assign_clicked)
        assign.addWidget(self.assign_btn)
        assign.addStretch(1)
        root.addLayout(assign)

        self.note = QLabel("")
        self.note.setWordWrap(True)
        root.addWidget(self.note)

    # -- population --------------------------------------------------------------

    def populate(self, items: list[dict]) -> None:
        """Fill the table from worker inventory: ``[{"can_id", "uids":[hex,...]}]``."""
        self.table.setRowCount(0)
        for entry in items:
            can_id = int(entry["can_id"])
            uids = list(entry.get("uids") or [])
            row = self.table.rowCount()
            self.table.insertRow(row)

            id_item = QTableWidgetItem(str(can_id))
            id_item.setData(Qt.UserRole, can_id)
            id_item.setData(Qt.UserRole + 1, len(uids) > 1)
            self.table.setItem(row, 0, id_item)

            count_item = QTableWidgetItem(str(len(uids)) if uids else "?")
            self.table.setItem(row, 1, count_item)

            uid_text = "\n".join(uids) if uids else "(no id reported)"
            if len(uids) > 1:
                uid_text = uid_text + "   ⚠ collision: motors share this id"
            uid_item = QTableWidgetItem(uid_text)
            self.table.setItem(row, 2, uid_item)

        self.table.resizeRowsToContents()
        if not items:
            self.note.setText("No motors responded. Check power and wiring, then Detect again.")
        else:
            self.note.setText("Select a row, set a new CAN id, then Assign. "
                              "Power-cycle the motor afterwards to confirm it sticks.")
        self._on_selection_changed()

    # -- selection / assign ------------------------------------------------------

    def _on_selection_changed(self) -> None:
        model = self.table.selectionModel()
        rows = model.selectedRows() if model else []
        if not rows:
            self._selected_id = None
            self.assign_btn.setEnabled(False)
            return
        item = self.table.item(rows[0].row(), 0)
        self._selected_id = item.data(Qt.UserRole)
        self._selected_collision = bool(item.data(Qt.UserRole + 1))
        self.new_id_spin.setValue(self._selected_id)
        self.assign_btn.setEnabled(True)

    def _on_assign_clicked(self) -> None:
        if self._selected_id is None:
            return
        new_id = self.new_id_spin.value()
        if new_id == self._selected_id:
            self.note.setText("New id is the same as the current id — nothing to do.")
            return
        if self._selected_collision:
            self.note.setText(
                f"⚠ id {self._selected_id} has multiple motors. Assigning will change "
                "ALL of them. To split them, leave only one powered on the bus first.")
        self.assignRequested.emit(int(self._selected_id), int(new_id))
