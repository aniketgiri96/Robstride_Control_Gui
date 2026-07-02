"""Error popups must be rate-limited so a failing bus cannot freeze the UI.

The worker emits one error per failure; before rate-limiting, each one opened a
*modal* QMessageBox, so a dead adapter at the 100 Hz loop rate stacked hundreds
of dialogs. Every error must still reach the log - only the popup is throttled.

Runs the real :class:`MainWindow` under Qt's offscreen platform, with
``QMessageBox.warning`` stubbed out so nothing blocks.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui.ui import main_window as mw


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


@pytest.fixture
def window(app, monkeypatch):
    shown: list[str] = []
    monkeypatch.setattr(
        mw.QMessageBox, "warning",
        staticmethod(lambda parent, title, text, *a, **k: shown.append(text)))
    w = mw.MainWindow()
    w.shown_dialogs = shown
    yield w
    w.close()
    w.deleteLater()


def test_error_burst_shows_one_dialog_but_logs_everything(window):
    # Act: an error burst like the one a dead adapter produces
    for i in range(20):
        window._on_error(f"serial write failed ({i})")

    # Assert: one popup, twenty log lines
    assert len(window.shown_dialogs) == 1
    log_text = window.log_view.toPlainText()
    assert log_text.count("serial write failed") == 20


def test_error_dialog_allowed_again_after_interval(window):
    window._on_error("first failure")
    assert len(window.shown_dialogs) == 1

    # Simulate the throttle window elapsing, then a new error arrives.
    window._last_error_dialog_time = float("-inf")
    window._on_error("second failure")

    assert len(window.shown_dialogs) == 2
