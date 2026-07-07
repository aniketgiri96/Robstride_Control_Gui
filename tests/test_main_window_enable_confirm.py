"""Enabling a motor right after zeroing must ask for confirmation.

A fresh zero moves the reference frame. The worker's safe-enable holds the
current position, but the GUI still guards the *first* enable after a SetZero
with a confirmation dialog so the operator eyeballs the rig first.

Runs the real MainWindow under Qt's offscreen platform, with the worker's
``post`` and ``QMessageBox.question`` stubbed so nothing blocks or talks to
hardware.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui import worker as wk
from robstride_gui.ui import main_window as mw


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


@pytest.fixture
def window(app, monkeypatch):
    w = mw.MainWindow()
    w._connected = True
    w.posted = []
    monkeypatch.setattr(w.worker, "post", w.posted.append)
    w._add_motor(1, "04")
    yield w
    w.close()
    w.deleteLater()


def _stub_question(monkeypatch, answer):
    monkeypatch.setattr(
        mw.QMessageBox, "question",
        staticmethod(lambda *a, **k: answer))


def test_enable_after_zero_prompts_and_enables_when_confirmed(window, monkeypatch):
    window._on_zero_requested(1)          # arms the guard (posts SetZero)
    _stub_question(monkeypatch, mw.QMessageBox.Yes)

    window._on_enable_toggled(1, True)

    assert any(isinstance(c, wk.Enable) for c in window.posted)


def test_enable_after_zero_is_cancelled_when_declined(window, monkeypatch):
    window._on_zero_requested(1)
    _stub_question(monkeypatch, mw.QMessageBox.No)

    window._on_enable_toggled(1, True)

    assert not any(isinstance(c, wk.Enable) for c in window.posted)
    assert window.panels[1]._enabled is False  # button reverted to off


def test_guard_is_one_shot_after_confirmation(window, monkeypatch):
    window._on_zero_requested(1)
    _stub_question(monkeypatch, mw.QMessageBox.Yes)
    window._on_enable_toggled(1, True)

    # A later enable (after a disable) must NOT prompt again: the flag cleared.
    def _boom(*a, **k):  # pragma: no cover - only fires on regression
        raise AssertionError("confirmation asked a second time")
    monkeypatch.setattr(mw.QMessageBox, "question", staticmethod(_boom))

    window._on_enable_toggled(1, False)
    window._on_enable_toggled(1, True)  # would raise if it re-prompted


def test_plain_enable_without_zero_does_not_prompt(window, monkeypatch):
    def _boom(*a, **k):  # pragma: no cover - only fires on regression
        raise AssertionError("confirmation asked without a prior zero")
    monkeypatch.setattr(mw.QMessageBox, "question", staticmethod(_boom))

    window._on_enable_toggled(1, True)

    assert any(isinstance(c, wk.Enable) for c in window.posted)
