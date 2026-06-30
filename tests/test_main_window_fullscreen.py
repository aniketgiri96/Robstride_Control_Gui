"""Tests for the main window's fullscreen toggle and state restore.

These run the real :class:`MainWindow` under Qt's offscreen platform so no
display is needed. They verify that entering and leaving fullscreen restores
the window to its previous state (normal vs maximized) rather than collapsing a
maximized window down to a small normal one.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui.ui.main_window import MainWindow


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


@pytest.fixture
def window(app):
    w = MainWindow()
    yield w
    w.close()
    w.deleteLater()


def test_f11_enters_and_exits_fullscreen_from_normal(window):
    # Arrange
    window.showNormal()

    # Act: F11 in, F11 out
    window._toggle_fullscreen()
    assert window.isFullScreen()
    window._toggle_fullscreen()

    # Assert: back to a normal (non-fullscreen, non-maximized) window
    assert not window.isFullScreen()
    assert not window.isMaximized()


def test_exiting_fullscreen_restores_maximized_state(window):
    # Arrange: window is maximized before going fullscreen
    window.showMaximized()
    assert window.isMaximized()

    # Act: enter fullscreen, then leave it
    window._toggle_fullscreen()
    assert window.isFullScreen()
    window._exit_fullscreen()

    # Assert: maximized state is restored, not dropped to a small window
    assert not window.isFullScreen()
    assert window.isMaximized()


def test_escape_only_acts_in_fullscreen(window):
    # Arrange: a maximized window that is NOT fullscreen
    window.showMaximized()

    # Act: Esc should be a no-op when not fullscreen
    window._exit_fullscreen()

    # Assert: still maximized, untouched
    assert window.isMaximized()
    assert not window.isFullScreen()
