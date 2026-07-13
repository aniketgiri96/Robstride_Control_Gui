"""Tests for SequencePlayer stepping and transport.

Driven deterministically via ``player.tick()`` with a capturing post sink, so no
real timer or event loop is needed. Verifies each tick posts the mapped
channels' angles, that the frame advances, that abort rewinds and stop pauses,
and that reaching the end emits ``finished``.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from PySide6.QtWidgets import QApplication

from robstride_gui.sequence import Sequence
from robstride_gui.ui.sequencer import SequencePlayer


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def _seq():
    # 3 frames, 2 channels.
    return Sequence(fps=30.0, channels=("m1", "m2"),
                    frames=((0.0, 10.0), (1.0, 11.0), (2.0, 12.0)))


def _player(app):
    posts: list[tuple[int, float]] = []
    player = SequencePlayer(post=lambda dev, pos: posts.append((dev, pos)))
    return player, posts


def test_tick_posts_mapped_channel_angles(app):
    # Arrange: channel 0 -> motor 5, channel 1 -> motor 6
    player, posts = _player(app)
    player.load(_seq(), {0: 5, 1: 6})

    # Act
    player.tick()

    # Assert: first frame's angles routed to the mapped motors
    assert (5, 0.0) in posts
    assert (6, 10.0) in posts
    assert player.frame == 1


def test_tick_only_drives_mapped_channels(app):
    # Arrange: only channel 1 mapped
    player, posts = _player(app)
    player.load(_seq(), {1: 6})

    # Act
    player.tick()

    # Assert: channel 0 is not posted
    assert posts == [(6, 10.0)]


def test_playing_through_end_emits_finished(app):
    # Arrange
    player, posts = _player(app)
    player.load(_seq(), {0: 1})
    finished: list[bool] = []
    player.finished.connect(lambda: finished.append(True))

    # Act: step past all 3 frames
    for _ in range(3):
        player.tick()

    # Assert: three frames posted, finished fired
    assert [p[1] for p in posts] == [0.0, 1.0, 2.0]
    assert finished == [True]


def test_tick_past_end_is_safe_and_finishes(app):
    player, _ = _player(app)
    player.load(_seq(), {0: 1})
    for _ in range(3):
        player.tick()
    # Extra tick beyond the end must not raise or post further.
    player.tick()
    assert player.frame == 3


def test_abort_rewinds_to_start(app):
    # Arrange
    player, _ = _player(app)
    player.load(_seq(), {0: 1})
    player.tick()
    player.tick()
    assert player.frame == 2

    # Act
    player.abort()

    # Assert
    assert player.frame == 0
    assert not player.is_playing


def test_load_resets_previous_frame(app):
    player, _ = _player(app)
    player.load(_seq(), {0: 1})
    player.tick()
    # Re-loading rewinds.
    player.load(_seq(), {0: 1})
    assert player.frame == 0


def test_play_without_sequence_does_not_crash(app):
    player, posts = _player(app)
    player.play()
    assert not player.is_playing
    assert posts == []
