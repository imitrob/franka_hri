#!/usr/bin/env python
"""Feature smoke tests for HRI: text-to-speech and speech-to-text.

These exercise the real GPU TTS/STT models and audio hardware, so the whole
module is skipped when the user's links config or the models are unavailable
(e.g. on CI). `test_listen_user` additionally needs a person to speak, so it
only runs in an interactive terminal.

Run:  pytest hri_manager/test_features.py
"""
from pathlib import Path
import sys

import pytest
import rclpy
import torch

import hri_manager

USER = "casper"
OOM_HINT = "GPU out of memory — free VRAM (e.g. stop the vLLM server sharing this GPU)"


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def hri():
    """A dry-run HRI (no robot) with TTS + STT models loaded."""
    if not Path(f"{hri_manager.package_path}/links/{USER}_links.yaml").is_file():
        pytest.skip(f"missing links/{USER}_links.yaml")
    from hri_manager.hri import HRI
    try:
        h = HRI(name_user=USER, dry_run=True)
    except Exception as exc:  # models/audio hardware not available
        pytest.skip(f"cannot initialise HRI: {exc}")

    yield h

    # HCI.delete() assumes a sentence_processor (set only by ReasoningMerger),
    # so free the models this HRI actually owns directly.
    if h.tts_enabled:
        h.tts.delete()
    if h.stt_enabled:
        h.stt.delete()


@pytest.mark.timeout(120)
def test_text_to_speech(hri):
    """speak() renders text through TTS without raising."""
    try:
        hri.speak("Testing the text to speech feature!")
    except torch.OutOfMemoryError:
        pytest.skip(OOM_HINT)


@pytest.mark.skipif(not sys.stdin.isatty(),
                    reason="interactive: needs a person to speak + press enter")
@pytest.mark.timeout(180)
def test_listen_user(hri):
    """Record the user and transcribe: STT must return a non-empty word."""
    try:
        action_word = hri.listen_user()
    except torch.OutOfMemoryError:
        pytest.skip(OOM_HINT)
    assert isinstance(action_word, str) and action_word.strip(), \
        f"STT returned nothing useful: {action_word!r}"
    hri.speak(f"Did you say the word: {action_word}")
