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


# rclpy is initialised once per session in conftest.py (ros_context fixture)


@pytest.fixture(scope="module")
def hri():
    """An HRI holding only clients (skills published to the robot node, LLM in
    the vLLM server, STT/TTS in their server nodes). Init needs the vLLM
    server; speech tests need stt_node/tts_node running to do real work."""
    if not Path(f"{hri_manager.package_path}/links/{USER}_links.yaml").is_file():
        pytest.skip(f"missing links/{USER}_links.yaml")
    from hri_manager.hri import HRI
    try:
        h = HRI(name_user=USER)
    except Exception as exc:  # audio hardware/vLLM server not available
        pytest.skip(f"cannot initialise HRI: {exc}")

    yield h


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
