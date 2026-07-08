#!/usr/bin/env python
"""Uncertain commands (e.g. "pick up this" with no gesture): the merger must
fall back with no action to execute -- the merged SkillCommand has to be
invalid (is_valid() == False), which is what play_skillcommand() checks
before handing the command to the robot."""
from multi_modal_reasoning.reasoning_merger import ReasoningMerger
from multi_modal_reasoning.role_setup import get_role_description
from hri_manager.user_links import load_user_links

import pathlib
import yaml
import pytest

# 1. Load test-case data from YAML                                            #
DATA_PATH = pathlib.Path(__file__).with_name("data") / "uncertain_cases.yaml"
with DATA_PATH.open() as f:
    _DATA = yaml.safe_load(f)

SCENE: str = _DATA["scene"]
_CASES: list[dict] = _DATA["cases"]

# Command constraints shared by the whole test suite (links/test_links.yaml).
# The links yaml holds object CLASSES; ground to this scene's instances.
COMCON = load_user_links("test")
COMCON["objects"] = _DATA["objects"]

# 2. Common kwargs for every merger.merge() call                              #
_COMMON_KWARGS = dict(
    role_description=get_role_description(
        A=COMCON["actions"],
        O=COMCON["objects"],
        S=SCENE,
    ),
    command_constraints=COMCON,
)

# 3. Build the parameter list for pytest                                      #
_PARAMS = [
    pytest.param(
        case["voice"],               # voice_stamped
        case["gesture"],             # gesture_stamped
        id=case.get("id", f"case-{idx}"),
    )
    for idx, case in enumerate(_CASES, start=1)
]

# 4. Fixtures (rclpy is initialised once per session in conftest.py)          #
@pytest.fixture(scope="module")
def merger():
    # The reasoning model is whatever the vLLM server is serving.
    m = ReasoningMerger()
    yield m                           # ---- tests run here ----
    m.delete()                        # tear-down after last test in module

# 5. The single parametrised test                                             #
@pytest.mark.parametrize("voice, gesture", _PARAMS)
def test_uncertain_cases(merger: ReasoningMerger, voice, gesture):
    result = merger.merge(
        voice_stamped=voice,
        gesture_stamped=gesture,
        **_COMMON_KWARGS,
    )
    assert not result.is_valid(), (
        f"SEE THIS: PREDICTED: {result} is executable, but the command is uncertain "
        f"and must NOT be executed\n Raw LM reasoning:{result.reasoning_text}"
    )
