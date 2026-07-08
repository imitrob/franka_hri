#!/usr/bin/env python
from multi_modal_reasoning.reasoning_merger import ReasoningMerger
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.skill_command import SkillCommand
from hri_manager.user_links import load_user_links

import pathlib
import yaml
import pytest

# 1. Load test-case data from YAML                                            #
DATA_PATH = pathlib.Path(__file__).with_name("data") / "directional_cases.yaml"
with DATA_PATH.open() as f:
    _DATA = yaml.safe_load(f)

_CASES: list[dict] = _DATA["cases"]

# Command constraints shared by the whole test suite (links/test_links.yaml)
COM_CON = load_user_links("test")

# 2. Common kwargs for every merger.merge() call                              #
_COMMON_KWARGS = dict(
    role_description=get_role_description(
        A=COM_CON["actions"],
        O=COM_CON["objects"],
        S="",
    ),
    command_constraints=COM_CON,
)

# 3. Build the parameter list for pytest                                      #
_PARAMS = [
    pytest.param(
        case["voice"],               # voice_stamped
        case["gesture"],             # gesture_stamped
        SkillCommand(case["expected"], COM_CON),
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
@pytest.mark.parametrize("voice, gesture, expected", _PARAMS)
def test_directional_cases(merger: ReasoningMerger, voice, gesture, expected):
    result = merger.merge(
        voice_stamped=voice,
        gesture_stamped=gesture,
        **_COMMON_KWARGS,
    )
    # merger.hri.delete()
    assert result == expected, f"SEE THIS---PREDICTED: {result} != GROUND TRUTH: {expected}\n Raw LM reasoning:{result.reasoning_text}"

