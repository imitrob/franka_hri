#!/usr/bin/env python
from multi_modal_reasoning.reasoning_merger import ReasoningMerger
import rclpy
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.skill_command import SkillCommand

import pathlib
import yaml
import pytest

# 1. Load test-case data from YAML                                            #
DATA_PATH = pathlib.Path(__file__).with_name("data") / "directional_cases.yaml"
with DATA_PATH.open() as f:
    _DATA = yaml.safe_load(f)

_CASES: list[dict] = _DATA["cases"]


COM_CON = {"directional_actions": ["move"], "actions": ["move"], "adjectives": []}
# 2. Common kwargs for every merger.merge() call                              #
_COMMON_KWARGS = dict(
    role_description=get_role_description(
        A=["move"],
        O=[],
        S="",
        version="DIRECTIONS",
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

# 4. Fixtures                                                                 #
@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Initialise rclpy exactly once for the whole test session."""
    rclpy.init()
    yield
    rclpy.shutdown()    

@pytest.fixture(
    scope="module",
    params=[
        pytest.param("Qwen/Qwen3-1.7B", id="qwen3"),
        pytest.param("LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct", id="exaone"),
        pytest.param("ibm-granite/granite-3.1-2b-instruct", id="granite"),
        pytest.param("deepseek-ai/DeepSeek-R1-Distill-Qwen-1.5B", id="deepseek"),
    ],
)
def merger(request):
    m = ReasoningMerger(
        name_user="casper",
        model_name=request.param,
        tts_enabled=False,
    )
    yield m                           # ---- tests run here ----
    m.hri.delete()                    # tear-down after last test in module
    
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

