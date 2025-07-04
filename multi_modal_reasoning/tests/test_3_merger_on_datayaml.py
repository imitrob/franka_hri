#!/usr/bin/env python
from multi_modal_reasoning.reasoning_merger import ReasoningMerger
import rclpy
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.skill_command import SkillCommand

import pathlib
import yaml
import pytest

# 1. Load test-case data from YAML                                            #
DATA_PATH = pathlib.Path(__file__).with_name("data") / "reasoning_cases.yaml"
with DATA_PATH.open() as f:
    _DATA = yaml.safe_load(f)

SCENE: str = _DATA["scene"]
_CASES: list[dict] = _DATA["cases"]

COMCON = {
    "directional_actions": [],
    "zero_object_actions": [],
    "single_object_actions": ["pick", "push"],
    "double_object_actions": ["pour"],
    "actions": ["pick", "push", "pour"], # all actions
    "adjectives": ["fast","slow","force"],
    #"prepositions": ["to"], #["to", "into", "onto", "from"],
    #"object_types": ["cup", "cube", "plate", "table", "can", "box", "fork", "marker", "note", "storage", "blade", "rack", "ledge", "stand", "platform"],
}

# 2. Common kwargs for every merger.merge() call                              #
_COMMON_KWARGS = dict(
    role_description=get_role_description(
        A=["pick", "push", "pour"],
        O=["cup1", "container1", "bowl1"],
        S=SCENE,
    ),
    command_constraints=COMCON,
)

# 3. Build the parameter list for pytest                                      #
_PARAMS = [
    pytest.param(
        case["voice"],               # voice_stamped
        case["gesture"],             # gesture_stamped
        SkillCommand(case["expected"], COMCON),
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
def test_reasoning_cases(merger: ReasoningMerger, voice, gesture, expected):
    result = merger.merge(
        voice_stamped=voice,
        gesture_stamped=gesture,
        **_COMMON_KWARGS,
    )
    # merger.hri.delete()
    assert result == expected, f"SEE THIS: PREDICTED: {result} != GROUND TRUTH: {expected}\n Raw LM reasoning:{result.reasoning_text}"
