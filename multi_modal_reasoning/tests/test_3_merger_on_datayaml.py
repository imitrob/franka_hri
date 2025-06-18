#!/usr/bin/env python
from multi_modal_reasoning.reasoning_merger import ReasoningMerger
import rclpy
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.generate_dataset import CONFIG3
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

# 2. Common kwargs for every merger.merge() call                              #
_COMMON_KWARGS = dict(
    role_description=get_role_description(
        A=["pick", "push", "pour"],
        O=["cup1", "container1", "bowl1"],
        S=SCENE,
    ),
    command_constraints=CONFIG3,
)

# 3. Build the parameter list for pytest                                      #
_PARAMS = [
    pytest.param(
        case["voice"],               # voice_stamped
        case["gesture"],             # gesture_stamped
        SkillCommand(case["expected"]),
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
        pytest.param("SultanR/SmolTulu-1.7b-Instruct",  id="smoltulu-1.7b"),
        pytest.param("Qwen/Qwen3-0.6B",                 id="qwen3-0.6b"),
    ],
)
def merger(request) -> ReasoningMerger:
    """Create a ReasoningMerger for each model under test."""
    return ReasoningMerger(
        name_user="casper",
        model_name=request.param,
        tts_enabled=False,
    )


# 5. The single parametrised test                                             #
@pytest.mark.parametrize("voice, gesture, expected", _PARAMS)
def test_reasoning_cases(merger: ReasoningMerger, voice, gesture, expected):
    result = merger.merge(
        voice_stamped=voice,
        gesture_stamped=gesture,
        **_COMMON_KWARGS,
    )
    # merger.hri.delete()
    assert result == expected



 


if __name__ == "__main__":
    # test_skill_commands()
    # test_just_to_see_if_works()
    test_reasoning_cases()
    # test_just_probabilistic()
    # test_just_alternatives()
    # test_just_alternatives2()
    # test_just_alternatives3()

    # test_lm()
    # test_unsuccessful()

    # test_alignment_noise()
    # test_on_saved_data()