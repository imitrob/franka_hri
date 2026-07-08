#!/usr/bin/env python
from multi_modal_reasoning.reasoning_merger import ReasoningMerger
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.skill_command import SkillCommand
from hri_manager.user_links import load_user_links
import pathlib, yaml, time, pytest

# rclpy is initialised once per session in conftest.py (ros_context fixture)

@pytest.fixture(scope="module")
def dummy_merger():
    """Merger instance using whatever model the vLLM server serves."""
    return ReasoningMerger()

# Command constraints shared by the whole test suite (links/test_links.yaml)
COMCON = load_user_links("test")

DATA_PATH = pathlib.Path(__file__).with_name("data") / "reasoning_cases.yaml"
with DATA_PATH.open() as f:
    _DATA = yaml.safe_load(f)
SCENE: str = _DATA["scene"]
COMCON["objects"] = _DATA["objects"]  # ground to the scene's instances

_COMMON_KWARGS = dict(
    role_description=get_role_description(
        A=COMCON["actions"],
        O=COMCON["objects"],
        S=SCENE,
    ),
    command_constraints=COMCON,
)

@pytest.fixture(scope="module")
def merger():
    # The reasoning model is whatever the vLLM server is serving.
    m = ReasoningMerger()
    yield m                           # ---- tests run here ----
    m.delete()                        # tear-down after last test in module
    
""" When I install this package, I always try to run this function """
def test_just_to_see_if_works(merger):
    
    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge(
        voice_stamped=[
            [0.0, "pick"],
            [0.1, "green"],
            [0.4, "cup"],
        ],
        gesture_stamped = [
            [0.4, "cup1"],
        ],
        role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "drawer", "bowl"], S=S),
        command_constraints=COMCON,
    )
    merger.save_log("pick cup1", skill_command, [[0.0, "pick"],[0.1, "green"],[0.4, "cup"],], [ [0.4, "cup1"],], S, ["cup1", "drawer", "bowl"], COMCON, get_role_description(A=["pick", "push", "pour"], O=["cup1", "drawer", "bowl"], S=S))

    # merger.hri.delete()
    

@pytest.mark.parametrize(
    "time_limit",
    [
        pytest.param(0.5, id="under_500ms"),
        pytest.param(1, id="under_1s"),
    ],
)
def test_merge_completes_within_limit(merger, time_limit):
    """merge() must finish before *time_limit* seconds, result itself irrelevant."""
    voice = [[0.0, "Pick"], [0.2, "cup1"]]
    gesture = []

    start = time.perf_counter()
    result = merger.merge(
        voice_stamped=voice,
        gesture_stamped=gesture,
        **_COMMON_KWARGS,
    )
    elapsed = time.perf_counter() - start

    assert elapsed < time_limit, (
        f"merge() took {elapsed:.2f}s which exceeds the {time_limit}s limit"
    )
    # Optional sanity check: we only care it returns *something*.
    assert isinstance(result, SkillCommand)