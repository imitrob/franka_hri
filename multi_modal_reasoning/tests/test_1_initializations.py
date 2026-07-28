import pytest
from multi_modal_reasoning.skill_command import SkillCommand

# rclpy is initialised once per session in conftest.py (ros_context fixture)

import pytest
import rclpy
from rclpy.node import Node

@pytest.fixture
def ros_node():
    """
    Create and destroy a Node for each test.
    """
    node = Node("test_node")
    yield node
    # Properly destroy node
    try:
        node.destroy_node()
    except Exception:
        pass

def test_just_to_see_if_works(ros_node):
    # Use ros_node for publishing/subscribing, or simply check it exists
    assert ros_node.get_name() == "test_node"

def test_skill_commands():
    context = {
    "directional_actions": ["move"],
    "zero_object_actions": ["stop", "release", "home"],
    "single_object_actions": ["pick", "push", "pass", "point", "open", "close"],
    "double_object_actions": ["place", "transfer", "put", "pour"],
    "actions": ["pick", "push", "pass", "place", "move", "point", "open", "close", "put", "stop", "release", "pour"],
    "adjectives": ["quickly", "slowly", "carefully", "lightly", "force"],
    "objects": ["cup1", "cup2"],
    }

    # these should be valid
    assert SkillCommand("stop", context).is_valid()
    assert SkillCommand("pick cup1", context).is_valid()
    assert SkillCommand("pour cup1 to cup2", context).is_valid()
    assert SkillCommand("move left 1cm", context).is_valid()

    # these are clipped
    assert SkillCommand("stop cup1", context) == SkillCommand("stop", context)
    assert SkillCommand("stop cup1 to cup2", context) == SkillCommand("stop", context)
    assert SkillCommand("pick cup1 cup2", context) == SkillCommand("pick cup1", context)
    assert SkillCommand("pick cup1 to cup2", context) == SkillCommand("pick cup1", context)
    # these should not be valid
    assert not SkillCommand("pour cup1", context).is_valid()
    assert not SkillCommand("pour", context).is_valid()
    assert not SkillCommand("grab cup1", context).is_valid()  # unknown action
    assert not SkillCommand("move left", context).is_valid()  # missing metric
    # strict object membership, with the reason on the command
    cmd = SkillCommand("pick asdf", context)
    assert not cmd.is_valid()
    assert "asdf" in cmd.invalid_reason

    # canonical fields
    assert SkillCommand("pour", context).action == "pour"
    assert SkillCommand("pour cup1", context).action == "pour"
    assert SkillCommand("pour cup1", context).objects == ["cup1"]
    assert SkillCommand("quickly pour cup1", context).action == "pour"
    assert SkillCommand("quickly pour cup1", context).objects == ["cup1"]
    assert SkillCommand("quickly pour cup1", context).parameters == {"speed": "quickly"}
    assert SkillCommand("quickly pour cup1 to cup2", context).objects == ["cup1", "cup2"]
    assert SkillCommand("move left 1cm", context).parameters == {"direction": "left", "metric": "1cm"}

    # the command string round-trips
    for sentence in ["stop", "pick cup1", "quickly pour cup1 to cup2", "move left 1cm"]:
        assert str(SkillCommand(sentence, context)) == sentence

    # structured (LLM schema) path ends in the same command
    structured = {"action": "pour", "speed": "quickly", "object1": "cup1", "relation": "to",
                  "object2": "cup2", "direction": "none", "distance": "none", "unit": "none"}
    assert SkillCommand.from_structured(structured, context) == SkillCommand("quickly pour cup1 to cup2", context)
    assert SkillCommand.from_structured(structured, context).is_valid()

