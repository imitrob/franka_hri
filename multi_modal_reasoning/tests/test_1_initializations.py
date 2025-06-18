import pytest
import rclpy
from multi_modal_reasoning.skill_command import SkillCommand

@pytest.fixture(scope="session", autouse=True)
def ros_init_shutdown():
    """
    Initialize rclpy once per pytest session, and shutdown at the end.
    """
    # Initialize ROS
    rclpy.init()
    yield
    # Shutdown ROS after all tests
    try:
        rclpy.shutdown()
    except Exception:
        pass

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
    comcon = {
    "zero_object_actions": ["stop", "release", "home"],
    "single_object_actions": ["pick", "push", "pass", "point", "open", "close"],
    "double_object_actions": ["place", "transfer", "move", "put", "pour"],
    "actions": ["pick", "push", "pass", "place", "point", "open", "close", "put", "stop", "release", "pour"], 
    }

    # these should be valid
    assert SkillCommand("stop", command_constraints=comcon).is_valid()
    assert SkillCommand("pick cup1", command_constraints=comcon).is_valid()
    assert SkillCommand("pour cup1 to cup2", command_constraints=comcon).is_valid()

    # these should not be valid
    assert not SkillCommand("stop cup1", command_constraints=comcon).is_valid()
    assert not SkillCommand("stop cup1 to cup2", command_constraints=comcon).is_valid()
    assert not SkillCommand("pick cup1 cup2", command_constraints=comcon).is_valid()
    assert not SkillCommand("pick cup1 to cup2", command_constraints=comcon).is_valid()
    assert not SkillCommand("pour cup1", command_constraints=comcon).is_valid()
    assert not SkillCommand("pour", command_constraints=comcon).is_valid()

    assert SkillCommand("pour", command_constraints=comcon).target_action == "pour"
    assert SkillCommand("pour cup1", command_constraints=comcon).target_action == "pour"
    assert SkillCommand("pour cup1", command_constraints=comcon).target_object == "cup1"
    assert SkillCommand("quickly pour cup1", command_constraints=comcon).target_action == "pour"
    assert SkillCommand("quickly pour cup1", command_constraints=comcon).target_object == "cup1"
    assert SkillCommand("quickly pour cup1", command_constraints=comcon).action_parameter == "quickly"
    assert SkillCommand("quickly pour cup1 to cup2", command_constraints=comcon).target_object2 == "cup2"
    assert SkillCommand("quickly pour cup1 to cup2", command_constraints=comcon).object_preposition == "to"