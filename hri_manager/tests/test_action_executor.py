"""ActionExecutor.execute_text_command usability tests.

Runs WITHOUT any external node (no vLLM, no stt/tts servers, no robot):
the executor only parses the typed text against the user's links
(links/test_links.yaml) and publishes valid commands on /hri/skill_command.
The tests subscribe there and check what comes out.
"""
import json

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from hri_manager.action_executor import ActionExecutor
from hri_manager.interaction import SKILL_COMMAND_TOPIC

USER = "test"  # links/test_links.yaml: pick/push (single), pour (double), move (directional)
RECEIVE_TIMEOUT = 3.0  # [s]

# rclpy is initialised once per session in conftest.py (ros_context fixture)


@pytest.fixture(scope="module")
def executor():
    return ActionExecutor(name_user=USER)


@pytest.fixture
def published(executor):
    """Call it with a text command; returns the wire payload dict published on
    SKILL_COMMAND_TOPIC, or None when nothing was published."""
    sub_node = Node("skill_command_listener")
    received = []
    sub_node.create_subscription(
        String, SKILL_COMMAND_TOPIC, lambda msg: received.append(json.loads(msg.data)),
        QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))

    def run(text):
        received.clear()
        executor.execute_text_command(text)
        for _ in range(int(RECEIVE_TIMEOUT / 0.1)):
            rclpy.spin_once(sub_node, timeout_sec=0.1)
            if received:
                return received[0]
        return None

    yield run
    sub_node.destroy_node()


def test_single_object_command(published):
    assert published("pick cup") == {
        "action": "pick", "objects": ["cup"], "parameters": {}, "command": "pick cup"}


def test_double_object_command_with_speed(published):
    assert published("fast pour cup to bowl") == {
        "action": "pour", "objects": ["cup", "bowl"],
        "parameters": {"speed": "fast"}, "command": "fast pour cup to bowl"}


def test_directional_command(published):
    assert published("move left 1cm") == {
        "action": "move", "objects": [],
        "parameters": {"direction": "left", "metric": "1cm"}, "command": "move left 1cm"}


def test_extra_words_are_clipped(published):
    assert published("pick cup bowl")["command"] == "pick cup"


def test_invalid_commands_not_published(published):
    assert published("grab cup") is None        # unknown action
    assert published("pick asdf") is None       # object not in the user's links
    assert published("pour cup") is None        # missing second object
    assert published("move left") is None       # missing metric
    assert published("") is None                # empty input
