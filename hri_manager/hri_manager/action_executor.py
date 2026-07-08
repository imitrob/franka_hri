#!/usr/bin/env python
"""Plain action executor: single-modality bypass with NO merging and NO
reasoning (LLM). Use it to rule out merger problems and just execute.

Inputs, each handled independently:
- typed terminal command ("pick cube") -> SkillCommand parser -> publish
- /modality/gestures -> execute the gesture's action directly
"""
from hri_manager.HriCommand import HriCommand
from hri_manager.hri import HRI

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG
import argparse

from multi_modal_reasoning.skill_command import SkillCommand

class ActionExecutor():
    def __init__(self, name_user: str):
        self.hri = HRI(name_user=name_user)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.hri.create_subscription(HRICommandMSG, '/modality/gestures', self.play_skill_callback, qos_profile=qos)

    def execute_text_command(self, text: str):
        """Deterministic path: exact-vocabulary parse against the user's
        links, validity-gated publish. Wrong words -> command rejected."""
        skillcommand = SkillCommand(text, self.hri.user_profile_links_dict)
        self.hri.play_skillcommand(skillcommand)

    def play_skill_callback(self, msg):
        hricommand = HriCommand.from_ros(msg)
        target_object = hricommand.target_object if "object" in hricommand.pv_dict else ""
        self.hri.play_skill(hricommand.target_action, name_template=target_object)

def main():
    parser = argparse.ArgumentParser(description="Plain (no merge, no reasoning) action executor")
    parser.add_argument('--name_user', type=str, help='The user name')
    args = parser.parse_args()

    rclpy.init()
    node = ActionExecutor(name_user=args.name_user)
    node.hri.speak(f"Hi, {node.hri.user}! Type a command, e.g.: {node.hri.A[0] if node.hri.A else 'pick'} {node.hri.O[0] if node.hri.O else 'cube'}")
    try:
        while rclpy.ok():
            text = input("Command: ").strip()
            if text == "":
                continue
            node.execute_text_command(text)
    except (KeyboardInterrupt, EOFError):
        pass

def action_executor():
    main()

if __name__ == "__main__":
    action_executor()
