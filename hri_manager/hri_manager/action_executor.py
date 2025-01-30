
#!/usr/bin/env python
from hri_manager.HriCommand import HriCommand

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG

from hri_manager.hri import HRI

import argparse

class ActionExecutor():
    def __init__(self, name_user: str, listener_topics=["gestures", "nl"], dry_run:bool = False):
        self.hri = HRI(name_user=name_user,tts_enabled=True, dry_run=dry_run)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for mod in listener_topics:
            self.hri.create_subscription(
                HRICommandMSG,
                f'/modality/{mod}',
                self.play_skill_callback,
                qos_profile=qos)
        
    def play_skill_callback(self, msg):
        hricommand = HriCommand.from_ros(msg)
        self.hri.play_skill(hricommand.target_action, name_template=hricommand.target_action)

def main(dry_run: bool):
    parser = argparse.ArgumentParser(description="My ROS 2 Node")
    parser.add_argument('--name_user', type=str, help='The user name')
    args = parser.parse_args()

    rclpy.init()
    node = ActionExecutor(name_user=args.name_user, dry_run=dry_run)
    node.hri.speak(f"Hi, {node.hri.user}! You can press 'R' to start recording")
    try:
        while True:
            input("")
    except KeyboardInterrupt:
        pass    

def action_executor():
    main(dry_run=False)

def action_executor_dry_run():
    print("DRY RUN !", flush=True)
    main(dry_run=True)

if __name__ == "__main__":
    action_executor()
