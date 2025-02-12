
#!/usr/bin/env python
from hri_manager.HriCommand import HriCommand
from hri_manager.hri import HRI
from hri_manager.hci import map_instruction_words

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG
from std_msgs.msg import String
import argparse
import numpy as np

class ActionExecutor():
    def __init__(self, name_user: str, dry_run:bool = False):
        self.hri = HRI(name_user=name_user,tts_enabled=True, dry_run=dry_run)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.hri.create_subscription(HRICommandMSG, '/modality/gestures', self.play_skill_callback, qos_profile=qos)
        self.hri.create_subscription(String, '/recorded_file', self.process_and_play_skill_callback, qos_profile=qos)

    def process_and_play_skill_callback(self, msg):
        recording_name = msg.data
        print(f"1. Speech to text; file: {recording_name}", flush=True)
        sentence_text = self.hri.stt.forward(recording_name)
        print("Sentence text: ", sentence_text, flush=True)
        self.hri.speak(f"You said: {sentence_text}")
        print("2. Sentence processing", flush=True)
        output = self.hri.sentence_processor.predict(sentence_text)

        for k in output.keys():
            if isinstance(output[k], np.ndarray):
                output[k] = list(output[k])

        print("Sentence processing output: ", output, flush=True)

        target_action, target_object = map_instruction_words(output, self.hri.user)
        self.hri.play_skill(target_action, target_object)

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
