#!/usr/bin/env python
"""Human-robot interaction node.

Every model runs in its own server; HRI only holds clients (see
hri_manager/interaction.py for the shared interaction functions):
- LLM: vLLM server, SentenceProcessor client (multi_modal_reasoning/models/llm.py)
- Speech-to-text: stt_node, SpeechToTextClient (__call__(file) -> text)
- Text-to-speech: tts_node, TextToSpeechClient (speak(text))
- Robot: the LfD node subscribes to SKILL_COMMAND_TOPIC; skills are published
  there, never executed in-process.
"""
import json

from gesture_sentence_maker.gesture_sentence_getter import GestureSentenceGetter

from std_msgs.msg import String

from skills_manager.ros_utils import SpinningRosNode
from hri_manager.interaction import InteractionNode
from hri_manager.user_links import load_user_links
from scene_getter.scene_getting import SceneGetter


class HRI(SceneGetter, InteractionNode, SpinningRosNode):
    def __init__(self, name_user: str):
        self.user = name_user
        super(HRI, self).__init__()

        self.user_profile_links_dict = load_user_links(self.user)
        self.A = self.user_profile_links_dict["actions"]
        self.O = self.user_profile_links_dict["objects"]

        self.init_interaction()  # model clients + modality/robot publishers
        self.gestures = GestureSentenceGetter(self)

    def listen_user(self):
        self.rec.start_recording()
        input("Press enter to finish")
        file, _ = self.rec.stop_recording()
        return self.stt(file)

    def play_skill(self,
            name_skill: str,
            name_template: str = "",
            skill_parameter: None | float = None,
        ):
        if name_skill == "":
            self.speak(f"No action found, try again")
            return

        self.speak(f"Executing {name_skill} with object {name_template}!")

        self.skill_command_pub.publish(String(data=json.dumps({
            "action": name_skill,
            "objects": [name_template] if name_template else [],
            "parameters": {} if skill_parameter is None else {"skill_parameter": skill_parameter},
            "command": f"{name_skill} {name_template}".strip(),
        })))


def main():
    import argparse
    import rclpy
    parser = argparse.ArgumentParser(description="HRI node")
    parser.add_argument('--name_user', type=str, help='The user name')
    args = parser.parse_args()

    rclpy.init()
    hri = HRI(name_user=args.name_user)
    hri.speak("HRI is ready!")

if __name__ == "__main__":
    main()
