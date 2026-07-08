#!/usr/bin/env python
"""Multi-modal reasoning merger (ROS2 node).

Pipeline: voice (STT) and gestures arrive as timestamped word streams, get
interleaved by timestamp into one sentence, sent to the LLM (vLLM server,
see models/llm.py) and the reply is parsed into a SkillCommand, which is
published on SKILL_COMMAND_TOPIC for the robot (LfD) node to execute.

The shared interaction functions (model clients, speak, record_voice,
play_skillcommand) come from hri_manager/interaction.py -- every model runs
in its own server, this node only holds clients. Only "deterministic"
interpretation.
"""
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from hri_msgs.msg import HRICommand as HRICommandMSG
from hri_manager.HriCommand import HriCommand
from hri_manager.interaction import InteractionNode, SKILL_COMMAND_TOPIC
from hri_manager.user_links import load_user_links

from gesture_meaning.one_to_one_mapping import OneToOneMapping
import multi_modal_reasoning
from multi_modal_reasoning.models import llm
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.skill_command import SkillCommand, build_command_schema
from multi_modal_reasoning.utils import print_modalities
from naive_merger.utils import cc

RECEIVE_CHECK_INTERVAL = 1.0  # [s]
EXIT_AFTER_EXECUTION = True

class ReasoningMerger(InteractionNode, Node):
    def __init__(self):
        """
        The reasoning model is whatever the vLLM server is serving; it is
        discovered from the server (see models/llm.py) and available as
        `self.model_name` for logging.
        """
        super().__init__(f"reasoning_merger_{np.random.randint(100000)}")

        # Model clients + /recorded_file and skill-command publishers
        self.init_interaction()
        self.model_name = self.sentence_processor.model_name

        # Gesture names -> action words (grab -> pick), as a direct function
        # call: the /teleop_gesture_toolbox/get_meaning service is too slow.
        self.gesture_mapping = OneToOneMapping()

        # Modality inputs
        self.record_queue = []
        self.gestures_queue = []
        self.create_subscription(HRICommandMSG, "/modality/gestures", self.gesture_callback,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
        self.create_subscription(String, "/recorded_file", self.voice_record_callback,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        # Spin this node in a background thread (replaces SpinningRosNode)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
        from rclpy.executors import ExternalShutdownException
        try:
            while rclpy.ok():
                self._executor.spin_once(timeout_sec=0.01)
        except ExternalShutdownException:
            pass  # rclpy.shutdown() called elsewhere; exit the spin thread quietly

    def name(self):
        return self.model_name.split("/")[-1]

    def voice_record_callback(self, msg):
        print(f"Received: {cc.W}Recording{cc.E}", flush=True)
        self.record_queue.append(json.loads(msg.data))

    def gesture_callback(self, msg):
        hricommand = HriCommand.from_ros(msg)
        print(f"Received: {cc.W}Gestures{cc.E}", flush=True)
        self.gestures_queue.append(hricommand)

    def delete(self):
        self.delete_interaction()
        self.destroy_node()

    def spin(self, command_constraints):
        """Interactive loop: record voice (enter starts/stops), confirm
        (enter), merge with the last gesturing, publish the resulting skill
        command."""
        while rclpy.ok():
            if len(self.record_queue) == 0:
                self.record_voice()  # enter starts/stops (terminal input)
                # the recording arrives back via the /recorded_file topic
                time.sleep(RECEIVE_CHECK_INTERVAL)
                continue
            record = self.record_queue.pop()
            voice_stamped = self.stt.transcribe_to_stamped(file=record["file"], stamp=record["timestamp"])

            self.speak("Press enter to continue, or 'r' to try again!")
            if input().strip().lower() == "r":
                self.record_queue, self.gestures_queue = [], []
                print("Cleaned", flush=True)
                continue

            if len(self.gestures_queue) > 1:
                self.speak(f"There are {len(self.gestures_queue)} of gesturings, the last one is used, others are discarded")
            gesture_stamped = self.gestures_queue.pop().get_target_timestamped_list() if self.gestures_queue else []

            role_description = get_role_description(
                A=command_constraints["actions"],
                O=command_constraints["objects"],
                S=command_constraints["scene_text"],
            )
            skill_command = self.merge(gesture_stamped, voice_stamped,
                                       command_constraints=command_constraints,
                                       role_description=role_description)
            self.play_skillcommand(skill_command)

            self.record_queue, self.gestures_queue = [], []
            if EXIT_AFTER_EXECUTION:
                return

    def merge(self,
            gesture_stamped,
            voice_stamped,
            command_constraints: dict[str, list],  # valid (zero-object/single-object/double-object) actions
            role_description: str,  # for llm
            ):
        """ Main merge function """
        print(f"\n\n{cc.H}Merge function:{cc.E}")
        print(f"{cc.H}[1]{cc.E} Voice stamped: ", voice_stamped, flush=True)
        print(f"{cc.H}[2]{cc.E} Gesture stamped: ", gesture_stamped, flush=True)
        print_modalities(voice_stamped, gesture_stamped)

        # The gesture stream carries gesture names (e.g. "grab"); replace each
        # with the action it means (grab -> pick) so the LLM sees action words.
        # Object-grounding words (e.g. "cup1") pass through unchanged.
        gesture_stamped = self.gesture_mapping.map_stamped(gesture_stamped)
        print(f"{cc.H}[2b]{cc.E} Gestures as actions: ", gesture_stamped, flush=True)

        # Interleave by timestamp. On a tie, voice comes first: speech carries the
        # sentence structure (e.g. "to"), the gesture just grounds an object, so
        # "to" + <pointed object> reads naturally as "to bowl1", not "bowl1 to".
        sorted_sentence = sorted([*voice_stamped, *gesture_stamped], key=lambda x: x[0])
        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")

        final_sentence = " ".join([word for _, word in sorted_sentence if word is not None])
        self.speak(f"Merged sentence is: {final_sentence}, starting reasoner")

        # Constrained decoding: the LLM can only emit schema-valid JSON with
        # allowed values; we render it to a command deterministically below.
        schema = build_command_schema(command_constraints)
        structured = self.sentence_processor.predict_structured(
            final_sentence, role_description=role_description, schema=schema)

        print(f"{cc.W}LM says: {structured} {cc.E}")
        return SkillCommand.from_structured(structured, command_constraints)

    def save_log(self, true_sentence, skill_command, voice_stamped, gesture_stamped, scene, object_names,
                 cfg, role_description):
        data = {
            "successful": SkillCommand(true_sentence, cfg) == skill_command,
            "true_sentence": true_sentence,
            "predicted_sentence": skill_command.command,
            "predicted": skill_command.reasoning_text,
            "model_name": self.name(),
            "voice_stamped": [list(v) for v in voice_stamped],
            "gesture_stamped": [list(v) for v in gesture_stamped],
            "scene": scene,
            "object_names": object_names,
            "max_new_tokens": llm.MAX_NEW_TOKENS,
            "temperature": llm.TEMPERATURE,
            "top_p": llm.TOP_P,
            "repetition_penalty": llm.REPETITION_PENALTY,
            "cfg": cfg,
            "role_description": role_description,
        }

        i = 0
        while Path(f"{multi_modal_reasoning.path}/saved_logs/log_{i}.json").is_file():
            i += 1
        with open(f"{multi_modal_reasoning.path}/saved_logs/log_{i}.json", "w") as file:
            json.dump(data, file, indent=4)


def main():
    parser = argparse.ArgumentParser(description="Multi-modal reasoning merger node")
    parser.add_argument('--name_user', type=str, help='The user name (links/<user>_links.yaml holds the command constraints)', default="demo")
    args = parser.parse_args()

    command_constraints = load_user_links(args.name_user)

    rclpy.init()
    merger = ReasoningMerger()
    merger.speak("Ready!")
    merger.spin(command_constraints)


if __name__ == "__main__":
    main()
