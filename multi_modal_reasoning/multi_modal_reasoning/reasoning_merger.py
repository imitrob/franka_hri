#!/usr/bin/env python
"""Multi-modal reasoning merger (ROS2 node).

Pipeline: voice (STT) and gestures arrive as timestamped word streams, get
interleaved by timestamp into one sentence, sent to the LLM (vLLM server,
see models/llm.py) and the reply is parsed into a SkillCommand, which is
published on SKILL_COMMAND_TOPIC for the robot (LfD) node to execute.

This class replaces the former HRI/HCI stack: speech-to-text, text-to-speech
and keyboard interaction live here; the LLM runs in a separate vLLM server;
the robot runs in a separate node. Only "deterministic" interpretation.
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

import multi_modal_reasoning
from multi_modal_reasoning.models import llm
from multi_modal_reasoning.models.llm import SentenceProcessor
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.skill_command import SkillCommand, build_command_schema
from multi_modal_reasoning.utils import print_modalities
from naive_merger.utils import cc

RECEIVE_CHECK_INTERVAL = 1.0  # [s]
EXIT_AFTER_EXECUTION = True
SKILL_COMMAND_TOPIC = "/hri/skill_command"  # the robot (LfD) node subscribes here

# Command constraints: the valid action/object vocabulary for a scene. Used both
# to build the LLM prompt (actions/objects/scene_text) and to constrain and parse
# its output (the *_object_actions arity lists, adjectives, prepositions). Select
# one in main() and pass it through spin() -> merge() as `command_constraints`.
CONFIG_DEMO = {
    "actions": ["stop", "pick", "touch", "put", "push"],
    "directional_actions": [],
    "zero_object_actions": ["stop"],
    "single_object_actions": ["pick", "touch", "push"],
    "double_object_actions": ["put"],
    "adjectives": ["fast", "slow", "force"],
    "prepositions": ["to"],
    "objects": ["cube", "bowl", "drawer", "banana", "box"],
    "scene_text": "In a scene is a yellow plastic banana, metal red bowl, plastic green cube, paper rectangular box. ",
}

CONFIG3 = {
    "actions": ["pick", "push", "pass", "place", "point", "open", "close", "put", "stop", "release", "home"],
    "directional_actions": [],
    "zero_object_actions": ["stop", "release", "home"],
    "single_object_actions": ["pick", "push", "pass", "point", "open", "close"],
    "double_object_actions": ["place", "put"],
    "adjectives": ["fast", "slow", "force"],
    "prepositions": ["to"],
    "objects": ["cup", "cube", "plate", "table", "can", "box", "fork", "marker", "note", "storage", "blade", "rack", "ledge", "stand", "platform"],
    "scene_text": "",
}


class ReasoningMerger(Node):
    def __init__(self,
                tts_enabled: bool = True,
                stt_enabled: bool = True,
                ):
        """
        Args:
            tts_enabled (bool, optional): Load text-to-speech model
            stt_enabled (bool, optional): Load speech-to-text model + audio recorder

        The reasoning model is whatever the vLLM server is serving; it is
        discovered from the server (see models/llm.py) and available as
        `self.model_name` for logging.
        """
        super().__init__(f"reasoning_merger_{np.random.randint(100000)}")

        # LLM runs in a separate vLLM server, this is just a client
        self.sentence_processor = SentenceProcessor()
        self.model_name = self.sentence_processor.model_name

        # Interaction peripherals (moved from HCI), imported lazily so that
        # disabled peripherals don't pull in their (GPU) dependencies
        self.tts = None
        if tts_enabled:
            from natural_language_processing.text_to_speech.kokoro_model import Chatterbox
            self.tts = Chatterbox(device="cuda")
        self.stt = None
        self.rec = None
        if stt_enabled:
            from natural_language_processing.speech_to_text.whisper_model import SpeechToTextModel
            from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
            self.stt = SpeechToTextModel(device="cuda")
            self.rec = AudioRecorder()

        # Modality inputs
        self.record_queue = []
        self.gestures_queue = []
        self.create_subscription(HRICommandMSG, "/modality/gestures", self.gesture_callback,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
        self.create_subscription(String, "/recorded_file", self.voice_record_callback,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.voicerecord_pub = self.create_publisher(String, "/recorded_file",
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        # Robot output (moved from HRI, which called LfD directly)
        self.skill_command_pub = self.create_publisher(String, SKILL_COMMAND_TOPIC,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))

        # Keyboard state (moved from Feedback_for_HRI): "+" (hold) records voice,
        # enter executes, "-" retries
        self.wait_action = ""
        self.is_recording = False
        self._keyboard_listener = None

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

    def speak(self, text):
        if self.tts is not None:
            self.tts.speak(text)
        print(f"\n{text}\n", flush=True)

    def keyboard_start(self):
        """'+' (hold) records voice, enter = execute, '-' = try again."""
        from pynput.keyboard import Key, KeyCode, Listener

        def on_press(key):
            if key == KeyCode.from_char("+") and not self.is_recording:
                self.is_recording = True
                self.rec.start_recording()
            elif key == KeyCode.from_char("-"):
                self.wait_action = "again"
            elif key == Key.enter:
                self.wait_action = "exec"

        def on_release(key):
            if key == KeyCode.from_char("+") and self.is_recording:
                self.is_recording = False
                file, start_time = self.rec.stop_recording()
                if file is not None:
                    self.voicerecord_pub.publish(String(data=json.dumps({"file": file, "timestamp": start_time})))
                    print("Voice recorded and msg sent", flush=True)

        self._keyboard_listener = Listener(on_press=on_press, on_release=on_release)
        self._keyboard_listener.start()

    def voice_record_callback(self, msg):
        print(f"Received: {cc.W}Recording{cc.E}", flush=True)
        self.record_queue.append(json.loads(msg.data))

    def gesture_callback(self, msg):
        hricommand = HriCommand.from_ros(msg)
        print(f"Received: {cc.W}Gestures{cc.E}", flush=True)
        self.gestures_queue.append(hricommand)

    def delete(self):
        if self.tts is not None:
            self.tts.delete()
        if self.stt is not None:
            self.stt.delete()
        if self._keyboard_listener is not None:
            self._keyboard_listener.stop()
        self.sentence_processor.delete()
        self.destroy_node()

    # ------------------------------------------------------------------ #
    # Main loop                                                           #
    # ------------------------------------------------------------------ #
    def spin(self, command_constraints, role_version: str = "structured"):
        """Interactive loop: record voice ('+'), confirm (enter), merge with
        the last gesturing, publish the resulting skill command."""
        while rclpy.ok():
            time.sleep(RECEIVE_CHECK_INTERVAL)
            if len(self.record_queue) == 0:
                continue
            record = self.record_queue.pop()
            voice_stamped = self.stt.transcribe_to_stamped(file=record["file"], stamp=record["timestamp"])

            self.wait_action = ""
            self.speak("Press enter to continue or minus to try again!")
            while self.wait_action == "":
                time.sleep(0.5)
            if self.wait_action == "again":
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
                version=role_version,
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

    def play_skillcommand(self, skillcommand: SkillCommand):
        """Hand the command over to the robot (LfD) node via ROS2 (moved from
        HRI, which executed LfD skills in-process)."""
        print(f"{cc.W}Playing skill command: {skillcommand}{cc.E}")
        if not skillcommand.is_valid():
            self.speak("Skill Command is Not valid, returning!")
            return
        self.speak(f"Executing: {skillcommand}")
        self.skill_command_pub.publish(String(data=json.dumps({
            "command": skillcommand.command,
            "target_action": skillcommand.target_action,
            "target_object": skillcommand.target_object,
            "object_preposition": skillcommand.object_preposition,
            "target_object2": skillcommand.target_object2,
            "action_parameter": skillcommand.action_parameter,
        })))

    # ------------------------------------------------------------------ #
    # Logging                                                             #
    # ------------------------------------------------------------------ #
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
    parser.add_argument('--config_name', type=str, help='config_name', default="CONFIG_DEMO", choices=["CONFIG_DEMO", "CONFIG3"])
    parser.add_argument('--role_version', type=str, help='Role version spec', default="structured")
    args = parser.parse_args()

    command_constraints = {"CONFIG_DEMO": CONFIG_DEMO, "CONFIG3": CONFIG3}[args.config_name]

    rclpy.init()
    merger = ReasoningMerger()
    merger.keyboard_start()
    merger.speak("Ready! Hold a plus to record voice.")
    merger.spin(command_constraints, role_version=args.role_version)


if __name__ == "__main__":
    main()
