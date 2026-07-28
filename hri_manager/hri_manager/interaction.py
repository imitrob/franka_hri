"""Shared human-interaction functions for interactive nodes (HRI and
ReasoningMerger): model clients, voice recording, speaking, and publishing
skill commands to the robot node. Every model runs in its own server; the
node only holds clients."""
import json

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
from natural_language_processing.speech_to_text.stt_client import SpeechToTextClient
from natural_language_processing.text_to_speech.tts_client import TextToSpeechClient
from multi_modal_reasoning.models.llm import SentenceProcessor
from multi_modal_reasoning.skill_command import SkillCommand
from naive_merger.utils import cc

SKILL_COMMAND_TOPIC = "/hri/skill_command"  # the robot (LfD) node subscribes here


class InteractionNode():
    """Mixin for an rclpy Node. Call init_interaction() once after the node
    is initialized."""

    def init_interaction(self):
        # Models run in separate servers, these are just clients.
        # The LLM client is lazy: created on first use, so nodes that never
        # reason (e.g. action_executor) run without a vLLM server.
        self._sentence_processor = None
        self.stt = SpeechToTextClient()
        self.tts = TextToSpeechClient()
        self.rec = AudioRecorder()

        self.voicerecord_pub = self.create_publisher(String, "/recorded_file",
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        # Robot output: the robot (LfD) node subscribes here and executes the
        # published skill commands
        self.skill_command_pub = self.create_publisher(String, SKILL_COMMAND_TOPIC,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))

    @property
    def sentence_processor(self):
        if self._sentence_processor is None:
            self._sentence_processor = SentenceProcessor()
        return self._sentence_processor

    def delete_interaction(self):
        self.tts.delete()
        self.stt.delete()
        if self._sentence_processor is not None:
            self._sentence_processor.delete()

    def speak(self, text):
        self.tts.speak(text)
        print(f"\n{text}\n", flush=True)

    def record_voice(self):
        """Enter starts and stops the recording (terminal input, no global
        keyboard listener); the file is announced on /recorded_file."""
        input("Press enter to START recording...")
        self.rec.start_recording()
        input("Recording! Press enter to STOP...")
        file, start_time = self.rec.stop_recording()
        if file is not None:
            self.voicerecord_pub.publish(String(data=json.dumps({"file": file, "timestamp": start_time})))
            print("Voice recorded and msg sent", flush=True)

    def play_skillcommand(self, skillcommand: SkillCommand):
        """Hand the command over to the robot (LfD) node via ROS2."""
        print(f"{cc.W}Playing skill command: {skillcommand}{cc.E}")
        if not skillcommand.is_valid():
            self.speak(f"Skill Command is not valid ({skillcommand.invalid_reason}), returning!")
            return
        self.speak(f"Executing: {skillcommand}")
        self.skill_command_pub.publish(String(data=json.dumps(skillcommand.to_dict())))
