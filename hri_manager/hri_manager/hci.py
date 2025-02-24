import hri_manager, yaml
from pathlib import Path
from natural_language_processing.text_to_speech.kokoro_model import Chatterbox
from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
from natural_language_processing.speech_to_text.whisperx_model import SpeechToTextModel
from natural_language_processing.speech_to_text.whisper_probabilistic_model import SpeechToTextModel as ProbabilisticSpeechToTextModel
from natural_language_processing.sentence_instruct_transformer.sentence_processor import SentenceProcessor
from llm_merger.models.llm import ProbabilisticSentenceProcessor
from gesture_sentence_maker.gesture_sentence_getter import GestureSentenceGetter

import subprocess
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from skills_manager.ros_utils import SpinningRosNode
from hri_manager.feedback_for_hri import Feedback_for_HRI
from naive_merger.utils import cc

from std_msgs.msg import String

from hri_manager.user_preference_getter import UserPreferenceGetter

def get_gpu_memory():
    command = "nvidia-smi --query-gpu=memory.free --format=csv"
    memory_free_info = subprocess.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
    memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
    return memory_free_values

class HCI(UserPreferenceGetter, Feedback_for_HRI, SpinningRosNode):
    def __init__(self,
                 name_user = None,
                 nlp_model_name = None,
                 tts_enabled = None,
                 stt_type: str = "deterministic",
                 ):
        if name_user is not None: self.user = name_user
        if nlp_model_name is not None: self.nlp_model_name = nlp_model_name
        if tts_enabled is not None: self.tts_enabled = tts_enabled
        if stt_type is not None: self.stt_type = stt_type
        super(HCI, self).__init__()

        assert Path(f"{hri_manager.package_path}/links/{self.user}_links.yaml").is_file()
        print(f"1/3 Init STT: VRAM memory left: {get_gpu_memory()}", flush=True)

        if self.stt_type == "deterministic":
            self.stt = SpeechToTextModel(device="cuda") # you might want to offload to cpu
        elif self.stt_type == "probabilistic":
            self.stt = ProbabilisticSpeechToTextModel(device="cuda")
        else: raise Exception()

        if self.tts_enabled:
            print(f"2/3 Init TTS: VRAM memory left: {get_gpu_memory()}", flush=True)
            self.tts = Chatterbox(device="cuda") # you might want to offload to cpu
        self.rec = AudioRecorder()
        print(f"3/3 Init LM: VRAM memory left: {get_gpu_memory()}", flush=True)
        
        if self.stt_type == "deterministic":
            self.sentence_processor = SentenceProcessor(model_name=self.nlp_model_name)
        elif self.stt_type == "probabilistic":
            self.sentence_processor = ProbabilisticSentenceProcessor(model_name=self.nlp_model_name)
        print(f"{cc.H}Initialization Done{cc.E}: VRAM memory left: {get_gpu_memory()}", flush=True)

        self.gestures = GestureSentenceGetter(self)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.voicerecord_pub = self.create_publisher(String, "/recorded_file", qos)
    
    def listen_user(self):
        self.rec.start_recording()
        input("Press enter to finish")
        file, _ = self.rec.stop_recording()
        return self.stt(file)

    def speak(self, text):
        if self.tts_enabled:
            self.tts.speak(text)
        print("", flush=True)
        print(text, flush=True)
        print("", flush=True)
        
    def play_skill(self, name_skill, name_template, skill_parameter: float = None, simplify=True):
        if name_skill == "":
            self.speak(f"No action found, try again")
            return 

        if name_template == "" and simplify:
            name_template = name_skill # simplified
            self.speak(f"No object specified! The object is set to {name_template} because Running simplified!")
        elif simplify:
            self.speak(f"Your object: {name_template} is changed to {name_skill}, because Running simplified!")
            name_template = name_skill # simplified

        self.speak(f"Executing {name_skill} with object {name_template}!")

        print("Dry run; Returning", flush=True)
        return
    
