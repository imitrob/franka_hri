import hri_manager, yaml
from pathlib import Path
from natural_language_processing.text_to_speech.kokoro_model import Chatterbox
from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
from natural_language_processing.speech_to_text.whisper_model import SpeechToTextModel
from gesture_sentence_maker.gesture_sentence_getter import GestureSentenceGetter

import subprocess
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from skills_manager.ros_utils import SpinningRosNode
from hri_manager.feedback_for_hri import Feedback_for_HRI
from naive_merger.utils import cc

from std_msgs.msg import String

from hri_manager.user_preference_getter import UserPreferenceGetter

from scene_getter.scene_getting import SceneGetter

def get_gpu_memory():
    command = "nvidia-smi --query-gpu=memory.free --format=csv"
    memory_free_info = subprocess.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
    memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
    return memory_free_values


import torch, gc, time

def clean_vram():
    gc.collect()
    torch.cuda.empty_cache()

class HCI(SceneGetter, UserPreferenceGetter, Feedback_for_HRI, SpinningRosNode):
    def __init__(self,
                 name_user = None,
                 nlp_model_name = None,
                 tts_enabled = None,
                 stt_type: str = "deterministic",
                 stt_enabled = None,
                 ):
        if name_user is not None: self.user = name_user
        if nlp_model_name is not None: self.nlp_model_name = nlp_model_name
        if tts_enabled is not None: self.tts_enabled = tts_enabled
        if stt_type is not None: self.stt_type = stt_type
        if stt_enabled is not None: self.stt_enabled = stt_enabled
        super(HCI, self).__init__()

        assert Path(f"{hri_manager.package_path}/links/{self.user}_links.yaml").is_file()
        print(f"0/3 VRAM memory left: {get_gpu_memory()}", flush=True)
        print(f"Memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
        print(f"Memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")

        if self.stt_enabled:
            if self.stt_type == "deterministic":
                self.stt = SpeechToTextModel(device="cuda") # you might want to offload to cpu
            elif self.stt_type == "probabilistic":
                self.stt = SpeechToTextModel(device="cuda")
            elif self.stt_type == "alternatives":
                self.stt = SpeechToTextModel(device="cuda")

        if self.tts_enabled:
            print(f"1/3 Inited SST: VRAM memory left: {get_gpu_memory()}", flush=True)
            print(f"Memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
            print(f"Memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")

            self.tts = Chatterbox(device="cuda") # you might want to offload to cpu
        self.rec = AudioRecorder()
        print(f"2/3 Inited TTS: VRAM memory left: {get_gpu_memory()}", flush=True)
        print(f"Memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
        print(f"Memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")

        self.gestures = GestureSentenceGetter(self)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.voicerecord_pub = self.create_publisher(String, "/recorded_file", qos)

    def delete(self):
        if self.tts_enabled:
            self.tts.delete()
        if self.stt_enabled:
            self.stt.delete()
        self.sentence_processor.delete()
        time.sleep(1.0)
        clean_vram()
        print(f"{cc.H}Cleaning done{cc.E}: VRAM memory left: {get_gpu_memory()}", flush=True)
        print(f"Memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
        print(f"Memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")


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
    
