import hri_manager, yaml
from pathlib import Path
from natural_language_processing.text_to_speech.kokoro_model import Chatterbox
from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
from natural_language_processing.speech_to_text.whisperx_model import SpeechToTextModel
from natural_language_processing.sentence_instruct_transformer.sentence_processor import SentenceProcessor
from gesture_sentence_maker.gesture_sentence_getter import GestureSentenceGetter

import subprocess
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from skills_manager.ros_utils import SpinningRosNode
from hri_manager.feedback_for_hri import Feedback_for_HRI
import numpy as np

from std_msgs.msg import String

def get_gpu_memory():
    command = "nvidia-smi --query-gpu=memory.free --format=csv"
    memory_free_info = subprocess.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
    memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
    return memory_free_values

class HCI(Feedback_for_HRI, SpinningRosNode):
    def __init__(self,
                 name_user = None,
                 nlp_model_name = None,
                 tts_enabled = None,
                 ):
        if name_user is not None: self.user = name_user
        if nlp_model_name is not None: self.nlp_model_name = nlp_model_name
        if tts_enabled is not None: self.tts_enabled = tts_enabled
        super(HCI, self).__init__()

        assert Path(f"{hri_manager.package_path}/links/{self.user}_links.yaml").is_file()
        print(f"1/3 Init STT: VRAM memory left: {get_gpu_memory()}", flush=True)
        self.stt = SpeechToTextModel(device="cuda") # you might want to offload to cpu
        if self.tts_enabled:
            print(f"2/3 Init TTS: VRAM memory left: {get_gpu_memory()}", flush=True)
            self.tts = Chatterbox(device="cuda") # you might want to offload to cpu
        self.rec = AudioRecorder()
        print(f"3/3 Init LM: VRAM memory left: {get_gpu_memory()}", flush=True)
        self.sentence_processor = SentenceProcessor(model_name=self.nlp_model_name)
        print(f"Done: VRAM memory left: {get_gpu_memory()}", flush=True)

        self.gestures = GestureSentenceGetter(self)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.voicerecord_pub = self.create_publisher(String, "/recorded_file", qos)
    
    def listen_user(self):
        self.rec.start_recording()
        input("Press enter to finish")
        return self.stt.forward(self.rec.stop_recording())

    def speak(self, text):
        if self.tts_enabled:
            self.tts.speak(text)
        print("", flush=True)
        print(text, flush=True)
        print("", flush=True)
        
    def play_skill(self, target_action, target_object):
        print(f"Playing skill: {target_action} with {target_object}", flush=True)


def map_instruction_words(output, user:str):
    action = output['target_action'] # e.g., open
    object = output['target_object'] # e.g., drawer
    mapped_action = ""
    mapped_object = ""

    links_dict = yaml.safe_load(open(f"{hri_manager.package_path}/links/{user}_links.yaml", mode='r'))
    
    action = action.lower() 
    for _,link in links_dict['links'].items(): 
        for action_wd in link['action_words']:
            if action_wd.lower() in action:  
                mapped_action = link["action_template"]
                break
        
    action = action.lower()
    for _,link in links_dict['links'].items():
        for object_wd in link['object_action_words']:
            if object_wd.lower() in object: 
                mapped_object = link["object_template"]
                break

    return mapped_action, mapped_object
