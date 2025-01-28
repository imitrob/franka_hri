#!/usr/bin/env python
from skills_manager.lfd import LfD

from natural_language_processing.text_to_speech.kokoro_model import Chatterbox
from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
from natural_language_processing.speech_to_text.whisper_model import SpeechToTextModel
from natural_language_processing.sentence_instruct_transformer.sentence_processor import SentenceProcessor

from hri_manager.utils import cc

# import rclpy
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# import time
# import collections

from gesture_sentence_maker.gesture_sentence_getter import GestureSentenceGetter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import subprocess as sp
import os, json
import numpy as np

from lfd_msgs.srv import SetTemplate
from std_srvs.srv import Trigger

def get_gpu_memory():
    command = "nvidia-smi --query-gpu=memory.free --format=csv"
    memory_free_info = sp.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
    memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
    return memory_free_values

get_gpu_memory()
from pynput import keyboard

from hri_manager.feedback_for_hri import Feedback_for_HRI

class HRI(Feedback_for_HRI, LfD):
    def __init__(self,
                tts_enabled: bool = True,
                dry_run: bool = False,
                ):
        super(HRI, self).__init__()
        self.tts_enabled = tts_enabled
        self.dry_run = dry_run

        self.start()
        self.user = self.declare_parameter("user_name", "Melichar").get_parameter_value().string_value # replaced if launch

        print(f"memory left (whisper): {get_gpu_memory()}", flush=True)
        self.stt = SpeechToTextModel(device="cuda:0")
        print(f"memory left (speaker): {get_gpu_memory()}", flush=True)
        if tts_enabled:
            self.tts = Chatterbox(device="cpu")
            print(f"memory left (recorder): {get_gpu_memory()}", flush=True)
        self.rec = AudioRecorder()
        print(f"memory left (sentence processor): {get_gpu_memory()}", flush=True)
        self.sentence_processor = SentenceProcessor()
        print(f"memory left (done): {get_gpu_memory()}", flush=True)

        self.gestures = GestureSentenceGetter(self)

    def listen_user(self):
        self.rec.start_recording()
        input("Press enter to finish")
        return self.stt.forward(self.rec.stop_recording())

    def play_skill(self, name_skill: str, name_template: str, skill_parameter: float = 0.0):
        print(f"{cc.W}Executing {name_skill} with object {name_template} (same skill as template for simplicity){cc.E}", flush=True)
        if self.dry_run:
            print("Dry run; Returning", flush=True)
            return
        
        # Execution contraint
        # valid = name_template in name_skill # is "sponge" (template) in "pick_sponge" (skill)
        # if not valid:
        #     print(f"Template action is invalid {name_template} not in {name_skill}", flush=True)
        #     return
        
        self.set_localizer_client.call(SetTemplate.Request(template_name=name_template))
        self.move_template_start()
        self.active_localizer_client.call(Trigger.Request())
        self.compute_final_transform() 

        try:
            self.load(name_skill)
            # self.load_morph_trajectory(skill_parameter, morph_parameter=skill_parameter)
            print(f"Execution", flush=True)
            self.execute()
        except KeyboardInterrupt:
            pass

    def speak(self, text):
        if self.tts_enabled:
            self.tts.speak(text)
        print("", flush=True)
        print(text, flush=True)
        print("", flush=True)
        
    def nl_forward(self, recording_name: str):
        print(f"1. Speech to text; file: {recording_name}", flush=True)
        sentence_text = self.stt.forward(recording_name)
        print("Sentence text: ", sentence_text, flush=True)
        print("2. Sentence processing", flush=True)
        output = self.sentence_processor.predict(sentence_text)

        for k in output.keys():
            if isinstance(output[k], np.ndarray):
                output[k] = list(output[k])

        target_action = map_instruction_words(output["target_action"], self.user)
        self.play_skill(target_action, target_action)

    def listen_for_voice_commands(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            print(f"Press and hold 'r' record...", flush=True)
            listener.join()

    def load_morph_trajectory(self, name_trajectory, morph_parameter: float):
        
        Ts = []
        for file in [name_trajectory, name_trajectory+"_alt"]:
            data = np.load(trajectory_data.package_path + '/trajectories/' + str(file) + '.npz')
            
            T = {}
            T['recorded_traj'] = data['traj']
            T['recorded_ori_wxyz'] = data['ori']
            T['recorded_gripper'] = data['grip']
            T['recorded_img'] = data['img']
            T['recorded_img_feedback_flag'] = data['img_feedback_flag']
            T['recorded_spiral_flag'] = data['spiral_flag']
            if self.final_transform is not None:
                T['recorded_traj'], T['recorded_ori_wxyz'] = self.transform_traj_ori(T['recorded_traj'], T['recorded_ori_wxyz'], self.final_transform)
            T['filename']=str(file)
            Ts.append(T)

        data =  morth_trajectories(Ts, morph_parameter)

        self.recorded_traj = data['traj']
        self.recorded_ori_wxyz = data['ori']
        self.recorded_gripper = data['grip']
        self.recorded_img = data['img']
        self.recorded_img_feedback_flag = data['img_feedback_flag']
        self.recorded_spiral_flag = data['spiral_flag']
        self.filename=str(file)

import trajectory_data, object_localization, hri_manager, yaml
from pathlib import Path

class Link():
    def __init__(self,
                 user_name, 
                 action_template, 
                 object_template, 
                 action_word, 
                 action_gesture,
                 ):
        self.user_name = user_name 
        self.action_template = action_template 
        self.object_template = object_template 
        self.action_word = action_word 
        self.action_gesture = action_gesture

    def save(self):
        new_link = {
            "user": self.user_name,
            "action_template": self.action_template, # e.g., push
            "object_template": self.object_template, # e.g., cube_template - cube
            "action_word": self.action_word, # e.g., "push"
            "action_gesture": self.action_gesture, # e.g., "grab" + "swipe right"
        }
        try:
            data_dict = yaml.safe_load(open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="r"))
        except FileNotFoundError:
            data_dict = {
                    "user": self.user_name,
                    "actions": [],
                    "action_words": [],
                    "links": {},
                }
            with open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="w") as file:
                yaml.safe_dump(data_dict, file, sort_keys=False)

        # Add the action_template to actions list if not present
        if new_link["action_template"] not in data_dict["actions"]:
            data_dict["actions"].append(new_link["action_template"])

        if new_link["action_word"] not in data_dict["action_words"]:
            data_dict["action_words"].append(new_link["action_word"])

        # Add the new link to links with a unique key
        new_link_name = f"link{len(data_dict['links']) + 1}"  # Generate unique link name
        data_dict["links"][new_link_name] = new_link
        

        print("data_dict", data_dict)
        with open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="w") as file:
            yaml.safe_dump(data_dict, file, sort_keys=False)

    def check_valid(self,hri):
        #does_action_exist(action_template)
        
        action_template_path = f"{trajectory_data.package_path}/trajectories/{self.action_template}.npz"
        assert Path(action_template_path).is_file(), f"{action_template_path} doesn't exists!"

        # does_object_template_exist(object_template)
        object_template_path = f"{object_localization.package_path}/cfg/{self.object_template}"
        assert Path(object_template_path).is_dir(), f"{object_template_path} doesn't exists!"

        # does_action_gesture_exist(action_gesture)
        assert self.action_gesture[0] in hri.gestures.Gs_static, f"{self.action_gesture[0]} is not in {hri.gestures.Gs_static}"
        assert self.action_gesture[1] in hri.gestures.Gs_dynamic, f"{self.action_gesture[1]} is not in {hri.gestures.Gs_dynamic}" 


import yaml
import hri_manager

def map_instruction_words(action, user:str):
    links_dict = yaml.safe_load(open(f"{hri_manager.package_path}/links/{user}_links.yaml", mode='r'))
    
    action = action.lower() # user says e.g., "I eehhh pick up eeeehhh. "
    for _,link in links_dict['links'].items(): # link action word is strictly "pick up"
        if link['action_word'].lower() in action:  # checks if "pick up" is in the "I eehhh pick up eeeehhh. ". 
            return link["action_template"]
    return ""



def morth_trajectories(Ts, morph_parameter: float):
    # TODO
    return Ts[0]

def main():
    import rclpy
    rclpy.init()
    hri = HRI()

if __name__ == "__main__":
    main()