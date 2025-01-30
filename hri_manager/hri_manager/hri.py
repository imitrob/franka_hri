#!/usr/bin/env python
from skills_manager.lfd import LfD

from natural_language_processing.text_to_speech.kokoro_model import Chatterbox
from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder
from natural_language_processing.speech_to_text.whisper_model import SpeechToTextModel
from natural_language_processing.sentence_instruct_transformer.sentence_processor import SentenceProcessor

from gesture_sentence_maker.gesture_sentence_getter import GestureSentenceGetter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import subprocess as sp
from pathlib import Path
import numpy as np

from lfd_msgs.srv import SetTemplate
from std_srvs.srv import Trigger

def get_gpu_memory():
    command = "nvidia-smi --query-gpu=memory.free --format=csv"
    memory_free_info = sp.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
    memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
    return memory_free_values

get_gpu_memory()

from hri_manager.feedback_for_hri import Feedback_for_HRI
from skills_manager.skill import Skill

class HRI(Feedback_for_HRI, LfD):
    def __init__(self,
                name_user: str,
                tts_enabled: bool = True,
                dry_run: bool = False,
                ):
        super(HRI, self).__init__()
        self.tts_enabled = tts_enabled
        self.dry_run = dry_run
        self.user = name_user
        assert Path(f"{hri_manager.package_path}/links/{self.user}_links.yaml").is_file()

        self.start()

        print(f"1/3 Init STT: VRAM memory left: {get_gpu_memory()}", flush=True)
        self.stt = SpeechToTextModel(device="cuda:0") # you might want to offload to cpu
        if tts_enabled:
            print(f"2/3 Init TTS: VRAM memory left: {get_gpu_memory()}", flush=True)
            self.tts = Chatterbox(device="cuda:0") # you might want to offload to cpu
        self.rec = AudioRecorder()
        print(f"3/3 Init LM: VRAM memory left: {get_gpu_memory()}", flush=True)
        self.sentence_processor = SentenceProcessor()
        print(f"Done: VRAM memory left: {get_gpu_memory()}", flush=True)

        self.gestures = GestureSentenceGetter(self)


    def listen_user(self):
        self.rec.start_recording()
        input("Press enter to finish")
        return self.stt.forward(self.rec.stop_recording())

    def play_skill(self, name_skill: str, name_template: str, skill_parameter: float = None, simplify=True):
        """ When simplify==True, target_object == target_actopm
        """
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
            if skill_parameter is not None:
                self.load_morph_trajectory(skill_parameter, morph_parameter=skill_parameter)
            else:
                self.load(name_skill)
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
        self.speak(f"You said: {sentence_text}")
        print("2. Sentence processing", flush=True)
        output = self.sentence_processor.predict(sentence_text)

        for k in output.keys():
            if isinstance(output[k], np.ndarray):
                output[k] = list(output[k])

        print("Sentence processing output: ", output, flush=True)

        target_action, target_object = map_instruction_words(output, self.user)
        self.play_skill(target_action, target_object)


    def load_morph_trajectory(self, name_trajectory, morph_parameter: float):
        
        skill1 = Skill().from_file(name_trajectory)
        skill2 = Skill().from_file(name_trajectory+"_alt")

        morph_skill = skill1.morth_trajectories(skill2, morph_parameter)

        self.recorded_traj = morph_skill.traj_T
        self.recorded_ori_wxyz = morph_skill.ori_T
        self.recorded_gripper = morph_skill.grip_T
        self.recorded_img = morph_skill.img
        self.recorded_img_feedback_flag = morph_skill.img_feedback_flag_T
        self.recorded_spiral_flag = morph_skill.spiral_flag_T
        self.filename=str(morph_skill.filename)

import trajectory_data, object_localization, hri_manager, yaml
from pathlib import Path

class Link():
    def __init__(self,
                 user_name, 
                 action_template, 
                 object_template, 
                 action_words, 
                 object_words,
                 action_gestures,
                 ):
        self.user_name = user_name 
        self.action_template = action_template 
        self.object_template = object_template 
        self.action_words = action_words 
        self.object_words = object_words
        self.action_gestures = action_gestures

    def save(self):
        new_link = {
            "user": self.user_name,
            "action_template": self.action_template, # e.g., push
            "object_template": self.object_template, # e.g., cube_template - cube
            "action_words": list(self.action_words), # e.g., ["push"]
            "object_words": list(self.object_words),
            "action_gestures": self.action_gestures, # e.g., "grab" + "swipe right"
        }
        try:
            data_dict = yaml.safe_load(open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="r"))
        except FileNotFoundError:
            data_dict = {
                    "user": self.user_name,
                    "actions": [],
                    "all_action_words": [],
                    "all_object_action_words": [],
                    "links": {},
                }
            with open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="w") as file:
                yaml.safe_dump(data_dict, file, sort_keys=False)

        # Add the action_template to actions list if not present
        if new_link["action_template"] not in data_dict["actions"]:
            data_dict["actions"].append(new_link["action_template"])

        for action_word in new_link["action_words"]: 
            if action_word not in data_dict["all_action_words"]:
                data_dict["all_action_words"].append(action_word)

        for object_action_word in new_link["object_words"]:
            if object_action_word not in data_dict["all_object_action_words"]:
                data_dict["all_object_action_words"].append(object_action_word)

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
        for action_gesture in self.action_gestures:
            assert action_gesture[0] in hri.gestures.Gs_static, f"{action_gesture[0]} is not in {hri.gestures.Gs_static}"
            assert action_gesture[1] in hri.gestures.Gs_dynamic, f"{action_gesture[1]} is not in {hri.gestures.Gs_dynamic}" 


import yaml
import hri_manager

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

def main():
    import rclpy
    rclpy.init()
    hri = HRI()

if __name__ == "__main__":
    main()