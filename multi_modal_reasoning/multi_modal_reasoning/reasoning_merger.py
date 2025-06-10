
import numpy as np
from hri_manager.hri import HRI
from hri_manager.hci import HCI

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG
from std_msgs.msg import String
from hri_manager.HriCommand import HriCommand

import rclpy, time, json
from naive_merger.utils import cc
from multi_modal_reasoning.skill_command import SkillCommand

import multi_modal_reasoning
import argparse
from pathlib import Path
from multi_modal_reasoning.utils import print_modalities
from transformers import pipeline
from multi_modal_reasoning.models.llm import SentenceProcessor
from playsound import playsound

RECEIVE_CHECK_INTERVAL = 1.0 # [s]
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.generate_dataset import CONFIG3, CONFIG_DEMO
import torch

class ReasoningMerger():
    def __init__(self,
                name_user: str,
                model_name: str,
                dry_run: bool = True,
                interpret_format: str = "deterministic",
                tts_enabled = True,
                stt_enabled = True,
                ):
        """_summary_

        Args:
            name_user (str): Loads "hri_manager/links/<user>_links.yaml" where the user config is stored
            model_name (str): _description_
            dry_run (bool, optional): _description_. Defaults to True.
            interpret_format (str): "deterministic" or "probabilistic" interpret format
        """   
        self.interpret_format = interpret_format
        self.dry_run = dry_run
        if dry_run: # No robot
            self.hri = HCI(
                name_user = name_user,
                nlp_model_name = model_name,
                tts_enabled = tts_enabled,
                stt_type = self.interpret_format,
                stt_enabled = stt_enabled,
            )
        else: # With robot control
            self.hri = HRI(
                name_user = name_user,
                tts_enabled = tts_enabled,
                dry_run = False,
                nlp_model_name = model_name,
                stt_type = self.interpret_format,
                stt_enabled = stt_enabled,
            )
        self.model_name = model_name
        self.hri.create_subscription(HRICommandMSG, '/modality/gestures', self.gesture_hricommand_callback, qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
        self.hri.create_subscription(String, '/recorded_file', self.receive_voice_record, qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.record_queue = []
        self.gestures_queue = []

        self.hri.superwaitexec = "" 
        self.hri.sentence_processor = SentenceProcessor(model_name=self.hri.nlp_model_name, quantization=16)

    def name(self):
        return self.model_name.split("/")[-1]

    def spin(self, *args, **kwargs):
        while rclpy.ok():
            time.sleep(RECEIVE_CHECK_INTERVAL)
            if len(self.record_queue) > 0:
                record_stamped_file_name = self.record_queue.pop()
                if self.interpret_format == "deterministic":
                    voicecommand = self.hri.stt.transcribe_to_stamped(file=record_stamped_file_name['file'], stamp=record_stamped_file_name['timestamp'])
                elif self.interpret_format in ["probabilistic", "alternatives"]:
                    voicecommand = self.hri.stt.transcribe_to_probstamped(file=record_stamped_file_name['file'], stamp=record_stamped_file_name['timestamp'])
                else: raise Exception

                self.hri.superwaitexec = ""
                self.hri.speak(f"Press enter to continue or minus to try again!")
                playsound("/home/imitlearn/Downloads/alert1.wav")
                # User presses Enter to execute or minus to to try again
                while self.hri.superwaitexec == "":
                    time.sleep(0.5)
                if self.hri.superwaitexec == "again":
                    self.record_queue = []
                    self.gestures_queue = []
                    print("Cleaned", flush=True)
                    continue
                self.hri.superwaitexec = ""
                    
                if len(self.gestures_queue) > 1:
                    self.hri.speak(f"There are {len(self.gestures_queue)} of gesturings, the last one is used, others are discarded")
                if len(self.gestures_queue) > 0:
                    hricommand = self.gestures_queue.pop()
                    print()
                    print("Gesture command is:")
                    print(hricommand)
                    print("Voice command is:")
                    print(voicecommand)
                    print()
                    
                    # if self.dry_run:
                    #     self.save_command(voicecommand, hricommand)
                    # else:
                    self.extract_merge_and_play(voicecommand, hricommand, *args, **kwargs)
                else:
                    print()
                    print("Voice command is:")
                    print(voicecommand)
                    print()
                    # if self.dry_run:
                    #     self.save_command(voicecommand, [])
                    # else:
                    self.extract_merge_and_play(voicecommand, None,  *args, **kwargs)

                self.record_queue = []
                self.gestures_queue = []

    def receive_voice_record(self, msg):
        msg_dict = json.loads(str(msg.data))
        print(f"Received: {cc.W}Recording{cc.E}", flush=True)
        self.record_queue.append(msg_dict)
        playsound("/home/imitlearn/Downloads/alert2.wav")

    def gesture_hricommand_callback(self, msg):
        hricommand = HriCommand.from_ros(msg)
        print(hricommand.get_target_timestamped_list())
        print(f"Received: {cc.W}Gestures{cc.E}", flush=True)
        self.gestures_queue.append(hricommand)
        playsound("/home/imitlearn/Downloads/alert2.wav")

    def extract_merge_and_play(self, voicecommand, hricommand, cfg, role_version, *args, **kwargs):
        print("extract merge and play")

        

        # extract
        if self.interpret_format == "deterministic":
            """ gesture_stamped: [:,0] - timestamps, [:,1] - words 
            voice_stamped:   [:,0] - timestamps, [:,1] - words
            """      
            if hricommand is not None:
                gesture_stamped = hricommand.get_target_timestamped_list()
                voice_stamped = voicecommand
            else: 
                gesture_stamped = []
                voice_stamped = voicecommand

        elif self.interpret_format == "probabilistic":
            """ both gesture_stamped and voice_stamped in format:
            [ # time,  word  : probs, ...
                [0.0, {"pick": 1.0, "kick": 0.2}],
                [0.1, {"up": 0.96, "lap": 0.1}],
            ]
            """
            if hricommand is not None:
                gesture_stamped = hricommand.get_target_timestamped_probabilistic()
                voice_stamped = voicecommand
            else: 
                gesture_stamped = []
                voice_stamped = voicecommand
        role_description = get_role_description(
            A=cfg["actions"], 
            O=self.hri.scene.O, 
            S=self.hri.scene.get_scene_param_description(), 
            version=role_version
        )

        print()
        print("Role_description:")
        print(role_description)
        print()
        skillcommand = self.merge(gesture_stamped, voice_stamped, 
                                  command_constraints=cfg, 
                                  role_description=role_description, *args, **kwargs)
        
        print()
        print("SkillCommand is:")
        print(skillcommand)
        print()

        self.hri.play_skillcommand(skillcommand)
        # self.hri.play_skill(target_action, target_object)


    def merge(self, 
            gesture_stamped, 
            voice_stamped, 
            command_constraints: dict[str, list], # valid (zero-object/single-object/double-object) actions
            role_description: str, # for llm
            quantization, # REMOVED TEMPORARILY
            *args, **kwargs # llm params: temperature, top_p, repetition_penalty, max_generated_tokens
            ):
        """ Main merge function """
        print()
        print()
        print(f"{cc.H}Merge function:{cc.E}")
        print(f"{cc.H}[1]{cc.E} Voice stamped: ", voice_stamped, flush=True)
        print(f"{cc.H}[2]{cc.E} Gesture stamped: ", gesture_stamped, flush=True)
        print(f"{cc.H}[3]{cc.E} Role description incl. scene: ", role_description, flush=True)
        print_modalities(voice_stamped, gesture_stamped)

        gesture_stamped.extend(voice_stamped)
        sorted_sentence = sorted(gesture_stamped, key=lambda x: x[0])

        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        
        words = np.array(sorted_sentence)[:,1]

        # self.hri.sentence_processor = SentenceProcessor(model_name=self.hri.nlp_model_name, quantization=quantization)


        if self.interpret_format == "deterministic":
            final_sentence = " ".join(words[words!=None])
            # Run second round on object similarity
            # predicted = self.hri.sentence_processor.raw_predict(final_sentence, role_description, *args, **kwargs)
            self.hri.speak(f"Merged sentence is: {final_sentence}, starting reasoner")
            predicted = self.hri.sentence_processor.raw_predict(final_sentence, role_description=role_description, *args, **kwargs)



        elif self.interpret_format == "probabilistic":
            final_sentence = list(words[words!={}])
            for n,word in enumerate(final_sentence):
                if n%2==0:
                    final_sentence.insert(n, {" ": 1.0})
            final_sentence.insert(0, {" ": 1.0})
            final_sentence.insert(0, {" ": 1.0})
            final_sentence.append({" ": 1.0})
            final_sentence.append({" ": 1.0})
            self.hri.speak(f"Merged sentence is: {final_sentence}, starting reasoner")
            predicted = self.hri.sentence_processor.probabilistic_predict(final_sentence, role_description=role_description, *args, **kwargs)
        elif self.interpret_format == "alternatives":
            words = list(words[words!={}])
            final_sentence = f"\n"
            for word in words:
                most_prob_alternatives = [k for k, v in sorted(word.items(), key=lambda x: x[1], reverse=True)][:2]
                grading = ["likely", "possible", "unlikely", "least likely"]
                for n,option in enumerate(most_prob_alternatives):
                    if option not in [" ", "a", "the", ""]: # Discard these words
                        if option[-1] == ".":
                            option = option[:-1]
                        final_sentence += f'- "{option}" is {grading[n]}\n'

            self.hri.speak(f"Merged sentence is: {final_sentence}, starting reasoner")
            predicted = self.hri.sentence_processor.raw_predict(final_sentence, role_description=role_description, *args, **kwargs)
            
        print(f"{cc.W}LM says: {predicted} {cc.E}")
        return SkillCommand.from_predicted(predicted, command_constraints=command_constraints)

    def save_command(self, voice_command, gesture_command, subfolder="June2025"):
        print("save command")
        i = 0
        Path(f"{multi_modal_reasoning.path}/saved_inputs/{subfolder}/").mkdir(exist_ok=True)
        while Path(f"{multi_modal_reasoning.path}/saved_inputs/{subfolder}/save_{i}.npz").is_file():
            i+=1
        np.savez(f"{multi_modal_reasoning.path}/saved_inputs/{subfolder}/save_{i}", voice_command=np.array(voice_command), gesture_command=np.array(gesture_command))

    def save_log(self, true_sentence, skill_command, voice_stamped, gesture_stamped, scene, object_names, 
                 max_new_tokens, temperature, top_p, repetition_penalty, cfg, role_description, quantization):
        data = {
            "successful": SkillCommand(true_sentence) == skill_command,
            "true_sentence": true_sentence, 
            "predicted_sentence": skill_command.command,
            "predicted": skill_command.predicted,
            "model_name": self.name(),
            "voice_stamped": [list(v) for v in voice_stamped],
            "gesture_stamped": [list(v) for v in gesture_stamped],
            "scene": scene, 
            "object_names": object_names, 
            "max_new_tokens": max_new_tokens, 
            "temperature": temperature, 
            "top_p": top_p, 
            "repetition_penalty": repetition_penalty, 
            "cfg": cfg, 
            "role_description": role_description, 
            "quantization": quantization, # REMOVED TEMPORARILY
        }
    
        i = 0
        while Path(f"{multi_modal_reasoning.path}/saved_samples/save_{i}.json").is_file():
            i+=1
        with open(f"{multi_modal_reasoning.path}/saved_samples/save_{i}.json", "w") as file:
            json.dump(data, file, indent=4)


class ArgmaxMerger():
    def __init__(self,
                name_user: str = None,
                model_name: str = None,
                dry_run: bool = None,
                interpret_format: str = None,
                ):
        self.interpret_format = interpret_format

    def name(self):
        return "Argmax"

    def merge(self, gesture_stamped, voice_stamped, command_constraints, object_names, role_description=None, *args, **kwargs):
        gesture_stamped.extend(voice_stamped)
        sorted_sentence = sorted(gesture_stamped, key=lambda x: x[0])

        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        
        words = np.array(sorted_sentence)[:,1]
        
        if self.interpret_format == "deterministic":
            final_sentence = " ".join(words[words!=None])
            return self._merge(final_sentence, cfg, object_names)
        elif self.interpret_format == "probabilistic":
            raise Exception()
        
    def _merge2(self, final_sentence, cfg, object_names):
        a, o, p, o2, ap = "none", "none", "none", "none", "none"
        setting_obj_2 = False
        for word in final_sentence.split(" "):
            if word in cfg["actions"]:
                a = word
            if word in object_names:
                if setting_obj_2:
                    o2 = word
                else:
                    o = word
                    setting_obj_2 = True
            if word in cfg["prepositions"]:
                p = word
            if word in cfg["adjectives"]:
                ap = word
        predicted = f"property: {ap}, action: {a}, object: {o}, relationship: {p}, object2: {o2}"
        return SkillCommand.from_predicted(predicted, cfg=cfg)

    def _merge(self, final_sentence, cfg, object_names):
        a, o, p, o2, ap = "none", "none", "none", "none", "none"
        setting_obj_2 = False
        for word in final_sentence.split(" "):
            if a == "none" and word in cfg["actions"]:
                a = word
            if (o == "none" or o2 == "none") and word in object_names:
                if setting_obj_2:
                    o2 = word
                else:
                    o = word
                    setting_obj_2 = True
            if p == "none" and word in cfg["prepositions"]:
                p = word
            if ap == "none" and word in cfg["adjectives"]:
                ap = word
        predicted = f"property: {ap}, action: {a}, object: {o}, relationship: {p}, object2: {o2}"
        return SkillCommand.from_predicted(predicted, cfg=cfg)

def beam_search(input_lattice, beam_width=5):
    beam = [{'sentence': [], 'score': 1.0}]
    for word_options in input_lattice:
        new_beam = []
        for candidate in beam:
            for word, prob in word_options.items():
                new_sentence = candidate['sentence'] + [word]
                new_score = candidate['score'] * prob
                new_beam.append({'sentence': new_sentence, 'score': new_score})
        # Keep top-k candidates by score
        new_beam.sort(key=lambda x: -x['score'])
        beam = new_beam[:beam_width]
    return beam

class ZeroShotMerger():
    def __init__(self,
                name_user: str = None,
                model_name: str = None,
                dry_run: bool = None,
                interpret_format: str = None,
                ):
        # Initialize zero-shot classifier
        self.classifier = pipeline("zero-shot-classification", model="MoritzLaurer/DeBERTa-v3-base-mnli-fever-anli")

    def name(self):
        return "ZeroShot"

    def merge(self, gesture_stamped, voice_stamped, command_constraints, role_description=None, *args, **kwargs):
        # Classify action and primary object
        target_action = self.classify_entity(sentence, actions)
        target_object = self.classify_entity(sentence, objects)


    def classify_entity(self, sentence, candidates):
        result = self.classifier(sentence, candidates, multi_label=False)
        return result['labels'][0]

class BeamSearchMerger():
    def __init__(self,
                name_user: str = None,
                model_name: str = None,
                dry_run: bool = None,
                interpret_format: str = None,
                ):
        pass

    def name(self):
        return "BeamSearch"

    def merge(self, gesture_stamped, voice_stamped, command_constraints, role_description=None, *args, **kwargs):
        pass

def main():
    parser = argparse.ArgumentParser(description="My ROS 2 Node")
    parser.add_argument('--name_user', type=str, help='The user name', default="casper")
    parser.add_argument('--interpret_format', type=str, help='deterministic or probabilistic', default="deterministic")
    # parser.add_argument('--interpret_format', type=str, help='deterministic or probabilistic', default="probabilistic")
    # parser.add_argument('--name_model', type=str, help='', default="ArgmaxMerger")
    # parser.add_argument('--name_model', type=str, help='', default="SultanR/SmolTulu-1.7b-Instruct")
    # parser.add_argument('--name_model', type=str, help='', default="meta-llama/Llama-3.2-1B-Instruct") 
    parser.add_argument('--name_model', type=str, help='', default="LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct")
    # parser.add_argument('--name_model', type=str, help='', default="ibm-granite/granite-3.1-2b-instruct")
    # parser.add_argument('--name_model', type=str, help='', default="nvidia/Nemotron-Research-Reasoning-Qwen-1.5B")
    # parser.add_argument('--name_model', type=str, help='', default="deepseek-ai/DeepSeek-R1-Distill-Qwen-1.5B") # so stupid model
    # parser.add_argument('--name_model', type=str, help='', default="google/gemma-3-1b-it")

    parser.add_argument('--quantization', type=int, help='4,8,16,32 bits', default=4) # REMOVED TEMPORARILY
    parser.add_argument('--dry_run', type=bool, help='Dont play skills', default=False)
    parser.add_argument('--config_name', type=str, help='config_name', default="CONFIG_DEMO")
    parser.add_argument('--temperature', type=float, help='temperature', default=0.0)
    parser.add_argument('--top_p', type=float, help='top_p', default=1.0)
    parser.add_argument('--repetition_penalty', type=float, help='repetition penalty', default=1.1)
    parser.add_argument('--max_new_tokens', type=int, help='Max generated tokens', default=1000)
    parser.add_argument('--role_version', type=str, help='Role version spec', default="v4")
    args = parser.parse_args()

    rclpy.init()

    if args.name_model == "ArgmaxMerger":
        merger = ArgmaxMerger(interpret_format=args.interpret_format)
    else:
        merger = ReasoningMerger(name_user=args.name_user, model_name=args.name_model, interpret_format=args.interpret_format, dry_run=args.dry_run)

    print()
    merger.hri.speak(f"Ready! Hold a plus to record voice.")
    print()

    merger.spin(
        eval(args.config_name), 
        role_version=args.role_version, 
        temperature=args.temperature, 
        top_p=args.top_p, 
        repetition_penalty=args.repetition_penalty,
        max_new_tokens=args.max_new_tokens,
        quantization = args.quantization, # REMOVED TEMPORARILY
    )
    

if __name__ == "__main__":
    main()
