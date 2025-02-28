
import numpy as np
from hri_manager.hri import HRI
from hri_manager.hci import HCI

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG
from std_msgs.msg import String
from hri_manager.HriCommand import HriCommand

import rclpy, time, json
from naive_merger.utils import cc
from llm_merger.skill_command import SkillCommand

import llm_merger
import argparse
from pathlib import Path
from llm_merger.utils import print_modalities
from transformers import pipeline

RECEIVE_CHECK_INTERVAL = 1.0 # [s]

class HRIMerger():
    def __init__(self,
                name_user: str,
                model_name: str,
                dry_run: bool = True,
                interpret_format: str = "deterministic",
                ):
        """_summary_

        Args:
            name_user (str): Loads "hri_manager/links/<user>_links.yaml" where the user config is stored
            model_name (str): _description_
            dry_run (bool, optional): _description_. Defaults to True.
            interpret_format (str): "deterministic" or "probabilistic" interpret format
        """   
        self.interpret_format = interpret_format
        if dry_run: # No robot
            self.hri = HCI(
                name_user = name_user,
                nlp_model_name = model_name,
                tts_enabled = True,
                stt_type = self.interpret_format
            )
        else: # With robot control
            self.hri = HRI(
                name_user = name_user,
                tts_enabled = True,
                dry_run = False,
                nlp_model_name = model_name,
                stt_type = self.interpret_format
            )
        self.model_name = model_name
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.hri.create_subscription(HRICommandMSG, '/modality/gestures', self.gesture_hricommand_callback, qos_profile=qos)
        self.hri.create_subscription(String, '/recorded_file', self.receive_voice_record, qos_profile=qos)
        self.record_queue = []
        self.gestures_queue = []

    def name(self):
        return self.model_name.split("/")[-1]

    def spin(self, role_description):
        while rclpy.ok():
            time.sleep(RECEIVE_CHECK_INTERVAL)
            if len(self.record_queue) > 0:
                record_stamped_file_name = self.record_queue.pop()
                voicecommand = self.hri.stt.stamped_transcribe(record_stamped_file_name)

                if len(self.gestures_queue) > 1:
                    self.hri.speak(f"There are {len(self.gestures_queue)} of gesturings, the last one is used, others are discarded")
                if len(self.gestures_queue) > 0:
                    hricommand = self.gestures_queue.pop()
                    self.extract_merge_and_play(voicecommand, hricommand, role_description)
                else:
                    self.extract_merge_and_play(voicecommand, [], role_description)

                self.record_queue = []
                self.gestures_queue = []


    def receive_voice_record(self, msg):
        msg_dict = json.loads(str(msg.data))
        print(f"Received: {cc.W}Recording{cc.E}", flush=True)
        self.record_queue.append(msg_dict)

    def gesture_hricommand_callback(self, msg):
        print(f"Received: {cc.W}Gestures{cc.E}", flush=True)
        hricommand = HriCommand.from_ros(msg)
        self.gestures_queue.append(hricommand)

    def extract_merge_and_play(self, hricommand, voicecommand, *args, **kwargs):
        # extract
        if self.interpret_format == "deterministic":
            """ gesture_stamped: [:,0] - timestamps, [:,1] - words 
            voice_stamped:   [:,0] - timestamps, [:,1] - words
            """      
            gesture_stamped = hricommand.get_target_timestamped_list()
            voice_stamped = voicecommand
            
        elif self.interpret_format == "probabilistic":
            """ both gesture_stamped and voice_stamped in format:
            [ # time,  word  : probs, ...
                [0.0, {"pick": 1.0, "kick": 0.2}],
                [0.1, {"up": 0.96, "lap": 0.1}],
            ]
            """
            gesture_stamped = hricommand.get_target_timestamped_probabilistic()
            voice_stamped = voicecommand

        # merge
        target_action, target_object = self.merge(gesture_stamped, voice_stamped, *args, **kwargs)
        
        # play
        self.hri.play_skill(target_action, target_object)


    def merge(self, gesture_stamped, voice_stamped, CONFIG, *args, **kwargs):
        print("Voice stamped: ", voice_stamped, flush=True)
        print("Gesture stamped: ", gesture_stamped, flush=True)
        print_modalities(voice_stamped, gesture_stamped)

        gesture_stamped.extend(voice_stamped)
        sorted_sentence = sorted(gesture_stamped, key=lambda x: x[0])

        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        
        words = np.array(sorted_sentence)[:,1]
        
        if self.interpret_format == "deterministic":
            final_sentence = " ".join(words[words!=None])
            self.hri.speak(f"Merged sentence is: {final_sentence}")
            predicted = self.hri.sentence_processor.raw_predict(final_sentence, *args, **kwargs)
            # Run second round on object similarity
            # predicted = self.hri.sentence_processor.raw_predict(final_sentence, role_description, *args, **kwargs)
        elif self.interpret_format == "probabilistic":
            final_sentence = list(words[words!={}])
            for n,word in enumerate(final_sentence):
                if n%2==0:
                    final_sentence.insert(n, {" ": 1.0})
            self.hri.speak(f"Merged sentence is: {final_sentence}")
            predicted = self.hri.sentence_processor.probabilistic_predict(final_sentence, *args, **kwargs)
        print(f"{cc.W}LM says: {predicted} {cc.E}")
        return SkillCommand.from_predicted(predicted, CONFIG=CONFIG)

        # target_action, target_object = self.hri.map_instruction_words(predicted) # tunnel down to commands


    def save(self, voice_stamped, gesture_stamped):
        i = 0
        while Path(f"{llm_merger.path}/saved_inputs/save_{i}.npz").is_file():
            i+=1
        np.savez(f"{llm_merger.path}/saved_inputs/save_{i}", voice_stamped=np.array(voice_stamped), gesture_stamped=np.array(gesture_stamped))

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

    def merge(self, gesture_stamped, voice_stamped, CONFIG, object_names, *args, **kwargs):
        gesture_stamped.extend(voice_stamped)
        sorted_sentence = sorted(gesture_stamped, key=lambda x: x[0])

        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        
        words = np.array(sorted_sentence)[:,1]
        
        if self.interpret_format == "deterministic":
            final_sentence = " ".join(words[words!=None])
            return self._merge(final_sentence, CONFIG, object_names)
        elif self.interpret_format == "probabilistic":
            final_sentence = list(words[words!={}])
            for n,word in enumerate(final_sentence):
                if n%2==0:
                    final_sentence.insert(n, {" ": 1.0})
        else: raise Exception("self.interpret_format", self.interpret_format)
        
    def _merge(self, final_sentence, CONFIG, object_names):
        a, o, p, o2, ap = "none", "none", "none", "none", "none"
        for word in final_sentence.split(" "):
            if word in CONFIG["actions"]:
                a = word
            if word in object_names:
                o = word
                o2 = word
            if word in CONFIG["prepositions"]:
                p = word
            if word in CONFIG["adjectives"]:
                ap = word
        predicted = f"property: {ap}, action: {a}, object: {o}, relationship: {p}, object2: {o2}"
        return SkillCommand.from_predicted(predicted, CONFIG=CONFIG)

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

    def merge(self, gesture_stamped, voice_stamped, CONFIG, object_names, *args, **kwargs):
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

    def merge(self, gesture_stamped, voice_stamped, CONFIG, object_names, *args, **kwargs):
        pass

def main():
    parser = argparse.ArgumentParser(description="My ROS 2 Node")
    parser.add_argument('--name_user', type=str, help='The user name', default="casper")
    parser.add_argument('--interpret_format', type=str, help='deterministic or probabilistic', default="deterministic")
    parser.add_argument('--name_model', type=str, help='The user name', default="SultanR/SmolTulu-1.7b-Reinforced")
    parser.add_argument('--dry_run', type=bool, help='Dont play skills', default=True)
    parser.add_argument('--role_version', type=str, help='Role description', default="v1")
    parser.add_argument('--temperature', type=float, help='temperature, 0.0 is deterministic ', default=0.0)
    parser.add_argument('--top_p', type=float, help='top p', default=1.0)
    parser.add_argument('--repetition_penalty', type=float, help='', default=1.1)
    parser.add_argument('--max_new_tokens', type=int, help='max words output', default=50)
    args = parser.parse_args()

    rclpy.init()

    merger = HRIMerger(name_user=args.name_user, model_name=args.name_model, dry_run=args.dry_run, interpret_format=args.interpret_format)
    print(merger.hri.print_user_preferences())
    merger.spin(args.role_version)

if __name__ == "__main__":
    main()
