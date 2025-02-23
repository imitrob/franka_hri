
import numpy as np
from hri_manager.hri import HRI
from hri_manager.hci import HCI

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG
from std_msgs.msg import String
from hri_manager.HriCommand import HriCommand

import rclpy, time, json
from naive_merger.utils import cc
from llm_merger.role_setup import get_role_description

import llm_merger
import argparse
from pathlib import Path
from llm_merger.utils import print_modalities

RECEIVE_CHECK_INTERVAL = 1.0 # [s]

class HRIMerger():
    def __init__(self,
                name_user: str,
                model_name: str,
                role_version: str,
                dry_run: bool = True,
                merge_approach: str = "deterministic",
                ):
        """_summary_

        Args:
            name_user (str): Loads "hri_manager/links/<user>_links.yaml" where the user config is stored
            model_name (str): _description_
            dry_run (bool, optional): _description_. Defaults to True.
        """   
        self.merge_approach = merge_approach
        if dry_run: # No robot
            self.hri = HCI(
                name_user = name_user,
                nlp_model_name = model_name,
                tts_enabled = True,
                stt_type = self.merge_approach
            )
        else: # With robot control
            self.hri = HRI(
                name_user = name_user,
                tts_enabled = True,
                dry_run = False,
                nlp_model_name = model_name,
                stt_type = self.merge_approach
            )
        self.role_description = get_role_description(self.hri.A, O=self.hri.O, version=role_version)     


        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.hri.create_subscription(HRICommandMSG, '/modality/gestures', self.gesture_hricommand_callback, qos_profile=qos)
        self.hri.create_subscription(String, '/recorded_file', self.receive_voice_record, qos_profile=qos)
        self.record_queue = []
        self.gestures_queue = []

    def spin(self):
        while rclpy.ok():
            time.sleep(RECEIVE_CHECK_INTERVAL)
            if len(self.record_queue) > 0:
                record_stamped_file_name = self.record_queue.pop()
                voicecommand = self.hri.stt.stamped_transcribe(record_stamped_file_name)

                if len(self.gestures_queue) > 1:
                    self.hri.speak(f"There are {len(self.gestures_queue)} of gesturings, the last one is used, others are discarded")
                if len(self.gestures_queue) > 0:
                    hricommand = self.gestures_queue.pop()
                    self.merge(voicecommand, hricommand, self.role_description)
                else:
                    self.merge(voicecommand, [], self.role_description)

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

    
    def merge(self, hricommand, voicecommand, *args, **kwargs):
        if self.merge_approach == "deterministic":
            gesture_stamped = hricommand.get_target_timestamped_list()
            voice_stamped = voicecommand
            predicted = self._merge(gesture_stamped, voice_stamped, *args, **kwargs)
        elif self.merge_approach == "probabilistic":
            gesture_stamped = hricommand.get_target_timestamped_probabilistic()
            voice_stamped = voicecommand
            predicted = self.prob_merge(gesture_stamped, voice_stamped, *args, **kwargs)

        target_action, target_object = self.hri.map_instruction_words(predicted) # tunnel down to commands
        self.hri.play_skill(target_action, target_object)

    def _merge(self, gesture_stamped, voice_stamped, *args, **kwargs):
        """ gesture_stamped: [:,0] - timestamps, [:,1] - words 
            voice_stamped:   [:,0] - timestamps, [:,1] - words
        """      
        print("Voice stamped: ", voice_stamped, flush=True)
        print("Gesture stamped: ", gesture_stamped, flush=True)
        print_modalities(voice_stamped, gesture_stamped)
        self.save(voice_stamped, gesture_stamped)
        
        gesture_stamped.extend(voice_stamped)
        sorted_sentence = sorted(gesture_stamped, key=lambda x: x[0])

        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        
        words = np.array(sorted_sentence)[:,1]
        final_sentence = " ".join(words[words!=None])
        
        self.hri.speak(f"Merged sentence is: {final_sentence}")
        
        predicted = self.hri.sentence_processor.predict(final_sentence, *args, **kwargs)
        print(f"Predicted sentence: {predicted}", flush=True)
        return predicted
    
    def prob_merge(self, gesture_stamped, voice_stamped, *args, **kwargs):
        """ both gesture_stamped and voice_stamped in format:
        [ # time,  word  : probs, ...
            [0.0, {"pick": 1.0, "kick": 0.2}],
            [0.1, {"up": 0.96, "lap": 0.1}],
        ]
        """
        print("Voice stamped: ", voice_stamped, flush=True)
        print("Gesture stamped: ", gesture_stamped, flush=True)
        print_modalities(voice_stamped, gesture_stamped)
        self.save(voice_stamped, gesture_stamped)
        
        gesture_stamped.extend(voice_stamped)
        sorted_sentence = sorted(gesture_stamped, key=lambda x: x[0])

        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        
        words = np.array(sorted_sentence)[:,1]
        final_sentence = str(words[words!={}])
        
        self.hri.speak(f"Merged sentence is: {final_sentence}")
        
        predicted = self.hri.sentence_processor.predict_with_probs(final_sentence, *args, **kwargs)
        print(f"Predicted sentence: {predicted}", flush=True)
        return predicted

    def save(self, voice_stamped, gesture_stamped):
        i = 0
        while Path(f"{llm_merger.path}/saved_inputs/save_{i}.npz").is_file():
            i+=1
        np.savez(f"{llm_merger.path}/saved_inputs/save_{i}", voice_stamped=np.array(voice_stamped), gesture_stamped=np.array(gesture_stamped))





def main():
    parser = argparse.ArgumentParser(description="My ROS 2 Node")
    parser.add_argument('--name_user', type=str, help='The user name', default="casper")
    parser.add_argument('--merge_approach', type=str, help='deterministic or probabilistic', default="deterministic")
    parser.add_argument('--name_model', type=str, help='The user name', default="SultanR/SmolTulu-1.7b-Reinforced")
    parser.add_argument('--dry_run', type=bool, help='Dont play skills', default=True)
    parser.add_argument('--role_version', type=str, help='Role description', default="v1")
    parser.add_argument('--temperature', type=float, help='temperature, 0.0 is deterministic ', default=0.0)
    parser.add_argument('--top_p', type=float, help='top p', default=1.0)
    parser.add_argument('--repetition_penalty', type=float, help='', default=1.1)
    parser.add_argument('--max_new_tokens', type=int, help='max words output', default=50)
    args = parser.parse_args()

    rclpy.init()

    merger = HRIMerger(name_user=args.name_user, model_name=args.name_model, role_version=args.role_version, dry_run=args.dry_run, merge_approach=args.merge_approach)
    print(merger.hri.print_user_preferences())
    merger.spin()

if __name__ == "__main__":
    main()
