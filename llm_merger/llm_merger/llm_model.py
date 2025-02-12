
import numpy as np
from hri_manager.hri import HRI
from hri_manager.hci import HCI

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG
from std_msgs.msg import String
from hri_manager.HriCommand import HriCommand

import rclpy, time
from naive_merger.utils import cc

import llm_merger
from pathlib import Path

RECEIVE_CHECK_INTERVAL = 1.0 # [s]

class HRIMerger():
    def __init__(self,
                name_user: str,
                model_name: str,
                dry_run: bool = True,
                ):
        if dry_run: # No robot
            self.hri = HCI(
                name_user = name_user,
                nlp_model_name = model_name,
                tts_enabled = True,
            )
        else: # With robot control
            self.hri = HRI(
                name_user = name_user,
                tts_enabled = True,
                dry_run = False,
                nlp_model_name = model_name,
            )
        
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.hri.create_subscription(HRICommandMSG, '/modality/gestures', self.gesture_hricommand_callback, qos_profile=qos)
        self.hri.create_subscription(String, '/recorded_file', self.receive_voice_record, qos_profile=qos)
        self.record_queue = []
        self.gestures_queue = []

    def spin(self):
        while rclpy.ok():
            time.sleep(RECEIVE_CHECK_INTERVAL)
            if len(self.record_queue) > 0:
                record_file_name = self.record_queue.pop()
                voice_stamped = self.hri.stt.forward_timestamped(record_file_name)
                if len(self.gestures_queue) > 0:
                    gesture_stamped = self.gestures_queue.pop()
                    self.forward(voice_stamped, gesture_stamped, ROLE_DESCRIPTION)
                else:
                    self.forward(voice_stamped, None, ROLE_DESCRIPTION)


    def receive_voice_record(self, msg):
        print(f"Received: {cc.H}Recording{cc.E}", flush=True)
        self.record_queue.append(msg.data)

    def gesture_hricommand_callback(self, msg):
        print(f"Received: {cc.H}Gestures{cc.E}", flush=True)
        hricommand = HriCommand.from_ros(msg)
        self.gestures_queue.append(hricommand.get_target_timestamped_list())

    def sort_merge(self, gesture_stamped, voice_stamped):
        """ gesture_stamped: [:,0] - timestamps, [:,1] - words 
            voice_stamped:   [:,0] - timestamps, [:,1] - words
        """      
        gesture_stamped.extend(voice_stamped)
        return sorted(gesture_stamped, key=lambda x: x[0])

    def forward(self, 
                voice_stamped,
                gesture_stamped, 
                role_description,
                max_new_tokens = 50,
                temperature = 0.0,
                top_p = 1.0,
                repetition_penalty = 1.1,
                ):
        self.save(voice_stamped, gesture_stamped)
        sorted_sentence = self.sort_merge(gesture_stamped, voice_stamped)
        print(f"{cc.H}Sorted stamped sentence{cc.E}: {sorted_sentence}")
        words = np.array(sorted_sentence)[:,1]
        final_sentence = " ".join(words[words!=None])
        print(f"{cc.H}Merged sentence: {final_sentence}{cc.E}")
        predicted = self.hri.sentence_processor.raw_predict(
            final_sentence,
            role_description,
            max_new_tokens = max_new_tokens, 
            temperature = temperature, 
            top_p = top_p,
            repetition_penalty = repetition_penalty,    
        )
        print(f"{cc.W}Predicted: {predicted}{cc.E}", flush=True)

        for k in predicted.keys():
            if isinstance(predicted[k], np.ndarray):
                predicted[k] = list(predicted[k])

        print("Sentence processing output: ", predicted, flush=True)

        target_action, target_object = map_instruction_words(output, self.hri.user)
        self.hri.play_skill(target_action, target_object)

    def save(self, voice_stamped, gesture_stamped):
        i = 0
        while Path(f"{llm_merger.path}/saved_inputs/save_{i}").is_file():
            i+=1
        np.savez(f"{llm_merger.path}/saved_inputs/save_{i}", voice_stamped=np.array(voice_stamped), gesture_stamped=np.array(gesture_stamped))

def main():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    merger.spin()

if __name__ == "__main__":
    main()
