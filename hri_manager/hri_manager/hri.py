#!/usr/bin/env python
from skills_manager.lfd import LfD
from hri_manager.feedback_for_hri import Feedback_for_HRI
from hri_manager.hci import HCI
from skills_manager.skill import Skill

from lfd_msgs.srv import SetTemplate
from std_srvs.srv import Trigger

from multi_modal_reasoning.skill_command import SkillCommand
from naive_merger.utils import cc

class HRI(HCI, Feedback_for_HRI, LfD):
    def __init__(self,
                name_user: str,
                tts_enabled: bool = True,
                dry_run: bool = False,
                nlp_model_name: str = None,
                stt_type: str = "deterministic",
                stt_enabled: bool = True,
                ):
        self.tts_enabled = tts_enabled
        self.dry_run = dry_run
        self.user = name_user
        self.nlp_model_name = nlp_model_name
        self.stt_type = stt_type
        self.stt_enabled = stt_enabled
        super(HRI, self).__init__()
        self.start() # Starts robotic controller

    def play_skillcommand(self, skillcommand: SkillCommand):
        
        print(f"{cc.W}Playing skill command: {skillcommand}{cc.E}")

        # TODO: Check skill validity        
        # if not skillcommand.is_valid(): 
        #     self.speak("Skill Command is Not valid, returning!")
        #     return
        
        if skillcommand.target_action in skillcommand.command_constraints["zero_object_actions"]:
            self.play_skill(name_skill=skillcommand.target_action, simplify=False)

        if skillcommand.target_action in skillcommand.command_constraints["single_object_actions"]:
            self.play_skill(name_skill=skillcommand.target_action+"_"+skillcommand.target_object, name_template=skillcommand.target_object, simplify=False)

        if skillcommand.target_action in skillcommand.command_constraints["double_object_actions"]:
            self.play_skill(name_skill=skillcommand.target_action+"1_"+skillcommand.target_object, name_template=skillcommand.target_object, simplify=False)
            self.play_skill(name_skill=skillcommand.target_action+"2_"+skillcommand.target_storage, name_template=skillcommand.target_storage, simplify=False)


    def play_skill(self, name_skill: str, name_template: str= "", skill_parameter: float = None, simplify=True):
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
        
        if not self.set_localizer_client.wait_for_service(timeout_sec=5.0):
            self.speak(f"Localization service is unavailable! Returning")
            return
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
        except KeyboardInterrupt: # not working, because there is another thread (Feedback) that shutsdown then KeyboardInterrupt
            pass

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

def main():
    import rclpy
    rclpy.init()
    hri = HRI()

if __name__ == "__main__":
    main()