#!/usr/bin/env python
import rclpy, argparse
from hri_manager.hri import HRI, Link

def main():
    rclpy.init()
    hri = HRI(tts_enabled=False)

    parser = argparse.ArgumentParser(description="My ROS 2 Node")
    parser.add_argument('--name_user', type=str, help='The user name')
    parser.add_argument('--name_skill', type=str, help='Skill name')
    args = parser.parse_args()

    hri.speak("Showing the action!")
    hri.home()
    # hri.play_skill(name_skill, name_skill)
    
    while True:
        hri.speak("Now say, what would you told the robot to do this action! One word please. Recording. 3, 2, 1, Go!")
        action_word = hri.listen_user()
        print(f"Action word: {action_word}", flush=True)
        hri.speak(f"You pick action word: {action_word}")
        if input("Repeat [y/N]") != "y": break
    
    while True:
        hri.speak("Now show the hand gesture!")
        action_gesture = hri.gestures.wait_gestures()
        print(f"Action gesture: {action_gesture}", flush=True)
        hri.speak(f"You picked the action gesture {action_gesture}.")
        if input("Repeat [y/N]") != "y": break
    
    link = Link(args.name_user, args.name_skill, args.name_skill, action_word, action_gesture)
    link.check_valid(hri)
    link.save()
    hri.speak("Link is saved!")
    hri.speak("Program succesfully exits!")

if __name__ == "__main__":
    # link = Link("casper", "push", "box_template", "push", ["grab", "swipe forward"])
    # link.save()
    # link = Link("casper", "pull", "asd_template", "pull", ["grab", None])
    # link.save()
    # link = Link("casper", "pick", "vsa_template", "pick", [None, None])
    # link.save()
    main()