#!/usr/bin/env python
import rclpy, argparse
from hri_manager.hri import HRI
from hri_manager.hri_link import Link

def main():
    parser = argparse.ArgumentParser(description="My ROS 2 Node")
    parser.add_argument('--name_user', type=str, help='The user name')
    parser.add_argument('--name_skill', type=str, help='Skill name')
    parser.add_argument('--name_object', type=str, help='Object name', default="")
    parser.add_argument('--dry_run', type=bool, help='Dont play skill at start', default=True)
    parser.add_argument('--tts_enabled', type=bool, help='', default=True)
    args = parser.parse_args()

    rclpy.init()
    hri = HRI(name_user=args.name_user, tts_enabled=args.tts_enabled, dry_run=args.dry_run)
    hri.speak("Showing the action!")
    hri.home()

    if args.name_object == "": # No name_object given, Object has same name as action
        args.name_object = args.name_skill

    hri.play_skill(args.name_skill, args.name_object)
    
    action_words = []
    while True:
        hri.speak("Now say, what would you told the robot to do this action! One word please. Recording. 3, 2, 1, Go!")
        action_word = hri.listen_user()
        print(f"Action word: {action_word}", flush=True)
        
        hri.speak(f"You pick action word: {action_word}")
        char = input("Is it good? [Y/n/a] (a = adds the word to the action words, but repeats)")
        if char == "a": # Repeats, but adds the word to list of action gestures
            action_words.append(action_word)
            continue
        elif char == 'n': # Tries again
            continue
        else: # Breaks
            action_words.append(action_word)
            break
    
    object_words = []
    while True:
        hri.speak("Now say, how would you call the scene object that the robot interacted with! One word please. Recording. 3, 2, 1, Go!")
        object_word = hri.listen_user()
        print(f"Action word: {object_word}", flush=True)
        
        hri.speak(f"You pick action word: {object_word}")
        char = input("Is it good? [Y/n/a] (a = adds the word to the action words, but repeats)")
        if char == "a": # Repeats, but adds the word to list of action gestures
            object_words.append(object_word)
            continue
        elif char == 'n': # Tries again
            continue
        else: # Breaks
            object_words.append(object_word)
            break

    action_gestures = []
    while True:
        hri.speak("Now show the hand gesture!")
        action_gesture = hri.gestures.wait_gestures()
        print(f"Action gesture: {action_gesture}", flush=True)
        hri.speak(f"You picked the action gesture {action_gesture}.")
        char = input("Is it good? [Y/n/a] (a = adds the word to the action words, but repeats)")
        if char == "a": # Repeats, but adds the word to list of action gestures
            action_gestures.append(action_gesture)
            continue
        elif char == 'n': # Tries again
            continue
        else: # Breaks
            action_gestures.append(action_gesture)
            break
    link = Link(args.name_user, args.name_skill, args.name_skill, action_words, object_words, action_gestures)
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