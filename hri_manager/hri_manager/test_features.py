#!/usr/bin/env python
import rclpy, time
from hri_manager.hri import HRI, Link

def main():
    rclpy.init()
    hri = HRI()

    # user_name = "Melichar"
    action_template = "touch_sponge"
    object_template = "sponge"
    print("0", time.time())

    hri.tts.speak("Testing the text to speech feature!")

    # hri.tts.speak("Testing homing!")
    # hri.lfd.home()
    # hri.tts.speak("Testing touch sponge action, be cautious!")
    # hri.lfd.play_skill(action_template, object_template)
    # hri.tts.speak("Testing recording!")
    print("1", time.time())
    action_word = hri.listen_user()
    hri.tts.speak(f"Did you say the word: {action_word}")
    time.sleep(2.0)
    hri.tts.speak(f"Great! Test done!")

if __name__ == "__main__":
    main()