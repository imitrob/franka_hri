
#!/usr/bin/env python
from naive_merger.HriCommand import HriCommand

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG

from hri_manager.hri import HRI



class ActionExecutor():
    def __init__(self, listener_topics=["gestures", "nl"], dry_run:bool = False):
        self.hri = HRI(tts_enabled=False, dry_run=dry_run)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for mod in listener_topics:
            self.hri.create_subscription(
                HRICommandMSG,
                f'/modality/{mod}',
                self.play_skill_callback,
                qos_profile=qos)
        
    def play_skill_callback(self, msg):
        hricommand = HriCommand.from_ros(msg)
        self.hri.play_skill(hricommand.target_action, object_template_name=hricommand.target_action)

def main(dry_run: bool):
    rclpy.init()
    node = ActionExecutor(dry_run=dry_run)
    print("Listening for voice commands", flush=True)
    node.hri.listen_for_voice_commands()
    try:
        while True:
            input("")
    except KeyboardInterrupt:
        pass    

def action_executor():
    main(dry_run=False)

def action_executor_dry_run():
    print("DRY RUN !", flush=True)
    main(dry_run=True)

if __name__ == "__main__":
    action_executor()
