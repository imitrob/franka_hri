
import argparse, threading, time
from functools import partial
from naive_merger.HriCommand import HriCommand

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hri_msgs.msg import HRICommand as HRICommandMSG

class MMNode(Node):
    def __init__(self, args=None):
        super().__init__('merge_modalities_node')
        self.mm_publisher = self.create_publisher(HRICommandMSG, '/mm_solution', 5)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for mod in args.topics:
            self.create_subscription(
                HRICommandMSG,
                f'/modality/{mod}',
                partial(self.receive, mod),
                qos_profile=qos)
        
        self.received_messages = []

    def receive(self, mod, msg):
        print(f"received {mod}")
        self.received_messages.append([
            mod,
            time.perf_counter(),
            msg,
        ])

    def merge_modalities(self):
        gesture_command = None
        nlp_command = None
        for mod, time, msg in self.received_messages:
            if mod == "gestures":
                gesture_command = HriCommand.from_ros(msg) # choose the last received
            elif mod == "nlp":
                nlp_command = HriCommand.from_ros(msg) # choose the last received

        self.received_messages = [] # clean up received messages

        if gesture_command is None:
            return nlp_command.to_ros()
        if nlp_command is None:
            return gesture_command.to_ros()

        merged_command = gesture_command @ nlp_command
        return merged_command.to_ros()

    def execution_trigger(self) -> bool:
        """
        """
        input("Execute with enter!")

        if len(self.received_messages) <= 0:
            print("No messages received, returning")
            return False # no exec
        else:
            return True # exec

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", nargs="+", default=["gestures", "nlp"]) # TODO: Can listen on single topic name in the future
    args, _ = parser.parse_known_args()

    rclpy.init()
    mm_node = MMNode(args)

    def spinning_threadfn():
        while rclpy.ok():
            rclpy.spin_once(mm_node)
            time.sleep(0.01)

    spinning_thread = threading.Thread(target=spinning_threadfn, args=(), daemon=True)
    spinning_thread.start()

    while rclpy.ok():
        exe = mm_node.execution_trigger()
        if exe:
            mm_node.mm_publisher.publish(mm_node.merge_modalities())

if __name__ == '__main__':
    main()