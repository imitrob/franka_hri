
import rclpy
from rclpy.node import Node
from hri_msgs.msg import HRICommand as HriCommandMSG
from naive_merger.HriCommand import HriCommand
import numpy as np


def test_node_merging_1():
    mm = Node("test_node_merging")
    gestures_pub = mm.create_publisher(HriCommandMSG, "/modality/gestures", 5)
    nlp_pub = mm.create_publisher(HriCommandMSG, "/modality/gestures", 5)

    arity_names = ["action", "object"]
    ls = {
        "action_names": ["pick", "pour", "put"],
        "action_probs": np.array([0.5, 0.3, 0.2]), 
        "object_names": ["cup_od_1", "cup_od_2", "drawer_od_1"],
        "object_probs": np.array([0.6, 0.1, 0.3]), 
    }
    gs = {
        "action_names": ["pick", "pour", "put"],
        "action_probs": np.array([0.5, 0.3, 0.2]), 
        "object_names": ["cup_od_1", "cup_od_2", "drawer_od_1"],
        "object_probs": np.array([0.6, 0.1, 0.3]), 
    }
    gestures_pub.publish(HriCommand.from_dict(arity_names, ls, thresholding="entropy").to_ros())


def main():
    rclpy.init()
    test_node_merging_1()


if __name__ == "__main__":
    main()