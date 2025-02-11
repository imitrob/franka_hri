
import numpy as np

from naive_merger.modality_merger import merge_probabilities
from naive_merger.HriCommand import HriCommand
from naive_merger.probs_vector import EntropyProbsVector, NaiveProbsVector, ProbsVector

def test_core_merging_1():
    ls = {
        "action": np.array([0.5, 0.3, 0.2]), 
        "object": np.array([0.6, 0.1, 0.3]), 
    }
    gs = {
        "action": np.array([0.5, 0.3, 0.2]), 
        "object": np.array([0.6, 0.1, 0.3]), 
    }

    arity_names = ["action", "object"]

    for magic_function in ["mul", "add"]:
        for thresholding in ["no thresholding", "fixed", "entropy"]:
            p = merge_probabilities(ls, gs, thresholding, magic_function, arity_names)
            print(p)

def test_core_merging_2():
    pv = ProbsVector(p=[0.1,0.2,0.3],template_names=['mark', 'park', 'quark'])
    print(f"{pv}")
    pv = EntropyProbsVector(p=[0.1,0.2,0.3],template_names=['mark', 'park', 'quark'])
    print(f"{pv}")
    pv = NaiveProbsVector(p=[0.1,0.2,0.3],template_names=['mark', 'park', 'quark'])
    print(f"{pv}")

def test_core_merging_3():
    hri_command_nlp = HriCommand.from_dict(
        arity_names = ["action", "object"],
        data_dict = {
            "action_probs": np.array([0.5, 0.3, 0.2]), 
            "object_probs": np.array([0.6, 0.1, 0.3]), 
            "action_names": np.array(["pick", "pour", "grab"]), 
            "object_names": np.array(["cube", "cup", "can"]), 
        }, 
        thresholding = "entropy",
    )
    hri_command_gs = HriCommand.from_dict(
        arity_names = ["action", "object"],
        data_dict = {
            "action_probs": np.array([0.5, 0.3, 0.2]), 
            "object_probs": np.array([0.6, 0.1, 0.3]), 
            "action_names": np.array(["pick", "pour", "grab"]), 
            "object_names": np.array(["cube", "cup", "can"]), 
        }, 
        thresholding = "entropy",
    )

    merged_hri_command = hri_command_nlp @ hri_command_gs

def test_hricommand_conversion():
    hri_command = HriCommand.from_dict(
        arity_names = ["action", "object"],
        data_dict = {
            "action_probs": np.array([0.5, 0.3, 0.2]), 
            "object_probs": np.array([0.6, 0.1, 0.3]), 
            "action_names": np.array(["pick", "pour", "grab"]), 
            "object_names": np.array(["cube", "cup", "can"]), 
        }, 
        thresholding="entropy",
    )

    hri_command2 = HriCommand.from_dict(
        arity_names = ["action", "object"],
        data_dict = hri_command.to_dict(),
        thresholding = "entropy",
    )

    assert hri_command == hri_command2

# def test_hricommand_conversion_ros():
#     HRICommandMSG(data=str(json.dump(self.to_dict())))

if __name__ == "__main__":
    test_core_merging_1()
    test_core_merging_2()
    test_core_merging_3()
    test_hricommand_conversion()