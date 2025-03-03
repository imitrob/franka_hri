
from llm_merger.llm_model import HRIMerger, ArgmaxMerger, ZeroShotMerger, BeamSearchMerger

import rclpy
import llm_merger
import numpy as np
from pathlib import Path

from llm_merger.role_setup import get_role_description
from llm_merger.generate_dataset import EnhancedDatasetGenerator, CONFIG, CONFIG3

from llm_merger.skill_command import SkillCommand
import json, copy
from llm_merger.generate_dataset import Sample
from naive_merger.utils import cc
import time
def test_skill_commands():
    # these should be valid
    assert SkillCommand("stop").is_valid()
    assert SkillCommand("pick cup1").is_valid()
    assert SkillCommand("pour cup1 to cup2").is_valid()

    # these should not be valid
    assert not SkillCommand("stop cup1").is_valid()
    assert not SkillCommand("stop cup1 to cup2").is_valid()
    assert not SkillCommand("pick cup1 cup2").is_valid()
    assert not SkillCommand("pick cup1 to cup2").is_valid()
    assert not SkillCommand("pour cup1").is_valid()
    assert not SkillCommand("pour").is_valid()

    assert SkillCommand("pour").target_action == "pour"
    assert SkillCommand("pour cup1").target_action == "pour"
    assert SkillCommand("pour cup1").target_object == "cup1"
    assert SkillCommand("quickly pour cup1").target_action == "pour"
    assert SkillCommand("quickly pour cup1").target_object == "cup1"
    assert SkillCommand("quickly pour cup1").action_parameter == "quickly"
    assert SkillCommand("quickly pour cup1 to cup2").target_object2 == "cup2"
    assert SkillCommand("quickly pour cup1 to cup2").object_preposition == "to"

""" When I install this package, I always try to run this function """
def test_just_to_see_if_works():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, "Pick"],
            [0.1, "Green"],
            [0.4, "Cup"],
        ],
        gesture_stamped = [
            [0.4, "cup1"],
        ],
        role_description=get_role_description(A=["Pick", "Push", "Pour"], O=["cup1", "Drawer", "Bowl"]),
        max_new_tokens = 1000,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    )
    assert skill_command == SkillCommand("pick cup1"), f"{skill_command} != 'pick cup1'"
    merger.hri.delete()
    rclpy.shutdown()


def test_just_probabilistic():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", interpret_format="probabilistic")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, {"pick": 1.0}],
            [0.2, {"up": 1.0}],
            [0.4, {"this": 1.0}],
        ],
        gesture_stamped = [
            [0.6, {"box": 1.0}],
        ], 
        role_description=get_role_description(
            A=["pick", "push", "pour"], 
            O=["cup1", "box", "bowl1"]
        ),
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_just_alternatives():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="ibm-granite/granite-3.1-2b-instruct", interpret_format="alternatives")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, {"kick": 1.0}],
            [0.2, {"up": 1.0}],
            [0.4, {"this": 1.0}],
        ],
        gesture_stamped = [
            [0.6, {"cup1": 0.5}],
        ], 
        role_description=get_role_description(
            A=CONFIG3["actions"], 
            O=["cup1", "container1", "bowl1"],
            S=S, 
        ), CONFIG=CONFIG3
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_just_alternatives2():
    rclpy.init()

    model_name="SultanR/SmolTulu-1.7b-Reinforced"
    model_name="LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct"
    model_name="ibm-granite/granite-3.1-2b-instruct"
    merger = HRIMerger(name_user="casper", model_name=model_name, interpret_format="alternatives")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, {"kick": 1.0, "pick": 0.8}],
            [0.2, {"up": 0.8, "pup": 0.6}],
            [0.4, {"this": 0.9, "is": 0.2}],
        ],
        gesture_stamped = [
            [0.6, {"box": 1.0, "fox": 0.9}],
        ], 
        role_description=get_role_description(
            A=CONFIG3["actions"], 
            O=["cup1", "box", "plate1"]
        ), CONFIG=CONFIG3
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_just_alternatives3():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", interpret_format="alternatives")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, {"pick": 1.0}],
            [0.2, {"up": 1.0}],
            [0.4, {"this": 1.0}],
        ],
        gesture_stamped = [
            [0.6, {"box": 1.0}],
        ], 
        role_description=get_role_description(
            A=CONFIG3["actions"], 
            O=["cup1", "box", "plate1"]
        ), CONFIG=CONFIG3
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_lm():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    print("(1/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "this"] ],
                                 gesture_stamped = [ [0.4, "container1"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick container1"), f"{skill_command} != 'pick container1'"
    print("(2/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Push"], [0.4, "this"] ],
                                 gesture_stamped = [ [0.4, "bowl1"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("push bowl1"), f"{skill_command} != 'push bowl1'"
    print("(3/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pour"], [0.1, "cup1"], [0.4, "to"] ],
                                 gesture_stamped = [ [0.4, "bowl1"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pour cup1 to bowl1"), f"{skill_command} != 'pour cup1 to bowl1'"
    print("(4/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "quickly"], [0.1, "pour"], [0.4, "cup1"], [0.6, "to"], [0.8, "bowl1"] ],
                                 gesture_stamped = [  ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("quickly pour cup1 to bowl1"), f"{skill_command} != 'quickly pour cup1 to bowl1'"
    print("(5/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "this"], [0.5, "right"], [0.6, "here"] ],
                                 gesture_stamped = [ [0.7, "bowl1"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick bowl1"), f"{skill_command} != 'pick bowl1'"
    print("(6/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "this"], [0.5, "right"], [0.6, "here"] ],
                                 gesture_stamped = [ [0.55, "bowl1"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick bowl1"), f"{skill_command} != 'pick bowl1'"
    print("(7/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "a"], [0.6, "bowl"], [0.7, "pen"] ],
                                 gesture_stamped = [ ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick bowl1"), f"{skill_command} != 'pick bowl1'"
    print("(8/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "a"], [0.7, "object"] ],
                                 gesture_stamped = [ [0.8, "cup1"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick cup1"), f"{skill_command} != 'pick cup1'"
    print("(9/9)")
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "a"], [0.7, "wide"], [0.8, "blue"], [0.9, "object"] ],
                                 gesture_stamped = [ ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick container1"), f"{skill_command} != 'pick container1'"
    print("(bonus)")
    
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "the"], [0.4, "red"], [0.5, "object"] ],
                                 gesture_stamped = [ ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick cup1"), f"{skill_command} != 'pick cup1'"
    print("Done")

    merger.hri.delete()
    rclpy.shutdown()



def test_unsuccessful():
    """ How it behaves when not enough knowledge provided """
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "this"] ],
                                 gesture_stamped = [ ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick this"), f"{skill_command} != 'pick this'"

    merger.hri.delete()
    rclpy.shutdown()


def test_on_scenarios():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", interpret_format="probabilistic")
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, {"pick": 1.0}],
            [0.1, {"up": 1.0}],
            [0.4, {"this": 1.0}],
        ],
        gesture_stamped = [
            [0.4, {"cup1": 1.0}],
        ], 
        role_description=get_role_description(
            A=ALL_COMMAND, 
            O=["cup1", "cup2", "bowl1"],
            S="On the scene are these objects: big blue cup1, small red cup2, big green bowl1.",
        ),
        max_new_tokens = 50,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    )
    assert skill_command == SkillCommand("pick cup1")

    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.4, "the"], [0.7, "apple"] ],
                                 gesture_stamped = [ [0.1, "quickly"] ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick cup1"), f"{skill_command} != 'pick cup1'"

    merger.hri.delete()
    rclpy.shutdown()

def test_alignment_noise(
        noise_levels = [0.0,0.2,0.4,0.6,0.8,1.0],
        # noise_levels = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0],
        # dataset_name = "D1",
        # dataset_name = "D2",
        # dataset_name = "D3",
        # dataset_name = "D1_CFG2",
        # dataset_name = "D2_CFG2",
        dataset_name = "D3_CFG2",
        ):
    """ Do not automate this, I think this is good like it is.
        1. Choose the Merger from commented options,
        2. Choose dataset D1 or D2
    """
    rclpy.init()

    interpret_format="deterministic"
    # interpret_format="probabilistic"
    # interpret_format="alternatives"
    for model_name in [
            # "SultanR/SmolTulu-1.7b-Reinforced",
            # "SultanR/SmolTulu-1.7b-Instruct",
            # "LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct",
            # "ibm-granite/granite-3.1-2b-instruct",
            "ArgmaxMerger",
            # "ZeroShotMerger",
            # "BeamSearchMerger",
        ]:
        if model_name == "ArgmaxMerger":
            merger = ArgmaxMerger(interpret_format=interpret_format)
        elif model_name == "ZeroShotMerger":
            merger = ZeroShotMerger(interpret_format=interpret_format)
        elif model_name == "BeamSearchMerger":
            merger = BeamSearchMerger(interpret_format=interpret_format)
        else:
            merger = HRIMerger(name_user="casper", model_name=model_name, interpret_format=interpret_format, tts_enabled=False)
        result_accuracy = []
        for noise in noise_levels:
            acc_sum = 0
            dataset = np.load(f"{llm_merger.path}/saved_datasets/{dataset_name}_n{noise}.npy", allow_pickle=True)
            for n,sample in enumerate(dataset):
                print("SAMPLE: ", n, " noise: ", noise)
                t0 = time.time()
                voice_stamped, gesture_stamped, scene, object_names, true_sentence, CONFIG = sample.export(interpret_format)
                
                role_description = get_role_description(A=CONFIG["actions"], O=object_names, S=scene)
                print(role_description)
                
                max_new_tokens = 1000
                temperature = 0.0
                top_p = 1.0
                repetition_penalty = 1.1
                skill_command = merger.merge( 
                    voice_stamped=voice_stamped,
                    gesture_stamped=gesture_stamped, 
                    role_description=role_description,
                    max_new_tokens = max_new_tokens,
                    temperature = temperature,
                    top_p = top_p,
                    repetition_penalty = repetition_penalty,
                    CONFIG = CONFIG,
                    object_names = object_names,
                )
                if SkillCommand(true_sentence) == skill_command:
                    acc_sum += 1
                    print(f"{cc.W}SUCCESS{cc.E}")
                    successful = True
                else:
                    print(f"{cc.F}WRONG{cc.E}")
                    successful = False
                
                # here save
                data = {
                    "successful": successful,
                    "true_sentence": true_sentence, 
                    "predicted_sentence": skill_command.command,
                    "predicted": skill_command.predicted,
                    "model_name": merger.name(),
                    "voice_stamped": voice_stamped,
                    "gesture_stamped": gesture_stamped,
                    "scene": scene, 
                    "object_names": object_names, 
                    "max_new_tokens": max_new_tokens, 
                    "temperature": temperature, 
                    "top_p": top_p, 
                    "repetition_penalty": repetition_penalty, 
                    "CONFIG": CONFIG, 
                    "role_description": role_description, 
                }
                # if not successful:
                #     i = 0
                #     while Path(f"{llm_merger.path}/saved_samples/save_{i}.json").is_file():
                #         i+=1
                #     with open(f"{llm_merger.path}/saved_samples/save_{i}.json", "w") as file:
                #         json.dump(data, file, indent=4)
                print(f"time: {time.time()-t0}")
            accuracy = float(acc_sum) / len(dataset)
            result_accuracy.append(accuracy)

        print(f"accuracy: {result_accuracy}")

        # np.save(f"{llm_merger.path}/saved_results/results_{merger.name()}_{interpret_format}", np.array([noise_levels, result_accuracy]))
        if model_name != "ArgmaxMerger":
            merger.hri.delete()
        # from llm_merger.plotter import save_plot
        # save_plot()
    rclpy.shutdown()


def test_on_saved_data(
        modality = "g+v",    
        object_names = ["cup", "cube", "plate", "table", "box"], # ",  "can", , "fork", "marker", "note", "storage", "blade", "rack", "ledge", "stand", "platform"
        action_names = ["pick", "push", "pass", "place", "point", "open", "close", "pour", "put", "stop", "release", "home"],
        scene = "cube is red cube. cup is red cup. plate is blue plate. table is big green table.",
        gt = ["pick cube", "pick_cube"],
        # gt = ["pick cube", "pick_red_object"],
        # gt = ["put cube to box", "put_cup_to_bowl"],
        # gt = ["put cube to box", "put_this_to_there"],

    ):
    true_sentence, folder = gt
    rclpy.init()
    CONFIG = CONFIG3
    interpret_format="deterministic"
    # interpret_format="probabilistic"
    # interpret_format="alternatives"
    for model_name in [
            # "SultanR/SmolTulu-1.7b-Reinforced",
            # "SultanR/SmolTulu-1.7b-Instruct",
            # "LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct",
            # "ibm-granite/granite-3.1-2b-instruct",
            "ArgmaxMerger",
            # "ZeroShotMerger",
            # "BeamSearchMerger",
        ]:
        if model_name == "ArgmaxMerger":
            merger = ArgmaxMerger(interpret_format=interpret_format)
        elif model_name == "ZeroShotMerger":
            merger = ZeroShotMerger(interpret_format=interpret_format)
        elif model_name == "BeamSearchMerger":
            merger = BeamSearchMerger(interpret_format=interpret_format)
        else:
            merger = HRIMerger(name_user="casper", model_name=model_name, interpret_format=interpret_format, tts_enabled=False)
    
        
        j = 0
        acc_sum = 0
        while Path(f"{llm_merger.path}/saved_inputs/{folder}/save_{j}.npz").is_file():
            save = np.load(f"{llm_merger.path}/saved_inputs/{folder}/save_{j}.npz", allow_pickle=True)
            j+=1

            
            voicecommand, hricommand = save["voice_command"], save["gesture_command"].item()
            
            if interpret_format == "deterministic":
                gesture_stamped = hricommand.get_target_timestamped_list()
                voice_stamped = voicecommand
                ret = []
                for t,w in voicecommand:
                    ret.append([t,sorted(w])
            elif interpret_format in ["probabilistic", "alternatives"]:
                gesture_stamped = hricommand.get_target_timestamped_probabilistic()
                voice_stamped = voicecommand
            print(gesture_stamped, voice_stamped)
            
            role_description = get_role_description(A=action_names, O=object_names, S=scene)
            print(role_description)
            
            max_new_tokens = 1000
            temperature = 0.0
            top_p = 1.0
            repetition_penalty = 1.1
            skill_command = merger.merge( 
                voice_stamped=voice_stamped if "v" in modality else [],
                gesture_stamped=gesture_stamped if "g" in modality else [], 
                role_description=role_description,
                max_new_tokens = max_new_tokens,
                temperature = temperature,
                top_p = top_p,
                repetition_penalty = repetition_penalty,
                CONFIG = CONFIG,
                object_names = object_names,
            )
            if SkillCommand(true_sentence) == skill_command:
                acc_sum += 1
                print(f"{cc.W}SUCCESS{cc.E}")
                successful = True
            else:
                print(f"{cc.F}WRONG{cc.E}")
                successful = False
            
            # here save
            data = {
                "successful": successful,
                "true_sentence": true_sentence, 
                "predicted_sentence": skill_command.command,
                "predicted": skill_command.predicted,
                "model_name": merger.name(),
                "voice_stamped": [list(v) for v in voice_stamped],
                "gesture_stamped": [list(v) for v in gesture_stamped],
                "scene": scene, 
                "object_names": object_names, 
                "max_new_tokens": max_new_tokens, 
                "temperature": temperature, 
                "top_p": top_p, 
                "repetition_penalty": repetition_penalty, 
                "CONFIG": CONFIG, 
                "role_description": role_description, 
            }
            # if not successful:
            #     i = 0
            #     while Path(f"{llm_merger.path}/saved_samples/save_{i}.json").is_file():
            #         i+=1
            #     with open(f"{llm_merger.path}/saved_samples/save_{i}.json", "w") as file:
            #         json.dump(data, file, indent=4)
            
        accuracy = float(acc_sum) / j

        print("final acc", accuracy)
        # np.save(f"{llm_merger.path}/saved_results/results_{merger.name()}_{interpret_format}", np.array([accuracy]))
        if model_name != "ArgmaxMerger":
            merger.hri.delete()
        # from llm_merger.plotter import save_plot
        # save_plot()
    rclpy.shutdown()

 


if __name__ == "__main__":
    # test_skill_commands()
    # test_just_to_see_if_works()
    # test_just_probabilistic()
    # test_just_alternatives()
    # test_just_alternatives2()
    # test_just_alternatives3()

    # test_lm()
    # test_unsuccessful()

    # test_alignment_noise()
    test_on_saved_data()