
from multi_modal_reasoning.reasoning_merger import ReasoningMerger, ArgmaxMerger, ZeroShotMerger, BeamSearchMerger

import rclpy
import multi_modal_reasoning
import numpy as np
from pathlib import Path

from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.generate_dataset import EnhancedDatasetGenerator, CONFIG, CONFIG3

from multi_modal_reasoning.skill_command import SkillCommand
import json, copy
from multi_modal_reasoning.generate_dataset import Sample
from naive_merger.utils import cc
import time
from tqdm import tqdm

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
    # merger = ReasoningMerger(name_user="casper", model_name="deepseek-ai/DeepSeek-R1-Distill-Qwen-1.5B")
    # merger = ReasoningMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Instruct")
    # merger = ReasoningMerger(name_user="casper", model_name="LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct")
    merger = ReasoningMerger(name_user="casper", model_name="ibm-granite/granite-3.1-2b-instruct")
    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge( 
        voice_stamped = [
            [0.0, "pick"],
            [0.1, "green"],
            [0.4, "cup"],
        ],
        gesture_stamped = [
            [0.4, "cup1"],
        ],
        role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "drawer", "bowl"], S=S),
        command_constraints=CONFIG3,
        max_new_tokens = 1000,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
        quantization = 4,
    )
    merger.save_log("pick cup1", skill_command, [[0.0, "pick"],[0.1, "green"],[0.4, "cup"],], [ [0.4, "cup1"],], S, ["cup1", "drawer", "bowl"], 1000, 0.0, 1.0, 1.1, CONFIG3, get_role_description(A=["pick", "push", "pour"], O=["cup1", "drawer", "bowl"], S=S, quantization=4))
    assert skill_command == SkillCommand("pick cup1"), f"{skill_command} != 'pick cup1'"
    merger.hri.delete()
    rclpy.shutdown()


def test_just_probabilistic():
    rclpy.init()
    merger = ReasoningMerger(name_user="casper", model_name="deepseek-ai/DeepSeek-R1-Distill-Qwen-1.5B", interpret_format="probabilistic")
    # merger = ReasoningMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Instruct", interpret_format="probabilistic")
    # merger = ReasoningMerger(name_user="casper", model_name="LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct", interpret_format="probabilistic")
    # merger = ReasoningMerger(name_user="casper", model_name="ibm-granite/granite-3.1-2b-instruct", interpret_format="probabilistic")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl. box is box."
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
            O=["cup1", "box", "bowl1"],
            S=S,
        ),
        command_constraints=CONFIG3,
        quantization = 4,
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_just_alternatives():
    rclpy.init()
    merger = ReasoningMerger(name_user="casper", model_name="ibm-granite/granite-3.1-2b-instruct", interpret_format="alternatives")

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
        ), 
        command_constraints=CONFIG3
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_just_alternatives2():
    rclpy.init()

    model_name="SultanR/SmolTulu-1.7b-Reinforced"
    model_name="LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct"
    model_name="ibm-granite/granite-3.1-2b-instruct"
    merger = ReasoningMerger(name_user="casper", model_name=model_name, interpret_format="alternatives")

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
        ), 
        command_constraints=CONFIG3
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_just_alternatives3():
    rclpy.init()
    merger = ReasoningMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", interpret_format="alternatives")

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
        ), 
        command_constraints=CONFIG3
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()

def test_lm():
    rclpy.init()
    merger = ReasoningMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
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
    merger = ReasoningMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")

    S = "In the scene are three objects. The cup1 is big red cup. The container1 is wide blue container. bowl1 is green small bowl."
    skill_command = merger.merge(voice_stamped = [[0.0, "Pick"], [0.1, "up"], [0.4, "this"] ],
                                 gesture_stamped = [ ],
                                 role_description=get_role_description(A=["pick", "push", "pour"], O=["cup1", "container1", "bowl1"], S=S))
    assert skill_command == SkillCommand("pick this"), f"{skill_command} != 'pick this'"

    merger.hri.delete()
    rclpy.shutdown()

def test_alignment_noise(
        noise_levels = [0.0,0.2,0.4,0.6,0.8,1.0],
        # dataset_name = "D1_CFG3",
        dataset_name = "D3_CFG3",
        # interpret_format="deterministic",
        # interpret_format="probabilistic",
        interpret_format="alternatives",
        cfg = CONFIG3,
        max_new_tokens = 1000,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
        quantization = 4,
        ):
    """ Do not automate this, I think this is good like it is.
    """
    rclpy.init()
    try: # Ctrl+C to quit and see the results
        for model_name in [
                # "SultanR/SmolTulu-1.7b-Instruct",
                "LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct",
                # "ibm-granite/granite-3.1-2b-instruct",
                # "ArgmaxMerger",
            ]:
            if model_name == "ArgmaxMerger":
                merger = ArgmaxMerger(interpret_format=interpret_format)
            elif model_name == "ZeroShotMerger":
                merger = ZeroShotMerger(interpret_format=interpret_format)
            elif model_name == "BeamSearchMerger":
                merger = BeamSearchMerger(interpret_format=interpret_format)
            else:
                merger = ReasoningMerger(name_user="casper", model_name=model_name, interpret_format=interpret_format, tts_enabled=False)
            result_accuracy = []
            for noise in tqdm(noise_levels, desc="Noise levels:", position=0, leave=True):
                acc_sum = 0
                dataset = np.load(f"{multi_modal_reasoning.path}/saved_datasets/{dataset_name}_n{noise}.npy", allow_pickle=True)
                n=-1
                for sample in tqdm(dataset, desc=f"Sample (noise={noise}):", position=1, leave=False):
                    n+=1
                    voice_stamped, gesture_stamped, scene, object_names, true_sentence, CONFIG = sample.export(interpret_format)
                    
                    role_description = get_role_description(A=cfg["actions"], O=object_names, S=scene)
                    
                    skill_command = merger.merge( 
                        voice_stamped=voice_stamped,
                        gesture_stamped=gesture_stamped, 
                        role_description=role_description,
                        max_new_tokens = max_new_tokens,
                        temperature = temperature,
                        top_p = top_p,
                        repetition_penalty = repetition_penalty,
                        command_constraints = cfg,
                        object_names = object_names,
                        quantization = quantization,
                    )
                    if not (SkillCommand(true_sentence) == skill_command):
                        merger.save_log(true_sentence, skill_command, voice_stamped, gesture_stamped, scene, object_names,
                            max_new_tokens, temperature, top_p, repetition_penalty, cfg, role_description, quantization)
                    if SkillCommand(true_sentence) == skill_command:
                        acc_sum += 1
                        print(f"{cc.W}SUCCESS{cc.E}")
                    else:
                        print(f"{cc.F}WRONG{cc.E}")
                    
                    if model_name != "ArgmaxMerger":
                        merger.hri.delete()
                accuracy = float(acc_sum) / len(dataset)
                result_accuracy.append(accuracy)

                print(f"Accuracy: {accuracy} on noise {noise}")

            print(f"accuracy: {result_accuracy}")
    except KeyboardInterrupt:
        print("interrupted")
        print("result_accuracy: ", result_accuracy)
    rclpy.shutdown()


def test_on_saved_data(
        modality = "g+v",    
        object_names = ["cup", "cube", "plate", "table", "box"], # ",  "can", , "fork", "marker", "note", "storage", "blade", "rack", "ledge", "stand", "platform"
        action_names = ["pick", "push", "pass", "place", "point", "open", "close", "pour", "put", "stop", "release", "home"],
        scene = "cube is red cube. cup is red cup. plate is blue plate. table is big green table. box is small black box",
        # gt = ["pick cube", "pick_cube"],
        # gt = ["pick cube", "pick_red_object"],
        gt = ["put cube to box", "put_the_red_thing_to_the_black_thing"],
        # gt = ["put cube to box", "put_this_to_there"],
        max_new_tokens = 1000,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
        quantization = 4,
    ):
    true_sentence, folder = gt
    rclpy.init()
    cfg = CONFIG3
    # interpret_format="deterministic"
    # interpret_format="probabilistic"
    interpret_format="alternatives"
    model_accuracies = []
    for model_name in [
            # "SultanR/SmolTulu-1.7b-Reinforced",
            "SultanR/SmolTulu-1.7b-Instruct",
            "LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct",
            "ibm-granite/granite-3.1-2b-instruct",
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
            merger = ReasoningMerger(name_user="casper", model_name=model_name, interpret_format=interpret_format, tts_enabled=False)
    
        
        j = 0
        acc_sum = 0
        while Path(f"{multi_modal_reasoning.path}/saved_inputs/{folder}/save_{j}.npz").is_file():
            save = np.load(f"{multi_modal_reasoning.path}/saved_inputs/{folder}/save_{j}.npz", allow_pickle=True)
            j+=1

            try:
                voicecommand, hricommand = save["voice_command"], save["gesture_command"].item()
            except ValueError:
                continue


            if interpret_format == "deterministic":
                gesture_stamped = hricommand.get_target_timestamped_list()
                ret = []
                for t,w in voicecommand:
                    maxv = [itm[0] for itm in sorted(w.items(), key=lambda x: x[1], reverse=True)][0]
                    ret.append([t,maxv])
                voice_stamped = ret
            elif interpret_format in ["probabilistic", "alternatives"]:
                gesture_stamped = hricommand.get_target_timestamped_probabilistic()
                voice_stamped = voicecommand
            print(gesture_stamped, voice_stamped)
            
            role_description = get_role_description(A=action_names, O=object_names, S=scene)
            print(role_description)

            skill_command = merger.merge( 
                voice_stamped=voice_stamped if "v" in modality else [],
                gesture_stamped=gesture_stamped if "g" in modality else [], 
                role_description=role_description,
                max_new_tokens = max_new_tokens,
                temperature = temperature,
                top_p = top_p,
                repetition_penalty = repetition_penalty,
                command_constraints = cfg,
                object_names = object_names,
            )
            if not (SkillCommand(true_sentence) == skill_command):
                merger.save_log(true_sentence, skill_command, voice_stamped, gesture_stamped, scene, object_names,
                    max_new_tokens, temperature, top_p, repetition_penalty, cfg, role_description, quantization)
            if SkillCommand(true_sentence) == skill_command:
                acc_sum += 1
                print(f"{cc.W}SUCCESS{cc.E}")
            else:
                print(f"{cc.F}WRONG{cc.E}")
            

        accuracy = float(acc_sum) / j
        model_accuracies.append([model_name, interpret_format, accuracy])
        print("final acc", accuracy)
        if model_name != "ArgmaxMerger":
            merger.hri.delete()
    print("model accuracies:")
    print(model_accuracies)
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

    test_alignment_noise()
    # test_on_saved_data()