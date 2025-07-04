#!/usr/bin/env python
from multi_modal_reasoning.reasoning_merger import ReasoningMerger, ArgmaxMerger, ZeroShotMerger, BeamSearchMerger
import rclpy
import multi_modal_reasoning
import numpy as np
from pathlib import Path
from multi_modal_reasoning.role_setup import get_role_description
from multi_modal_reasoning.generate_dataset import CONFIG3
from multi_modal_reasoning.skill_command import SkillCommand
from naive_merger.utils import cc
from tqdm import tqdm

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
                    voice_stamped, gesture_stamped, scene, object_names, true_sentence, _ = sample.export(interpret_format)
                    
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
                    )
                    if not (SkillCommand(true_sentence, cfg) == skill_command):
                        merger.save_log(true_sentence, skill_command, voice_stamped, gesture_stamped, scene, object_names,
                            max_new_tokens, temperature, top_p, repetition_penalty, cfg, role_description)
                    if SkillCommand(true_sentence, cfg) == skill_command:
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
    ):
    true_sentence, folder = gt
    rclpy.init()
    cfg = CONFIG3
    # interpret_format="deterministic"
    # interpret_format="probabilistic"
    interpret_format="alternatives"
    model_accuracies = []
    for model_name in [
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
            if not (SkillCommand(true_sentence, cfg) == skill_command):
                merger.save_log(true_sentence, skill_command, voice_stamped, gesture_stamped, scene, object_names,
                    max_new_tokens, temperature, top_p, repetition_penalty, cfg, role_description)
            if SkillCommand(true_sentence, cfg) == skill_command:
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
