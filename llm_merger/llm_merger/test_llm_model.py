
from llm_merger.llm_model import HRIMerger
import rclpy
import llm_merger
import numpy as np
from pathlib import Path

from llm_merger.role_setup import get_role_description
from llm_merger.generate_dataset import EnhancedDatasetGenerator, CONFIG

""" When I install this package, I always try to run this function """
def test_just_to_see_if_works():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    results = merger.merge( 
        voice_stamped = [
            [0.0, "Pick"],
            [0.1, "Green"],
            [0.4, "Cup"],
        ],
        gesture_stamped = [
            [0.4, ", pointing at a Cup, "],
        ],
        role_description=get_role_description(A=["Pick", "Push", "Pour"], O=["Cup", "Drawer", "Bowl"]),
        max_new_tokens = 50,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    )
    assert results["target_action"].lower() == "pick" and results["target_object"].lower() == "cup"

def test_just_probabilistic():
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", merge_approach="probabilistic")
    results = merger.merge( 
        voice_stamped = [
            [0.0, {"pick": 0.5, "kick": 0.5}],
            [0.1, {"green": 0.9, "in": 0.1}],
            [0.4, {"cup": 0.9, "cap": 0.1}],
        ],
        gesture_stamped = [
            [0.4, {"cup": 0.9, "container": 0.1}],
        ], 
        role_description=get_role_description(
            A=["pick", "push", "pour"], 
            O=["cup", "cabinet", "bowl"]
        ),
        max_new_tokens = 50,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    )
    assert results["target_action"].lower() == "pick" and results["target_object"].lower() == "cup"

def test_on_generated_data():
    rclpy.init()
    generator = EnhancedDatasetGenerator()
    sample = generator.generate_sample()
    O = sample.scene.keys()
    A = CONFIG["actions"]
    print(f"A: {A}, O: {O}")

    role_description=get_role_description(A=["Pick", "Push", "Pour"], O=["Cup", "Drawer", "Bowl"])
    max_new_tokens = 50
    temperature = 0.0
    top_p = 1.0
    repetition_penalty = 1.1

    print(sample)

    # for sample.
    # voice_stamped =



    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    
    voice_stamped = [
        [0.0, "Pick"],
        [0.1, "Green"],
        [0.4, "Cup"],
    ]
    gesture_stamped = [
        [0.4, ", pointing at a Cup, "],
    ]
    results = merger.merge( 
        voice_stamped,
        gesture_stamped, 
        role_description=role_description,
        max_new_tokens = max_new_tokens,
        temperature = temperature,
        top_p = top_p,
        repetition_penalty = repetition_penalty,
    )
    assert results["target_action"].lower() == "pick" and results["target_object"].lower() == "cup"


def test_on_saved_data(
        role_description=get_role_description(A=["Pick", "Push", "Pour"], O=["Cup", "Drawer", "Bowl"]),
        max_new_tokens = 50,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    ):
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    
    i = 0
    while Path(f"{llm_merger.path}/saved_inputs/save_{i}").is_file():
        save = np.load(f"{llm_merger.path}/saved_inputs/save_{i}")

        merger.forward( 
            save['voice_stamped'],
            save['gesture_stamped'], 
            role_description=role_description,
            max_new_tokens = max_new_tokens,
            temperature = temperature,
            top_p = top_p,
            repetition_penalty = repetition_penalty,
        )

        i+=1

if __name__ == "__main__":
    test_just_to_see_if_works()
    test_just_probabilistic()
    # test_on_generated_data()