
from llm_merger.llm_model import HRIMerger
import rclpy
import llm_merger
import numpy as np
from pathlib import Path

from llm_merger.role_setup import get_role_description

def test_on_artificial_data(
        role_description=get_role_description(A=["Pick", "Push", "Pour"], O=["Cup", "Drawer", "Bowl"]),
        max_new_tokens = 50,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    ):
    rclpy.init()
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced")
    
    voice_stamped = [
        [0.0, "Pick"],
        [0.1, "Green"],
        [0.4, "Cup"],
    ]
    gesture_stamped = [
        [0.4, ", pointing at a cup, "],
    ]
    results = merger.forward( 
        voice_stamped,
        gesture_stamped, 
        role_description=role_description,
        max_new_tokens = max_new_tokens,
        temperature = temperature,
        top_p = top_p,
        repetition_penalty = repetition_penalty,
    )
    assert results.action == "Pick" and results.object == "Cup"


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