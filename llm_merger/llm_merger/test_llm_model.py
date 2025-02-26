
from llm_merger.llm_model import HRIMerger
import rclpy
import llm_merger
import numpy as np
from pathlib import Path

from llm_merger.role_setup import get_role_description
from llm_merger.generate_dataset import EnhancedDatasetGenerator, CONFIG

from llm_merger.skill_command import ALL_COMMAND, OBJECT_TYPES, SkillCommand

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
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", merge_approach="probabilistic")

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
    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", merge_approach="probabilistic")
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

def test_on_generated_data(merge_approach="deterministic"):
    rclpy.init()
    generator = EnhancedDatasetGenerator()
    sample = generator.generate_sample()
    voice_stamped, gesture_stamped, scene, object_names, true_sentence, CONFIG = sample.export(merge_approach)
    
    print("role description:")
    print(get_role_description(A=CONFIG["actions"], O=object_names, S=scene))

    merger = HRIMerger(name_user="casper", model_name="SultanR/SmolTulu-1.7b-Reinforced", merge_approach=merge_approach)
    skill_command = merger.merge( 
        voice_stamped=voice_stamped,
        gesture_stamped=gesture_stamped, 
        role_description=get_role_description(A=CONFIG["actions"], O=object_names, S=scene),
        max_new_tokens = 1000,
        temperature = 0.0,
        top_p = 1.0,
        repetition_penalty = 1.1,
    )
    assert SkillCommand(true_sentence) == skill_command

    merger.hri.delete()
    rclpy.shutdown()


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
    # test_skill_commands()
    # test_just_to_see_if_works()
    # test_just_probabilistic()
    # test_lm()
    # test_unsuccessful()
    test_on_generated_data()