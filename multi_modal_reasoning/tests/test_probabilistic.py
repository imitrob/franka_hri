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
    )
    assert skill_command == SkillCommand("pick box")
    merger.hri.delete()
    rclpy.shutdown()