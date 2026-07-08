#!/usr/bin/env python
"""Gesture-name -> action-name mapping (OneToOneMapping), as used by
ReasoningMerger.merge() before interleaving the modalities. Pure function
call: no LLM, no ROS services needed."""
from gesture_meaning.one_to_one_mapping import OneToOneMapping

import pathlib
import yaml
import pytest

# 1. Load test-case data from YAML                                            #
DATA_PATH = pathlib.Path(__file__).with_name("data") / "gesture_meaning_cases.yaml"
with DATA_PATH.open() as f:
    _DATA = yaml.safe_load(f)

_CASES: list[dict] = _DATA["cases"]

# 2. Build the parameter list for pytest                                      #
_PARAMS = [
    pytest.param(
        case["gesture"],             # gesture_stamped, as delivered by the gesture modality
        case["expected"],            # gesture_stamped with gestures replaced by actions
        id=case.get("id", f"case-{idx}"),
    )
    for idx, case in enumerate(_CASES, start=1)
]

# 3. The single parametrised test                                             #
@pytest.mark.parametrize("gesture, expected", _PARAMS)
def test_gesture_meaning_cases(gesture, expected):
    result = OneToOneMapping().map_stamped(gesture)
    assert result == expected, f"SEE THIS: MAPPED: {result} != GROUND TRUTH: {expected}"
