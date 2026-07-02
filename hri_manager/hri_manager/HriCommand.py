"""HriCommand: one modality's probabilistic command hypotheses.

Each "arity" (e.g. "action", "object", "storage") holds a ProbsVector of
candidate names with probabilities. Commands from two modalities (e.g.
gestures and voice) are merged with the @ operator. Serializes to/from the
HRICommand ROS message, whose payload is one JSON string with keys
"<arity>_names", "<arity>_probs" and optional "target_<arity>_timestamp".
"""
import json
from typing import Any, Dict, List

import numpy as np
from naive_merger.modality_merger import merge_probabilities
from naive_merger.probs_vector import EntropyProbsVector, NaiveProbsVector, ProbsVector

THRESHOLDING = "entropy"

# thresholding mode -> ProbsVector flavour deciding which candidates survive
PROBS_VECTOR_TYPES = {
    "no thresholding": NaiveProbsVector,
    "fixed": ProbsVector,
    "entropy": EntropyProbsVector,
}


class HriCommand():
    def __init__(
        self,
        arity_names: List[str],
        pv_dict: Dict[str, Any],  # arity -> ProbsVector
        stamps: Dict[str, float] = None,  # arity -> detection timestamp [s]
        ):
        self.arity_names = arity_names
        self.pv_dict = pv_dict
        self.stamps = stamps if stamps is not None else {}
        self.results_dict = self.apply_thresholding()

    # ------------------------------------------------------------------ #
    # Construction                                                        #
    # ------------------------------------------------------------------ #
    @classmethod
    def from_dict(cls, arity_names, data_dict, thresholding=THRESHOLDING, stamps=None):
        """`data_dict` holds "<arity>_names" and "<arity>_probs" per arity."""
        try:
            make_pv = PROBS_VECTOR_TYPES[thresholding]
        except KeyError:
            raise ValueError(f"Unknown thresholding {thresholding!r}, expected one of {list(PROBS_VECTOR_TYPES)}")
        pv_dict = {
            arity: make_pv(data_dict[f"{arity}_probs"], data_dict[f"{arity}_names"])
            for arity in arity_names
        }
        return cls(arity_names, pv_dict, stamps)

    @classmethod
    def from_ros(cls, msg, thresholding=THRESHOLDING):
        """The message payload (see module docstring) names its own arities."""
        msg_dict = json.loads(msg.data[0])
        arity_names = [k.removesuffix("_probs") for k in msg_dict if k.endswith("_probs")]
        stamps = {
            arity: msg_dict[f"target_{arity}_timestamp"]
            for arity in arity_names if f"target_{arity}_timestamp" in msg_dict
        }
        return cls.from_dict(arity_names, msg_dict, thresholding, stamps)

    # ------------------------------------------------------------------ #
    # Winning targets                                                     #
    # ------------------------------------------------------------------ #
    @property
    def target_action(self):
        return self.pv_dict["action"].max

    @property
    def target_object(self):
        return self.pv_dict["object"].max

    @property
    def target_storage(self):
        return self.pv_dict["storage"].max

    def apply_thresholding(self):
        return {f"target_{arity}": self.pv_dict[arity].apply_thresholding()
                for arity in self.arity_names}

    def get_stamp(self, arity: str) -> float:
        """Detection timestamp of the arity, -1.0 when unknown."""
        return self.stamps.get(arity, -1.0)

    def get_action_stamp(self):
        return self.get_stamp("action")

    def get_object_stamp(self):
        return self.get_stamp("object")

    def get_storage_stamp(self):
        return self.get_stamp("storage")

    def get_target_timestamped_list(self):
        """Winning targets as [[stamp, word], ...], ordered action, object, storage.

        Storage is included only when it differs from the object: an equal
        storage means no distinct second object was gestured."""
        out = []
        if "action" in self.pv_dict:
            out.append([self.get_stamp("action"), self.target_action])
        if "object" in self.pv_dict:
            out.append([self.get_stamp("object"), self.target_object])
        if ("storage" in self.pv_dict and "action" in self.pv_dict
                and "object" in self.pv_dict and self.target_storage != self.target_object):
            out.append([self.get_stamp("storage"), self.target_storage])
        return out

    def get_target_timestamped_probabilistic(self):
        """Like get_target_timestamped_list, but each entry carries the full
        {name: prob} candidates instead of only the winner."""
        arities = ["action", "object"] + (["storage"] if "storage" in self.pv_dict else [])
        return [[self.get_stamp(arity), self.pv_dict[arity].dict] for arity in arities]

    # ------------------------------------------------------------------ #
    # Modality merging                                                    #
    # ------------------------------------------------------------------ #
    def __matmul__(self, other):
        """Merge two modalities: merged = voice_command @ gesture_command."""
        assert self.arity_names == other.arity_names
        for arity in self.arity_names:
            assert np.array_equal(self.pv_dict[arity].names, other.pv_dict[arity].names), \
                f"{self.pv_dict[arity]} != {other.pv_dict[arity]}"

        merged = merge_probabilities(self.probs_dict, other.probs_dict,
                                     thresholding=THRESHOLDING, arity_names=self.arity_names)
        data_dict = {}
        for arity in self.arity_names:
            data_dict[f"{arity}_probs"] = merged[arity]
            data_dict[f"{arity}_names"] = self.pv_dict[arity].names
        return HriCommand.from_dict(self.arity_names, data_dict, THRESHOLDING)

    @property
    def probs_dict(self):
        """arity -> probabilities array, e.g. {"action": [...], "object": [...]}."""
        return {arity: self.pv_dict[arity].p for arity in self.arity_names}

    data_dict = probs_dict  # legacy alias

    # ------------------------------------------------------------------ #
    # Serialization                                                       #
    # ------------------------------------------------------------------ #
    def to_dict(self):
        d = {"arity_names": self.arity_names}
        for arity in self.arity_names:
            d[f"{arity}_names"] = list(self.pv_dict[arity].names)
            d[f"{arity}_probs"] = list(self.pv_dict[arity].p)
        d.update(self.results_dict)
        return d

    def to_ros(self):
        from hri_msgs.msg import HRICommand as HRICommandMSG  # import here to keep ROS independency
        return HRICommandMSG(data=[json.dumps(self.to_dict())])

    def __str__(self):
        return "".join(f"[[{arity}]]\n{self.pv_dict[arity].info()}\n"
                       for arity in self.arity_names)

    def __eq__(self, other):
        """Same arities with (numerically) the same candidates and probabilities."""
        if self.arity_names != other.arity_names:
            return False
        return all(
            np.array_equal(self.pv_dict[arity].names, other.pv_dict[arity].names)
            and np.allclose(self.pv_dict[arity].p, other.pv_dict[arity].p)
            for arity in self.arity_names
        )
