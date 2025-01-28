from copy import deepcopy
import json
from typing import Any, Dict, List

from hri_manager.modality_merger import merge_probabilities
from hri_manager.probs_vector import EntropyProbsVector, NaiveProbsVector, ProbsVector

THRESHOLDING = "entropy"

class HriCommand():
    def __init__(
        self,
        arity_names: List[str],
        pv_dict: Dict[str, Any],
        ):
        self.arity_names = arity_names
        self.pv_dict = pv_dict
        self.results_dict = self.apply_thresholding()

    
    @property
    def target_action(self):
        return self.pv_dict["action"].max

    @property
    def target_object(self):
        return self.pv_dict["object"].max


    def __matmul__(self, other):
        """ Do modality merging of two HriCommand objects as (hri_command1 @ hri_command2) """
        assert self.arity_names == other.arity_names
        for arity_type in self.arity_names:
            assert (self.pv_dict[arity_type].names == other.pv_dict[arity_type].names).all(), f'{self.pv_dict[arity_type]} != {other.pv_dict[arity_type]}'

        data_dict_new = {}
        data_dict = merge_probabilities(self.probs_dict, other.probs_dict, thresholding=THRESHOLDING)
        for arity_type in self.arity_names:
            data_dict_new[arity_type+"_probs"] = data_dict[arity_type].p
            data_dict_new[arity_type+"_names"] = self.pv_dict[arity_type].names

        return HriCommand.from_dict(self.arity_names, data_dict_new, thresholding=THRESHOLDING)

    @property
    def probs_dict(self): # {"action": [...], "object": [...]} instead of {"action_probs": [...], "object_probs": [...]}
        r = {}
        for arity in self.arity_names:
            r[arity] = self.pv_dict[arity].p
        return r

    def apply_thresholding(self):
        targets_dict = {}
        for arity_type in self.arity_names:
            targets_dict["target_"+arity_type] = self.pv_dict[arity_type].apply_thresholding()
        return targets_dict

    @property
    def data_dict(self):
        ret = {}
        for arity_type in self.arity_names:
            ret[arity_type] = self.pv_dict[arity_type].p
        return ret
    

    @classmethod
    def from_ros(cls, msg, thresholding="entropy"):
        msg_dict = json.loads(msg.data[0])

        arity_names = [] # just add arity_names to the HRICommand definition!
        for k in msg_dict.keys():
            if "_probs" in k:
                arity_names.append(k.split("_probs")[0])

        pv_dict = {}
        for arity_type in arity_names:
            names = msg_dict[arity_type+"_names"]
            probs = msg_dict[arity_type+"_probs"]

            if thresholding == "no thresholding":
                pv_dict[arity_type] = NaiveProbsVector(probs, names)
            elif thresholding == "fixed":
                pv_dict[arity_type] = ProbsVector(probs, names)
            elif thresholding == "entropy":
                pv_dict[arity_type] = EntropyProbsVector(probs, names)
            else: raise Exception()

        return cls(arity_names, pv_dict)
    
    @classmethod
    def from_dict(cls, arity_names, data_dict, thresholding):
        pv_dict = {}
        for arity_type in arity_names:
            names = data_dict[arity_type+"_names"]
            probs = data_dict[arity_type+"_probs"]

            if thresholding == "no thresholding":
                pv_dict[arity_type] = NaiveProbsVector(probs, names)
            elif thresholding == "fixed":
                pv_dict[arity_type] = ProbsVector(probs, names)
            elif thresholding == "entropy":
                pv_dict[arity_type] = EntropyProbsVector(probs, names)
            else: raise Exception()

        return cls(arity_names, pv_dict)

    def to_dict(self):
        return_dict = {}
        return_dict["arity_names"] = self.arity_names
        for arity_type in self.arity_names:
            return_dict[arity_type] = self.pv_dict[arity_type].names
            return_dict[arity_type+"_probs"] = self.pv_dict[arity_type].p
        for k in self.results_dict.keys():
            return_dict[k] = self.results_dict[k]
        return return_dict
    
    def to_ros(self):
        from hri_msgs.msg import HRICommand as HRICommandMSG # import is here to keep ROS independency

        return HRICommandMSG(data=[str(json.dumps(self.to_dict()))])
    
    def __str__(self):
        s = ""
        for arity_type in self.arity_names:
            s += f"[[{arity_type}]]\n{self.pv_dict[arity_type].info()}\n"
        return s
            
    def __eq__(self, other):
        if self.arity_names == other.arity_names:
            if self.pv_dict == other.pv_dict:
                return True
        return False