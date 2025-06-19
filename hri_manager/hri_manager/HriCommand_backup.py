from copy import deepcopy
import json
from typing import Any, Dict, List

from naive_merger.modality_merger import merge_probabilities
from naive_merger.probs_vector import EntropyProbsVector, NaiveProbsVector, ProbsVector

THRESHOLDING = "entropy"

class HriCommand():
    def __init__(
        self,
        arity_names: List[str],
        pv_dict: Dict[str, Any],
        stamps = None,
        ):
        self.arity_names = arity_names
        self.pv_dict = pv_dict
        self.results_dict = self.apply_thresholding()
        self.stamps = stamps
    
    @property
    def target_action(self):
        return self.pv_dict["action"].max

    @property
    def target_object(self):
        return self.pv_dict["object"].max

    @property
    def target_storage(self):
        return self.pv_dict["storage"].max


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
        stamps = {}
        for arity_type in arity_names:
            names = msg_dict[arity_type+"_names"]
            probs = msg_dict[arity_type+"_probs"]
            if (f"target_{arity_type}_timestamp") in msg_dict:
                stamps[arity_type] = msg_dict[f"target_{arity_type}_timestamp"]

            if thresholding == "no thresholding":
                pv_dict[arity_type] = NaiveProbsVector(probs, names)
            elif thresholding == "fixed":
                pv_dict[arity_type] = ProbsVector(probs, names)
            elif thresholding == "entropy":
                pv_dict[arity_type] = EntropyProbsVector(probs, names)
            else: raise Exception()

        return cls(arity_names, pv_dict, stamps)
    
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
    
    def get_action_stamp(self):
        if 'action' in self.stamps:
            return self.stamps["action"]
        else:
            return -1.0

    def get_object_stamp(self):
        if 'object' in self.stamps:
            return self.stamps["object"]
        else:
            return -1.0

    def get_storage_stamp(self):
        if 'storage' in self.stamps:
            return self.stamps["storage"]
        else:
            return -1.0

    def get_target_timestamped_list(self):
        if "storage" in self.pv_dict:
            return [
                [self.get_action_stamp(), self.target_action],
                [self.get_object_stamp(), self.target_object],
                [self.get_storage_stamp(), self.target_storage],
            ]
        else:
            if ("action" in self.pv_dict) and ("object" in self.pv_dict):
                return [
                    [self.get_action_stamp(), self.target_action],
                    [self.get_object_stamp(), self.target_object],
                ]
            elif ("action" in self.pv_dict) and not ("object" in self.pv_dict):
                return [
                    [self.get_action_stamp(), self.target_action],
                ]
            elif not ("action" in self.pv_dict) and ("object" in self.pv_dict):
                return [
                    [self.get_object_stamp(), self.target_object],
                ]
            else:
                return []


    def get_target_timestamped_probabilistic(self):
        if "storage" in self.pv_dict:
            return [
                [self.get_action_stamp(), self.pv_dict["action"].dict],
                [self.get_object_stamp(), self.pv_dict["object"].dict],
                [self.get_storage_stamp(), self.pv_dict["storage"].dict],
            ]
        else:
            return [
                [self.get_action_stamp(), self.pv_dict["action"].dict],
                [self.get_object_stamp(), self.pv_dict["object"].dict],
            ]
