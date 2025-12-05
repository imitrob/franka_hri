from __future__ import annotations
import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Type

from naive_merger.modality_merger import merge_probabilities
from naive_merger.probs_vector import (
    EntropyProbsVector,
    NaiveProbsVector,
    ProbsVector,
    ProbsVectorType,
)

THRESHOLDING_CLASSES: Dict[str, Type[ProbsVectorType]] = {
    "no thresholding": NaiveProbsVector,
    "fixed":          ProbsVector,
    "entropy":        EntropyProbsVector,
}


@dataclass
class HriCommand:
    arity_names:   List[str]
    pv_dict:       Dict[str, ProbsVectorType]
    stamps:        Dict[str, float]                       = field(default_factory=dict)
    results_dict:  Dict[str, Any]                        = field(init=False)

    def __post_init__(self):
        # compute once, after pv_dict is set
        self.results_dict = {
            f"target_{a}": self.pv_dict[a].apply_thresholding()
            for a in self.arity_names
        }

    def __getattr__(self, name: str) -> Any:
        # dynamic target_<arity> access
        if name.startswith("target_"):
            key = name.split("target_", 1)[1]
            if key in self.pv_dict:
                return self.pv_dict[key].max
        raise AttributeError(f"{type(self).__name__!r} has no attribute {name!r}")

    def __matmul__(self, other: HriCommand) -> HriCommand:
        # modality merging
        assert self.arity_names == other.arity_names
        for a in self.arity_names:
            assert (self.pv_dict[a].names == other.pv_dict[a].names).all()

        merged = merge_probabilities(
            {a: self.pv_dict[a].p for a in self.arity_names},
            {a: other.pv_dict[a].p for a in other.arity_names},
            thresholding="entropy"
        )

        data = {
            **{f"{a}_probs":  merged[a].p
               for a in self.arity_names},
            **{f"{a}_names": self.pv_dict[a].names
               for a in self.arity_names}
        }
        return HriCommand.from_dict(self.arity_names, data, "entropy")

    @property
    def data_dict(self) -> Dict[str, List[float]]:
        return {a: self.pv_dict[a].p for a in self.arity_names}

    def to_dict(self) -> Dict[str, Any]:
        d = {"arity_names": self.arity_names}
        for a in self.arity_names:
            d[f"{a}_names"] = self.pv_dict[a].names
            d[f"{a}_probs"] = self.pv_dict[a].p
        d.update(self.results_dict)
        return d

    def to_ros(self) -> Any:
        from hri_msgs.msg import HRICommand as HRICommandMSG
        payload = json.dumps(self.to_dict())
        return HRICommandMSG(data=[payload])

    def __str__(self) -> str:
        return "\n\n".join(
            f"[[{a}]]\n{self.pv_dict[a].info()}"
            for a in self.arity_names
        )

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, HriCommand)
            and self.arity_names == other.arity_names
            and self.pv_dict    == other.pv_dict
        )

    @classmethod
    def _make_pv_dict(
        cls,
        arity_names: List[str],
        names_map: Dict[str, List[str]],
        probs_map: Dict[str, List[float]],
        thresholding: str,
    ) -> Dict[str, ProbsVectorType]:
        PV = THRESHOLDING_CLASSES.get(thresholding)
        if PV is None:
            raise ValueError(f"Unknown thresholding mode {thresholding!r}")
        return {
            a: PV(probs_map[f"{a}_probs"], names_map[f"{a}_names"])
            for a in arity_names
        }

    @classmethod
    def from_dict(
        cls,
        arity_names:   List[str],
        data_dict:     Dict[str, Any],
        thresholding:  str,
        stamps:        Optional[Dict[str, float]] = None,
    ) -> HriCommand:
        names_map = {f"{a}_names": data_dict[f"{a}_names"] for a in arity_names}
        probs_map = {f"{a}_probs": data_dict[f"{a}_probs"] for a in arity_names}
        pv_dict   = cls._make_pv_dict(arity_names, names_map, probs_map, thresholding)
        return cls(arity_names, pv_dict, stamps or {})

    @classmethod
    def from_ros(
        cls,
        msg,
        thresholding: str = "entropy"
    ) -> HriCommand:
        raw = json.loads(msg.data[0])
        arity_names = [k.split("_probs")[0] 
                       for k in raw if k.endswith("_probs")]

        # extract names, probs, and any timestamps
        names_map = {f"{a}_names": raw[f"{a}_names"] for a in arity_names}
        probs_map = {f"{a}_probs": raw[f"{a}_probs"] for a in arity_names}
        stamps    = {
            a: raw[f"target_{a}_timestamp"]
            for a in arity_names
            if f"target_{a}_timestamp" in raw
        }

        pv_dict = cls._make_pv_dict(arity_names, names_map, probs_map, thresholding)
        return cls(arity_names, pv_dict, stamps)

    def get_target_timestamped_list(self) -> List[List[Any]]:
        return [
            [self.stamps.get(a, -1.0), getattr(self, f"target_{a}")]
            for a in self.arity_names
        ]

    def get_target_timestamped_probabilistic(self) -> List[List[Any]]:
        return [
            [self.stamps.get(a, -1.0), self.pv_dict[a].dict]
            for a in self.arity_names
        ]
