"""SkillCommand: the resolved command the robot executes (post-decision).

HriCommand (hri_manager) holds pre-decision probabilistic hypotheses per
modality; a SkillCommand is the single decided command, built either from a
natural sentence or from the LLM's schema-constrained JSON. The command
string round-trips: SkillCommand(str(cmd), constraints) == cmd.

Grammar of a valid command string (words come from the user's links yaml,
`command_constraints` here):

    [speed] action                      zero-object action    "stop"
    [speed] action object               single-object action  "pick cup1"
    [speed] action object to object2    double-object action  "quickly pour cup1 to bowl1"
    [speed] action direction metric     directional action    "move left 1cm"

where speed is in constraints["adjectives"], action is in exactly one of the
arity lists (zero/single/double_object_actions, directional_actions), every
object is in constraints["objects"], direction is in
constraints["directions"], and metric is "<number><unit>" with unit in
constraints["units"]. The preposition is fixed to "to". Anything else is an
invalid command: the constructor never raises, is_valid() returns False and
invalid_reason says why. Extra trailing words are clipped ("stop cup1" ==
"stop").

Canonical fields:
    action: str          the action word ("" when none found)
    objects: list[str]   0-2 objects, ordered (object1, object2)
    parameters: dict     only set keys, from {"speed", "direction", "metric"}
"""
import json
import re

# Defaults for directional commands when the command_constraints don't specify them.
DEFAULT_DIRECTIONS = ["left", "right", "forward", "backward", "up", "down"]
DEFAULT_UNITS = ["mm", "cm", "m"]
NONE = "none"  # sentinel emitted by the model for "field does not apply"


def build_command_schema(command_constraints: dict) -> dict:
    """JSON schema for vLLM guided decoding: the model may only pick allowed
    values per field, so the reply is always valid, schema-conforming JSON
    (no reasoning preamble, no free text). Rendered to a command by
    `SkillCommand.from_structured`."""
    cc_ = command_constraints

    def enum(values):
        # de-duplicate while preserving order, always allow "none"
        return {"type": "string", "enum": list(dict.fromkeys([*values, NONE]))}

    objects = cc_.get("objects", [])
    return {
        "type": "object",
        "properties": {
            "action": enum(cc_.get("actions", [])),
            "speed": enum(cc_.get("adjectives", [])),
            "object1": enum(objects),
            "relation": enum(cc_.get("prepositions", ["to"])),
            "object2": enum(objects),
            "direction": enum(cc_.get("directions", DEFAULT_DIRECTIONS)),
            "distance": {"type": "string"},  # a number as text, e.g. "1", "0.5", or "none"
            "unit": enum(cc_.get("units", DEFAULT_UNITS)),
        },
        "required": ["action", "speed", "object1", "relation", "object2",
                     "direction", "distance", "unit"],
        "additionalProperties": False,
    }


class SkillCommand():
    def __init__(self,
                 sentence: str,  # natural command string, see module docstring grammar
                 command_constraints: dict,  # the user's vocabulary (links yaml)
                 reasoning_text: str = "",  # (optional) raw LLM response before parsing
                 ):
        self.reasoning_text = reasoning_text
        self.command_constraints = command_constraints
        self.action = ""
        self.objects = []
        self.parameters = {}
        self.invalid_reason = ""  # set by is_valid()

        tokens = sentence.lower().split()
        if tokens and tokens[0] in command_constraints.get("adjectives", []):
            self.parameters["speed"] = tokens.pop(0)
        if not tokens:
            return
        self.action = tokens.pop(0)

        cc_ = command_constraints
        # Consume per the action's arity; extra trailing tokens are clipped,
        # missing ones leave a partial command (is_valid() False).
        if self.action in cc_.get("directional_actions", []):
            if tokens[0:1]:
                self.parameters["direction"] = tokens[0]
            if tokens[1:2]:
                self.parameters["metric"] = tokens[1]
        elif self.action in cc_.get("single_object_actions", []):
            self.objects = tokens[0:1]
        elif self.action in cc_.get("double_object_actions", []):
            self.objects = tokens[0:1] + tokens[2:3]  # tokens[1] is the "to"

    @classmethod
    def from_structured(cls, structured: dict, command_constraints, reasoning_text: str = ""):
        """Deterministically render a schema-conforming dict (see
        `build_command_schema`) into a SkillCommand. The model only picks
        allowed values; the command string is derived here."""
        def clean(v):
            if v is None:
                return ""
            v = str(v).strip().lower()
            return "" if v in (NONE, "null", "") else v

        cmd = cls("", command_constraints, reasoning_text or json.dumps(structured))
        cmd.action = clean(structured.get("action"))
        cmd.objects = [o for o in (clean(structured.get("object1")),
                                   clean(structured.get("object2"))) if o]
        if clean(structured.get("speed")):
            cmd.parameters["speed"] = clean(structured.get("speed"))
        if clean(structured.get("direction")):
            cmd.parameters["direction"] = clean(structured.get("direction"))
        distance, unit = clean(structured.get("distance")), clean(structured.get("unit"))
        if distance:
            cmd.parameters["metric"] = f"{distance}{unit}"
        return cmd

    @property
    def command(self) -> str:
        """The canonical command string (round-trips through the constructor)."""
        if not self.action:
            return ""
        words = [self.parameters.get("speed", ""), self.action]
        if "direction" in self.parameters:
            words += [self.parameters["direction"], self.parameters.get("metric", "")]
        elif len(self.objects) == 2:
            words += [self.objects[0], "to", self.objects[1]]
        else:
            words += self.objects
        return " ".join(w for w in words if w)

    def to_dict(self) -> dict:
        """Self-contained wire payload (published on SKILL_COMMAND_TOPIC)."""
        return {
            "action": self.action,
            "objects": self.objects,
            "parameters": self.parameters,
            "command": self.command,
        }

    def is_valid(self) -> bool:
        """True when the command conforms to the grammar in the module
        docstring; otherwise False with the reason in self.invalid_reason."""
        cc_ = self.command_constraints
        reasons = []

        if not self.action:
            reasons.append("no action")
        elif self.action in cc_.get("directional_actions", []):
            if self.objects:
                reasons.append("directional action takes no objects")
            if self.parameters.get("direction") not in cc_.get("directions", DEFAULT_DIRECTIONS):
                reasons.append(f"unknown direction {self.parameters.get('direction')!r}")
            units = "|".join(cc_.get("units", DEFAULT_UNITS))
            if not re.fullmatch(rf"\d+(\.\d+)?({units})", self.parameters.get("metric", "")):
                reasons.append(f"metric {self.parameters.get('metric')!r} is not <number><{units}>")
        else:
            arity = None
            for n, key in enumerate(["zero_object_actions", "single_object_actions", "double_object_actions"]):
                if self.action in cc_.get(key, []):
                    arity = n
            if arity is None:
                reasons.append(f"unknown action {self.action!r}")
            elif len(self.objects) != arity:
                reasons.append(f"{self.action!r} needs {arity} object(s), got {len(self.objects)}")
            for obj in self.objects:
                if obj not in cc_.get("objects", []):
                    reasons.append(f"unknown object {obj!r}")

        speed = self.parameters.get("speed")
        if speed is not None and speed not in cc_.get("adjectives", []):
            reasons.append(f"unknown speed {speed!r}")

        self.invalid_reason = "; ".join(reasons)
        return not reasons

    def __str__(self):
        return self.command

    def __eq__(self, other):
        return self.command == getattr(other, "command", None)
