from copy import deepcopy
# THESE ARE USED TO DISCARD THINGS FROM SkillCommand 
COLORS = ["green", "blue", "red", "pink", "yellow", "black", "orange"]
RELATIONS = ["to", "into", "onto", "from"]
from naive_merger.utils import cc

SKILL_COMMAND_TEMPLATE = {
    "property": "", 
    "target_action": "",
    "relationship": "", 
    "target_object": "", 
    "target_object2": "", 
    "target_object_color": "",
    "direction": "",
    "metric": "",
}

class SkillCommand():
    def __init__(self,
                r: dict | str, # dict or str 
                command_constraints: dict, # The list of actions and is used for filtering the valid
                reasoning_text: str = "", # (Optional) The entire raw response before parsing into the command.
        ):
        self.reasoning_text = reasoning_text
        self.command_constraints=command_constraints

        self.target_action = None
        self.target_direction = None
        self.target_action_metric = None
        self.target_object = None
        self.object_preposition = None
        self.target_object2 = None
        self.action_parameter = None
        if isinstance(r, str):
            r = r.split(" ")
            
            if r[0] in command_constraints["adjectives"]:
                self.action_parameter = r[0]
                self.command = f"{r[0]} "
                add = 1
            else:
                self.command = f""
                add = 0

            try:
                if r[0 + add] in command_constraints["directional_actions"]:
                    self.target_action = r[0 + add]
                    self.target_direction = r[1 + add]
                    self.target_action_metric = r[2 + add]
                    self.command += f"{r[0 + add]} {r[1 + add]} {r[2 + add]}"
                elif r[0 + add] in command_constraints["zero_object_actions"]:
                    self.target_action = r[0 + add]
                    self.command += f"{r[0 + add]}"
                elif r[0 + add] in command_constraints["single_object_actions"]:
                    self.target_action = r[0 + add]
                    self.target_object = r[1 + add]
                    self.command += f"{r[0 + add]} {r[1 + add]}"
                elif r[0 + add] in command_constraints["double_object_actions"]:
                    self.target_action = r[0 + add]
                    self.target_object = r[1 + add]
                    self.object_preposition = r[2 + add]
                    self.target_object2 = r[3 + add]
                    self.command += f"{r[0 + add]} {r[1 + add]} {r[2 + add]} {r[3 + add]}"
                else: 
                    print("No known action", r[0 + add], " not in ", command_constraints["actions"])
                    self.command = "" 
            except IndexError:
                pass # This SkillCommand is not valid and function is_valid() -> False

        elif isinstance(r, dict):
            if r['property'] in command_constraints["adjectives"]:
                self.action_parameter = r['property']
                self.command = f"{r['property']} "
            else:
                self.command = f""
            
            if r["target_action"] in command_constraints["directional_actions"]:
                self.target_action = r["target_action"]
                self.target_direction = r['direction']
                self.target_action_metric = r['metric']
                self.command += f"{r['target_action']} {r['direction']} {r['metric']}"
            elif r["target_action"] in command_constraints["zero_object_actions"]:
                self.target_action = r["target_action"]
                self.command += f"{r['target_action']}"
            elif r["target_action"] in command_constraints["single_object_actions"]:
                self.target_action = r["target_action"]
                self.target_object = r['target_object']
                self.command += f"{r['target_action']} {r['target_object']}"
            elif r["target_action"] in command_constraints["double_object_actions"]:
                self.target_action = r["target_action"]
                self.target_object = r['target_object']
                self.object_preposition = r['relationship']
                self.target_object2 = r['target_object2']
                self.command += f"{r['target_action']} {r['target_object']} {r['relationship']} {r['target_object2']}"
            else: 
                print("No known action", r['target_action'], " not in ", command_constraints["actions"])
                self.command = "" 
        else: raise Exception()

    @property
    def target_storage(self):
        return self.target_object2

    def __str__(self):
        return self.command

    def __eq__(self, other):
        try:
            if self.command == other.command:
                print(f"{cc.H}{self.command} == {other.command}{cc.E}")
                return True
            else:
                print(f"{cc.F}{self.command} != {other.command}{cc.E}")
                return False
        except AttributeError:
            return False

    @classmethod
    def from_predicted(cls, predicted_text, command_constraints):
        """ Parsing the LLM string output to `brackets`. """
        
        reasoning_text = deepcopy(predicted_text)
        response = predicted_text.lower()
        
        """ response is raw string output from LLM, all format correction is here """
        
        # Parse Option 1: Output as: <reasoning> ```plaintext <result>```
        if "```plaintext" in response: # 

            response = response.split("```")
            response = response[-2]
            response = response.replace("\n", "")
            response = response.strip()
            response = response.split("plaintext")[-1]
        # Parse Option 2: Output as: <reasoning> ```<result>```
        if "```" in response:
            
            response = response.split("```")
            response = response[-2]
        # Parse Option 3: Output as: <reasoning> `<result>`
        elif "`" in response:
            
            response = response.split("`")
            if len(response) == 2:
                response = response[-1]
            else:
                try:
                    if "action:" in response[-2]: # Output as: <reasoning> `action: <action>, <item2>: <val2>, ...`
                        response = response[-2]

                    elif "action:" in response[-3]: # Output as: <reasoning> `action: <action>, <item2>: <val2>, ...` `<something useless>`
                            response = response[-3]

                    elif "action:" in response[-4]: # Output as: <reasoning> `action: <action>, <item2>: <val2>, ...` `<useless1> <useless2>`
                            response = response[-4]

                    elif "action:" in response[-5]: # Output as: <reasoning> `action: <action>, <item2>: <val2>, ...` `<useless1> <useless2> <useless3>`
                            response = response[-5]

                    elif "action:" in response[-6]:
                            response = response[-6]
                    else:
                        return cls(deepcopy(SKILL_COMMAND_TEMPLATE), command_constraints, reasoning_text)
                except IndexError: 
                    return cls(deepcopy(SKILL_COMMAND_TEMPLATE), command_constraints, reasoning_text)

        # Parse Option 4: Output as: <reasoning> **action:** <action>, **<item2>**: <val2>, ...
        elif "**action:" in response:
            if "**action:**" in response:
                response = response.split("**action:")[-1]
                response = "**action:" + response
                response = response.replace("*", "")
                response = response.replace("  \n", ",")
                response = response.replace(" \n", ",")
                response = response.replace("\n", ",")
            else:
                response = response.split("**")
                response = response[-2]
        # Parse Option 5: Output as: <reasoning> \n action: <action>, <item2>: <val2>, ...
        else:
            response = response.split("\n")[-1].strip()
            
        response = response.lower()
        response = response.replace("`", "")
        response = response.replace(", ", ",")
        response = response.replace("'", "")
        response = response.replace("a can", "can")
        response_list = response.split(",")

        r = deepcopy(SKILL_COMMAND_TEMPLATE)
        for i in range(len(response_list)):
            s = remove_article(response_list[i]) # get rid of a, the..
            
            k, v = remove_types(s) # get rid of "action: ..."
            if k is None: continue
            v = v.split(" ")[-1]

            if v != "none" and v != "unknown" and v != "none specified" and v != "**none**":
                if r[k] != "": # order from model is: object, object2 right after each other; color, color2
                    r[k+"2"] = v
                else:
                    r[k] = v

        return cls(r, command_constraints, reasoning_text)

    def is_valid(self):
        if self.command == "": return False

        not_valid = ""
        # Check format
        if not isinstance(self.action_parameter, str) and self.action_parameter is not None: not_valid += f"{self.action_parameter} is not str or None"
        if not isinstance(self.target_action, str) and self.target_action is not None: not_valid += f"{self.target_action} is not str or None"
        if not isinstance(self.target_object, str) and self.target_object is not None: not_valid += f"{self.target_object} is not str or None"
        if not isinstance(self.target_object2, str) and self.target_object2 is not None: not_valid += f"{self.target_object2} is not str or None"
        if not isinstance(self.object_preposition, str) and self.object_preposition is not None: not_valid += f"{self.object_preposition} is not str or None"

        # REMOVED TEMPORARILY AS THIS IS NOT VALID ANYMORE
        # if self.target_object is not None:
        #     if self.target_object[-1] not in "0123456789":
        #         not_valid += "object must have its id as last char"
        if self.object_preposition is not None:
            if self.target_object2 is None:
                not_valid += "object must have its id as last char"
        # if self.target_object2 is not None:
        #     if self.target_object2[-1] not in "0123456789":
        #         not_valid += "object must have its id as last char"

        if self.target_action in self.command_constraints["zero_object_actions"]:
            if self.target_object is not None or self.target_object2 is not None:
                not_valid += "Not correct object number defined"
        elif self.target_action in self.command_constraints["single_object_actions"]:
            if self.target_object is None or self.target_object2 is not None:
                not_valid += "Not correct object number defined"
        elif self.target_action in self.command_constraints["double_object_actions"]:
            if self.target_object is None or self.target_object2 is None:
                not_valid += "Not correct object number defined"
        elif self.target_action in self.command_constraints["directional_actions"]:
            if self.target_object is not None or self.target_object2 is not None:
                not_valid += "Not correct object number defined"
        else: raise Exception("Action not in list!")

        if self.target_object is not None and self.target_object2 is not None:
            if self.object_preposition is None:
                not_valid += "Double object action and no preposition"

        if not_valid == "":
            return True
        else:
            print(not_valid)
            return False

def remove_types(str):
    if "action: " in str.strip():
        str = str.split("action: ")[-1]
        # str = remove_relation(str)
        return "target_action", str
    if "action:" in str.strip():
        str = str.split("action:")[-1]
        # str = remove_relation(str)
        return "target_action", str
    if "object: " in str:
        str = str.split("object: ")[-1]
        str = remove_color(str)
        return "target_object", str
    if "object:" in str:
        str = str.split("object:")[-1]
        str = remove_color(str)
        return "target_object", str
    if "object1: " in str:
        str = str.split("object1: ")[-1]
        str = remove_color(str)
        return "target_object", str
    if "object2: " in str:
        str = str.split("object2: ")[-1]
        str = remove_color(str)
        return "target_object", str
    if "color: " in str:
        str = str.split("color: ")[-1]
        return "target_object_color", str
    if "color:" in str:
        str = str.split("color:")[-1]
        return "target_object_color", str
    if "relationship: " in str:
        str = str.split("relationship: ")[-1]
        # TODO:
        if "to" in str: str = "to"
        return "relationship", str
    if "relationship:" in str:
        str = str.split("relationship:")[-1]
        # TODO:
        if "to" in str: str = "to"
        return "relationship", str
    if "property: " in str:
        str = str.split("property: ")[-1]
        return "property", str
    if "property:" in str:
        str = str.split("property:")[-1]
        return "property", str
    
    if "direction: " in str:
        str = str.split("direction: ")[-1]
        return "direction", str
    if "direction:" in str:
        str = str.split("direction:")[-1]
        return "direction", str

    if "metric: " in str:
        str = str.split("metric: ")[-1]
        return "metric", str
    if "metric:" in str:
        str = str.split("metric:")[-1]
        return "metric", str


    print(f"!!! Either 'action:', 'object:', 'color: ' or 'relationship': in string {str}")
    return None, None

def remove_article(str):
    if str[0:2] == "a ":
        str = str.replace("a ", "")
    if str[0:4] == "the ":
        str = str.replace("the ", "")
    return str

def remove_color(str):
    ''' Sometimes, model puts color to object, this is a workaround '''
    for color in COLORS:
        if color in str:
            str = str.replace(color+" ", "") # "blue box" -> "box"
            str = str.replace(color, "") # "blue" -> "", does nothing if not found
    return str

def remove_relation(str):
    ''' Sometimes, model puts relation into an action, this is a workaround '''
    for relation in RELATIONS:
        if relation in str:
            str = str.replace(" "+relation, "") # "blue box" -> "box"
            str = str.replace(relation, "") # "blue" -> "", does nothing if not found
    return str
