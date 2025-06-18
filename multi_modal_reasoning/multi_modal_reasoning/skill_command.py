from copy import deepcopy
# THESE ARE USED TO DISCARD THINGS FROM SkillCommand 
COLORS = ["green", "blue", "red", "pink", "yellow", "black", "orange"]
RELATIONS = ["to", "into", "onto", "from"]
from naive_merger.utils import cc
class SkillCommand():
    def __init__(self, command: str = "", predicted: str = "", command_constraints={}):
        self.command = command
        self.predicted = predicted
        self.command_constraints=command_constraints

    def has_action_parameter(self):
        l = self.command.split(" ")
        if l[0] in self.command_constraints["actions"]: 
            return False
        elif l[1] in self.command_constraints["actions"]:
            return True
        else:
            raise Exception()

    @property
    def action_parameter(self):
        if self.has_action_parameter():
            return self.command.split(" ")[0]
        else:
            return None

    @property
    def target_action(self):
        if self.has_action_parameter():
            return self.command.split(" ")[1]
        else:
            return self.command.split(" ")[0]
        
    @property
    def target_object(self):
        try:
            if self.has_action_parameter():
                return self.command.split(" ")[2]
            else:
                return self.command.split(" ")[1]
        except IndexError:
            return None

    @property
    def object_preposition(self):
        try:
            if self.has_action_parameter():
                return self.command.split(" ")[3]
            else:
                return self.command.split(" ")[2]
        except IndexError:
            return None

    @property
    def target_storage(self):
        return self.target_object2
    @property
    def target_object2(self):
        try:
            if self.has_action_parameter():
                return self.command.split(" ")[4]
            else:
                return self.command.split(" ")[3]
        except IndexError:
            return None

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
    def from_predicted(cls, response, command_constraints):
        predicted = deepcopy(response)
        response = response.lower()
        """ response is raw string output from LLM, all format correction is here """
        if "```plaintext" in response:
            response = response.split("```")
            response = response[-2]
            response = response.replace("\n", "")
            response = response.strip()
            response = response.split("plaintext")[-1]
        if "```" in response:
            response = response.split("```")
            response = response[-2]
        elif "`" in response:
            response = response.split("`")
            if len(response) == 2:
                response = response[-1]
            else:
                try:
                    if "action:" in response[-2]:
                        response = response[-2]
                    elif "action:" in response[-3]:
                            response = response[-3]
                    elif "action:" in response[-4]:
                            response = response[-4]
                    elif "action:" in response[-5]:
                            response = response[-5]
                    elif "action:" in response[-6]:
                            response = response[-6]
                    else:
                        return cls("", predicted, command_constraints)
                except IndexError: 
                    return cls("", predicted, command_constraints)

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
        else:
            response = response.split("\n")[-1].strip()
            
        response = response.lower()
        response = response.replace("`", "")
        response = response.replace(", ", ",")
        response = response.replace("'", "")
        response = response.replace("a can", "can")
        response_list = response.split(",")
        r = {"property": "", "target_action": "", "relationship": "", "target_object": "", "target_object2": "", "target_object_color": ""}
        
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

        if r['property'] not in command_constraints["adjectives"]:
            r["property"] = ""

        if r['target_action'] in command_constraints["zero_object_actions"]:
            return cls(f"{r['property']} {r['target_action']}".strip(), predicted, command_constraints)
        elif r['target_action'] in command_constraints["single_object_actions"]:
            return cls(f"{r['property']} {r['target_action']} {r['target_object']}".strip(), predicted, command_constraints)
        elif r['target_action'] in command_constraints["double_object_actions"]:
            return cls(f"{r['property']} {r['target_action']} {r['target_object']} {r['relationship']} {r['target_object2']}".strip(), predicted, command_constraints)
        else: 
            print("No known action", r['target_action'], " not in ", command_constraints["actions"])
            return cls("", predicted, command_constraints)

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
