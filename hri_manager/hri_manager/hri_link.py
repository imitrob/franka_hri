
import trajectory_data, object_localization, hri_manager, yaml
from pathlib import Path
import hri_manager

class Link():
    def __init__(self,
                 user_name, 
                 action_template, 
                 object_template, 
                 action_words, 
                 object_words,
                 action_gestures,
                 ):
        self.user_name = user_name 
        self.action_template = action_template 
        self.object_template = object_template 
        self.action_words = action_words 
        self.object_words = object_words
        self.action_gestures = action_gestures

    def save(self):
        new_link = {
            "user": self.user_name,
            "action_template": self.action_template, # e.g., push
            "object_template": self.object_template, # e.g., cube_template - cube
            "action_words": list(self.action_words), # e.g., ["push"]
            "object_words": list(self.object_words),
            "action_gestures": self.action_gestures, # e.g., "grab" + "swipe right"
        }
        try:
            data_dict = yaml.safe_load(open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="r"))
        except FileNotFoundError:
            data_dict = {
                    "user": self.user_name,
                    "directional_actions": [],
                    "zero_object_actions": [],
                    "single_object_actions": [],
                    "double_object_actions": [],
                    "objects": [],
                    "links": {},
                }
            with open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="w") as file:
                yaml.safe_dump(data_dict, file, sort_keys=False)

        # Register the action in the arity lists if not present. The overall
        # `actions` list is derived from these (see user_links.py). A taught
        # link pairs the action with one object template -> single-object.
        known_actions = (data_dict.get("directional_actions", [])
                         + data_dict.get("zero_object_actions", [])
                         + data_dict.get("single_object_actions", [])
                         + data_dict.get("double_object_actions", []))
        if new_link["action_template"] not in known_actions:
            data_dict.setdefault("single_object_actions", []).append(new_link["action_template"])

        # Add the action_template to actions list if not present
        if new_link["object_template"] not in data_dict["objects"]:
            data_dict["objects"].append(new_link["object_template"])

        # Add the new link to links with a unique key
        new_link_name = f"link{len(data_dict['links']) + 1}"  # Generate unique link name
        data_dict["links"][new_link_name] = new_link
        

        print("data_dict", data_dict)
        with open(f"{hri_manager.package_path}/links/{self.user_name}_links.yaml", mode="w") as file:
            yaml.safe_dump(data_dict, file, sort_keys=False)

    def check_valid(self,hri):
        #does_action_exist(action_template)
        
        action_template_path = f"{trajectory_data.package_path}/trajectories/{self.action_template}.npz"
        assert Path(action_template_path).is_file(), f"{action_template_path} doesn't exists!"

        # does_object_template_exist(object_template)
        object_template_path = f"{object_localization.package_path}/cfg/{self.object_template}"
        assert Path(object_template_path).is_dir(), f"{object_template_path} doesn't exists!"

        # does_action_gesture_exist(action_gesture)
        for action_gesture in self.action_gestures:
            assert action_gesture[0] in hri.gestures.Gs_static, f"{action_gesture[0]} is not in {hri.gestures.Gs_static}"
            assert action_gesture[1] in hri.gestures.Gs_dynamic, f"{action_gesture[1]} is not in {hri.gestures.Gs_dynamic}" 
