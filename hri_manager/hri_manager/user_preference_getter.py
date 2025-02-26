
import yaml
import hri_manager

class UserPreferenceGetter():
    def __init__(self):
        super(UserPreferenceGetter, self).__init__()


        self.user_profile_links_dict = yaml.safe_load(open(f"{hri_manager.package_path}/links/{self.user}_links.yaml", mode='r'))

        self.A = self.user_profile_links_dict["actions"]
        self.O = self.user_profile_links_dict["objects"]
        self.user_preferred_action_words = self.user_profile_links_dict["all_action_words"]
        self.user_preferred_object_words = self.user_profile_links_dict["all_object_action_words"]

    def map_instruction_words(self, output):
        action = output['target_action'] # e.g., open
        object = output['target_object'] # e.g., drawer
        mapped_action = ""
        mapped_object = ""

        
        action = action.lower() 
        for _,link in self.user_profile_links_dict['links'].items(): 
            for action_wd in link['action_words']:
                if action_wd.lower() in action:  
                    mapped_action = link["action_template"]
                    break
            
        object = object.lower()
        for _,link in self.user_profile_links_dict['links'].items():
            for object_wd in link['object_action_words']:
                if object_wd.lower() in object: 
                    mapped_object = link["object_template"]
                    break

        return mapped_action, mapped_object

    def print_user_preferences(self):
        """ Returns What is possible to do """
        s =  f"Possible skills: {self.A}\n"
        s += f"Possible objects: {self.O}\n"
        for i,link in self.user_profile_links_dict['links'].items(): 
            s += f"Link {i}:\n"
            s += f"Action words: {link['action_words']} + Object-action words: {link['object_action_words']} | gesture: {link['action_gestures']}\n"
            s += f"     -> {link['action_template']}, {link['object_template']}\n"
        return s