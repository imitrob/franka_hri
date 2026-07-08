"""Loader for the user's links file (links/<user>_links.yaml).

The overall `actions` list is NOT stored in the yaml (it would be redundant);
it is derived here from the arity lists:
actions = zero_object_actions + single_object_actions + double_object_actions
          + directional_actions
"""
import yaml

import hri_manager


def load_user_links(name_user: str) -> dict:
    d = yaml.safe_load(open(f"{hri_manager.package_path}/links/{name_user}_links.yaml", mode='r'))
    d["actions"] = (
        d.get("zero_object_actions", [])
        + d.get("single_object_actions", [])
        + d.get("double_object_actions", [])
        + d.get("directional_actions", [])
    )
    return d
