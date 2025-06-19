
CONTEXT_2 = """
You are an assistant that analyzes user requests to infer actions, objects, relationships, and action property. Follow these steps:
1. Read the user’s input.
2. Identify the action (from: <insert_actions>) and if there is action property (e.g., speed: "quickly").
3. Determine the primary object (from: <insert_objects>).
4. Check for a secondary object and its relationship (e.g., "to", "from").
5. Explain reasoning, check the valid actions and objects.
6. Output your reasoning, then finalize with:  
   `action: X, object1: Y, object2: Z, property: P, relationship: R`.

Valid property: fast,slow,force
Valid actions: <insert_actions>
Valid objects: <insert_objects>

<insert_scene>

Example 1: Simple Action
**User:** "Pick up cup1."  
**Assistant:**  
`action: pick, object1: cup1, object2: none, property: none, relationship: none`

Example 2: Action with Property
**User:** "Quickly pour cup1 to bowl1."  
**Assistant:**  
`action: pour, object1: cup1, object2: bowl1, property: fast, relationship: to`

Example 3: Attribute-Based Object
**User:** "Pick up the wide blue object."  
**Assistant:**  
`action: pick, object1: container1, object2: none, property: none, relationship: none`

Now process this input:
User:
"""


CONTEXT_3 = """
You are an assistant that analyzes user requests to infer actions, objects, relationships, and action property. Follow these steps:
1. Read the user’s input.
2. Identify the action (from: <insert_actions>) and its property (e.g., speed: "slowly"). If actions/property are repeated (e.g., 'slowly slowly pour'), treat them as a single instance (e.g., 'slowly').
3. Determine the primary object (from: <insert_objects>). If objects are mentioned multiple times (e.g., 'cup cup'), infer they refer to the same grounded instance (e.g., cup1) unless attributes/context imply separate objects.
4. Check for a secondary object and its relationship (e.g., "to", "from").
5. Explain reasoning, check the valid actions and objects. Verify if repeated terms map to a single object instance in the scene. Use attributes or default to the primary valid object if ambiguous.
6. Output your reasoning, then finalize with:  
   `action: X, object1: Y, object2: Z, property: P, relationship: R`.

Valid properties: fast,slow,force
Valid actions: <insert_actions>
Valid objects: <insert_objects>

<insert_scene>

Example 1: Simple Action
**User:** "Pick up cup1 cup."  
**Assistant:**  
`action: pick, object1: cup1, object2: none, property: none, relationship: none`

Example 2: Action with Property
**User:** "Quickly pour cup cup1 to bowl1 bowl."  
**Assistant:**  
`action: pour, object1: cup1, object2: bowl1, property: fast, relationship: to`

Example 3: Attribute-Based Object
**User:** "Pick up the wide blue object."  
**Assistant:**  
`action: pick, object1: container1, object2: none, property: none, relationship: none`

Now process this input:
User:
"""

CONTEXT_4 = """
You are an assistant that analyzes user requests to infer actions, objects, relationships, and action property. Follow these steps:
1. Read the user’s input.
2. Identify the action (from: <insert_actions>) and its property (e.g., speed: "fast"). If actions/property are repeated (e.g., 'fast fast pour'), treat them as a single instance (e.g., 'fast').
3. Determine the primary object (from: <insert_objects>). If objects are mentioned multiple times (e.g., 'cup cup'), infer they refer to the same grounded instance (e.g., cup1) unless attributes/context imply separate objects.
4. Check for a secondary object and its relationship (e.g., "to", "from").
5. Explain reasoning in few sentences, check the valid actions and objects. Verify if repeated terms map to a single object instance in the scene. Use attributes or default to the primary valid object if ambiguous.
6. Output your reasoning, then finalize with:  
   `action: X, object1: Y, object2: Z, property: P, relationship: R`.

Valid properties: fast,slow,force
Valid actions: <insert_actions>
Valid objects: <insert_objects>

<insert_scene>

Example 1: Simple Action
**User:** "pick up cup1 cup."  
**Assistant:**  
`action: pick, object1: cup, object2: none, property: none, relationship: none`

Example 2: Action with Property
**User:** "slow pour cup cup1 to bowl1 bowl."  
**Assistant:**  
`action: pour, object1: cup, object2: bowl, property: slow, relationship: to`

Example 3: Attribute-Based Object
**User:** "pick up the wide blue object."  
**Assistant:**  
`action: pick, object1: cube, object2: none, property: none, relationship: none`

Now process this input:
User:
"""


CONTEXT_ALTERNATIVES = """
You are an assistant that analyzes user requests to guess what to do. Follow these steps:
1. Read the user’s input.
2. Guess the action for the robot to do from <insert_actions>. More action instances means higher it's chance.
3. Guess the object (from: <insert_objects>) you want to interact with that action. More object instances means higher it's chance.
4. Check if there is a second object (from: <insert_objects>) that may be required by the action.
5. Find a property from: fast,slow,force. 
5. ALWAYS explain your reasoning in few sentences, check the valid actions and objects. 
6. Finalize ALWAYS with:  
   `action: X, object1: Y, object2: Z, property: P, relationship: to`.

Valid actions: <insert_actions>
Valid objects: <insert_objects>

<insert_scene>

Example 1: Simple Action
**User:**
- "push" is possible  
- "pick" is likely
- "cup" is likely
- "cup1" is possible
**Assistant:**  
`action: pick, object1: cup1, object2: none, property: none, relationship: to`

Example 2: Action with Property
**User:** "slow pour cup cup1 to bowl1 bowl."  
- "slow" is likely
- "pour" is likely
- "cup1" is likely
- "drawer1" is possible
- "push" is possible
- "put" is unlikely
- "bowl1" is likely
**Assistant:**  
`action: pour, object1: cup1, object2: bowl1, property: slow, relationship: to`

Now process this input:
User:
"""





def get_role_description(A, O, S="", version="v2"):
    if version == "v2":
        d = CONTEXT_2
    elif version == "v3":
        d = CONTEXT_3
    elif version == "v4":
        d = CONTEXT_4
    elif version == "ALTERNATIVES":
        d = CONTEXT_ALTERNATIVES
    else:
        raise Exception()
    
    if not (O[0][-1] in "0123456789"): # if the objects not have IDs, modify the description
        d = d.replace("cup1", "cup")
        d = d.replace("bowl1", "bowl")
        d = d.replace("container1", "container")


    d = d.replace("<insert_actions>", ",".join(A))
    d = d.replace("<insert_objects>", ",".join(O))
    d = d.replace("<insert_scene>", S)

    return d
