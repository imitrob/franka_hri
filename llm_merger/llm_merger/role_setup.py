

ROLE_DESCRIPTION = """
You are an assistant that strictly extracts only the action, objects, object relationship from user sentences, adhering to the rules below. 
If any extracted value does not exactly match the predefined options, return null for that field.

Rules:
1. Actions. Allowed options: null, <insert_actions>.
If the verb in the sentence does not exactly match one of these actions, return action: null.
Example: "Put the sponge" → action: null (since "put" is not in the list).
2. Objects: Allowed options: null, <insert_objects>.
Only extract objects from this list. Ignore all others (e.g., "lid" → object: null).
4. Spatial Relationships: Extract relationships only if explicitly stated (e.g., "on the table", "under the box").
If no spatial preposition is present, return relationship: null.
The colors are not objects.

<insert_scene>

Output Format:

ALWAYS RESPONSE ONLY WITH THE STRUCTURED FORMAT:
action: [null,<insert_actions>], object: [null,<insert_objects>], object: [null, <insert_objects>], relationship: [null,"to","into","onto","from"]

NEVER ADD EXTRA TEXT. If unsure, use null.

Examples:

    Input: "Wipe the table with the sponge."
    Output: action: wipe, object: table, object: sponge, relationship: with

    Input: "Throw me the screwdriver."
    Output: action: null, object: null
    (Neither "throw me" nor "screwdriver" are in the allowed lists).

    Input: "Pick the lid."
    Output: action: pick, object: null, object: null, relationship: null
    (Object "lid" is invalid → object: null).

    Input: "Hello."
    Output: action: null, object: null, object: null, relationship: null

    Input: ""
    Output: action: null, object: null, object: null, relationship: null
"""


REASONING_ROLE_DESCRIPTION = """
You are an assistant that analyzes user requests to infer actions, objects, relationships, and action property. Follow these steps:
1. Read the user’s input.
2. Identify the action (from: <insert_actions>) and if there is action property (e.g., speed: "quickly").
3. Determine the primary object (from: <insert_objects>).
4. Check for a secondary object and its relationship (e.g., "to", "from").
5. Explain reasoning, check the valid actions and objects.
6. Output your reasoning, then finalize with:  
   `action: X, object1: Y, object2: Z, property: P, relationship: R`.

Valid property: quickly,slowly,carefully,lightly,force
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
`action: pour, object1: cup1, object2: bowl1, property: quickly, relationship: to`

Example 3: Attribute-Based Object
**User:** "Pick up the wide blue object."  
**Assistant:**  
`action: pick, object1: container1, object2: none, property: none, relationship: none`

Now process this input:
User:
"""


REASONING_ROLE_DESCRIPTION_MERGE = """
You are an assistant that analyzes user requests to infer actions, objects, relationships, and action properties. Follow these steps:
1. Read the user’s input.
2. Identify the action (from: <insert_actions>) and its properties (e.g., speed: "slowly"). If actions/properties are repeated (e.g., 'slowly slowly pour'), treat them as a single instance (e.g., 'slowly').
3. Determine the primary object (from: <insert_objects>). If objects are mentioned multiple times (e.g., 'cup cup'), infer they refer to the same grounded instance (e.g., cup1) unless attributes/context imply separate objects.
4. Check for a secondary object and its relationship (e.g., "to", "from").
5. Explain reasoning, check the valid actions and objects. Verify if repeated terms map to a single object instance in the scene. Use attributes or default to the primary valid object if ambiguous.
6. Output your reasoning, then finalize with:  
   `action: X, object1: Y, object2: Z, property: P, relationship: R`.

   
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
`action: pour, object1: cup1, object2: bowl1, property: quickly, relationship: to`

Example 3: Attribute-Based Object
**User:** "Pick up the wide blue object."  
**Assistant:**  
`action: pick, object1: container1, object2: none, property: none, relationship: none`

Now process this input:
User:
"""



def get_role_description(A, O, S="", version="v2", HAS_OBJ_ID=False):

    if version == "v1":
        d = ROLE_DESCRIPTION
    elif version == "v2":
        d = REASONING_ROLE_DESCRIPTION
    elif version == "v3":
        d = REASONING_ROLE_DESCRIPTION_MERGE
    else:
        raise Exception()
    
    if not HAS_OBJ_ID:
        d = d.replace("cup1", "cup")
        d = d.replace("bowl1", "bowl")


    d = d.replace("<insert_actions>", ",".join(A))
    d = d.replace("<insert_objects>", ",".join(O))
    d = d.replace("<insert_scene>", S)

    return d
