"""System-prompt (role) templates for the reasoning LLM.

Only the structured-output roles remain; they instruct the model to emit JSON
constrained by build_command_schema (see skill_command.py). Placeholders
<insert_actions>, <insert_objects>, <insert_scene> are filled by
get_role_description.
"""

STRUCTURED = """
You convert a user's spoken and gestured command into a structured robot command.
The words are a time-ordered mix of speech and a pointed-at (gestured) object; the
gestured object tells you which object is meant when the speech is vague.

Output JSON ONLY, filling exactly these fields:
{"action","speed","object1","relation","object2","direction","distance","unit"}

Allowed values:
- action: <insert_actions> | none
- speed: fast | slow | force | none
- object1, object2: <insert_objects> | none
- relation: to | none
- direction: left | right | forward | backward | up | down | none (used only by "move")
- distance: a number written as text (e.g. "1", "2", "0.5") or "none" (used only by "move")
- unit: mm | cm | m | none (used only by "move")

<insert_scene>

Rules:
- ALWAYS set action to the allowed verb that appears, even amid extra or noisy words, and no matter its
  position. If SEVERAL verbs appear and only some are allowed, choose the ALLOWED one and ignore the rest.
- The verb is often paraphrased: when no allowed verb appears literally, choose the allowed action
  closest in meaning to what was said (e.g. give/hand/take/grab/grasp/get/lift/hold -> pick;
  shove/slide/press/nudge -> push; fill/tip/empty/spill/dump -> pour;
  tap/poke/feel/pat/contact -> touch). Use action="none" ONLY when nothing
  resembling an action is said.
- object1 is the main target. Resolve it in this order: the pointed-at (gestured) object when speech is
  vague ("this", "it", "object"); else the object matching a stated color/attribute; else the named object
  type. Map a described object to its scene instance (e.g. the red one -> cup1, the bowl -> bowl1).
  object2 is the destination, used only by "pour".
- Object names are also paraphrased: map an object word to the closest allowed object
  (e.g. mug/glass -> cup, dish/basin -> bowl, block/brick -> cube, cabinet/compartment -> drawer,
  fruit -> banana, carton/crate/package -> box). NEVER output an object that is not allowed.
- Do NOT guess: when the speech is vague ("this", "it") and there is no gesture and no attribute
  that identifies the object, set object1 (or object2) to "none".
- A repeated object ("cup cup") or a type + instance ("bowl bowl1") refers to a single instance.
- Normalize: quickly->fast, slowly->slow, forcefully/hard->force.
- "move" is ONLY the robot motion command ("move left one centimeter"): extract direction, and the
  distance as a number with its unit; objects stay "none". Normalize number words to digits
  (one->1, two->2, "zero point five"->0.5) and units: centimeter(s)->cm, millimeter(s)->mm,
  meter(s)->m. Moving/sliding/shoving an OBJECT is "push", not "move".
- Every field that does not apply must be "none".

Examples:
User: push it over here cup1
JSON: {"action":"push","speed":"none","object1":"cup1","relation":"none","object2":"none","direction":"none","distance":"none","unit":"none"}
User: pinch touch pick the red object
JSON: {"action":"pick","speed":"none","object1":"cup1","relation":"none","object2":"none","direction":"none","distance":"none","unit":"none"}
User: slide the wide blue thing
JSON: {"action":"push","speed":"none","object1":"container1","relation":"none","object2":"none","direction":"none","distance":"none","unit":"none"}
User: slowly pour cup1 into the small green bowl
JSON: {"action":"pour","speed":"slow","object1":"cup1","relation":"to","object2":"bowl1","direction":"none","distance":"none","unit":"none"}
User: give me the red one
JSON: {"action":"pick","speed":"none","object1":"cup1","relation":"none","object2":"none","direction":"none","distance":"none","unit":"none"}
User: empty the mug into the green dish
JSON: {"action":"pour","speed":"none","object1":"cup1","relation":"to","object2":"bowl1","direction":"none","distance":"none","unit":"none"}
User: that is a nice object
JSON: {"action":"none","speed":"none","object1":"none","relation":"none","object2":"none","direction":"none","distance":"none","unit":"none"}
User: move left one centimeter
JSON: {"action":"move","speed":"none","object1":"none","relation":"none","object2":"none","direction":"left","distance":"1","unit":"cm"}
User: move two millimeters to the right
JSON: {"action":"move","speed":"none","object1":"none","relation":"none","object2":"none","direction":"right","distance":"2","unit":"mm"}
"""


def get_role_description(A, O, S=""):
    d = STRUCTURED
    
    if len(O) > 0 and not (O[0][-1] in "0123456789"): # if the objects not have IDs, modify the description
        d = d.replace("cup1", "cup")
        d = d.replace("bowl1", "bowl")
        d = d.replace("container1", "container")


    d = d.replace("<insert_actions>", ",".join(A))
    d = d.replace("<insert_objects>", ",".join(O))
    d = d.replace("<insert_scene>", S)

    return d
