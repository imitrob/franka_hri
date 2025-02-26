import random
from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np
from fuzzywuzzy import fuzz  # Requires 'fuzzywuzzy' library: pip install fuzzywuzzy


# ================= Enhanced Configuration =================
CONFIG = {
    "actions": ["stop", "release", "home", "pick", "push", "pass", "place", "point", "open", "close", "pour", "put"],
    "adjectives": ["quickly", "slowly", "carefully", "lightly", "force"],
    "prepositions": ["to", "into", "onto", "from"],
    "object_types": ["cube", "bow", "cup", "container", "water"],
    # "object_types": ["cube", "bowl", "cup", "drawer", "bottle"],
    "properties": {
        "size": ["small", "medium", "large"],
        "color": ["red", "green", "blue", "yellow"],
        "state": ["open", "closed", "half-full"]
    },
    "noise": {
        "phonetic_confusion": 0.0, #0.2,  # Probability of phonetic-based errors
        "filler_words": 0.0, #0.3,        # Probability of adding filler words
        "alignment_noise": 0.0, #0.4,     # 0=perfect alignment, 1=high misalignment
        "incomplete_sentence": 0.0, #0.1  # Probability of truncated commands
    },
    "max_instances": 1, #3,
    "filler_words": ["um", "ah", "like", "you know", "well", "so"]
}
PHONETIC_SIMILARITY_THR = 70
OBJECT_HAS_IDS = False # max_instantes should be 1
# ===========================================================

@dataclass
class SceneObject:
    obj_id: str
    obj_type: str
    properties: Dict[str, str]


class EnhancedDatasetGenerator:
    def __init__(self, config=CONFIG):
        self.cfg = config
        self._init_phonetic_dictionary()
        
    def _init_phonetic_dictionary(self):
        """Precompute phonetic representations of vocabulary"""
        self.phonetic_map = {}
        for word in set(self.cfg['actions'] + self.cfg['adjectives'] + 
                      self.cfg['prepositions'] + 
                      [p for plist in self.cfg['properties'].values() for p in plist]):
            self.phonetic_map[word] = {
                'ratio': fuzz.ratio
            }
    def _phonetic_similarity(self, w1: str, w2: str) -> bool:
        """Check phonetic similarity using fuzzywuzzy ratio"""
        ratio = fuzz.ratio(w1.lower(), w2.lower())
        return ratio > PHONETIC_SIMILARITY_THR

    def generate_scene(self) -> Dict[str, SceneObject]:
        """Generate random scene with unique object instances"""
        scene = {}
        for obj_type in self.cfg['object_types']:
            for i in range(random.randint(1, self.cfg['max_instances'])):
                if OBJECT_HAS_IDS:
                    obj_id = f"{obj_type}{i+1}"
                else:
                    obj_id = f"{obj_type}"
                print("obj_id", obj_id)
                properties = {
                    "size": random.choice(self.cfg['properties']['size']),
                    "color": random.choice(self.cfg['properties']['color']),
                    "state": random.choice(self.cfg['properties']['state'])
                }
                scene[obj_id] = SceneObject(obj_id, obj_type, properties)
        return scene

    def _add_linguistic_noise(self, word: str) -> Dict[str, float]:
        """Advanced noise model with phonetic confusion and filler words"""
        # Add filler words
        if random.random() < self.cfg['noise']['filler_words']:
            filler = random.choice(self.cfg['filler_words'])
            return {filler: 1.0}  # Replace word with filler
        
        # Phonetic confusion
        if random.random() < self.cfg['noise']['phonetic_confusion']:
            candidates = [w for w in self.phonetic_map 
                         if self._phonetic_similarity(w, word)]
            if candidates:
                confused = random.choice(candidates)
                return {confused: 0.7, word: 0.3}
                
        return {word: 1.0}

    def _apply_alignment_noise(self, voice: dict, gesture: dict) -> dict:
        """Shift gesture timestamps based on alignment noise level"""
        max_shift = self.cfg['noise']['alignment_noise'] * 2.0  # Max 2 sec shift
        shifted_gesture = {}
        
        for ts, probs in gesture.items():
            # Shift timestamp later with probability proportional to noise level
            if random.random() < self.cfg['noise']['alignment_noise']:
                new_ts = ts + random.uniform(0, max_shift)
            else:
                new_ts = ts
            shifted_gesture[new_ts] = probs
            
        return shifted_gesture

    def generate_voice_command(self, true_sentence: str) -> Dict[float, Dict[str, float]]:
        """Generate voice command with natural disfluencies"""
        words = true_sentence.lower().split()
        voice_input = {}
        timestamp = 0.0
        
        # Randomly truncate sentence
        if random.random() < self.cfg['noise']['incomplete_sentence']:
            words = words[:random.randint(1, len(words)-1)]
            
        for word in words:
            timestamp += round(random.uniform(0.1, 0.3), 1)
            alternatives = self._add_linguistic_noise(word)
            voice_input[timestamp] = alternatives
            
        return voice_input

    def generate_gesture_input(self, target_object: str, scene: dict, 
                              voice_timestamps: list) -> Dict[float, Dict[str, float]]:
        """Generate gesture input with temporal alignment noise"""
        # Base timestamp should align with relevant voice command word
        try:
            base_ts = random.choice(voice_timestamps)
        except IndexError:
            base_ts = 0.5
            
        timestamp = round(base_ts + random.uniform(-0.2, 0.2), 1)
        probs = self._create_gesture_distribution(target_object, scene)
        return {timestamp: probs}

    def _create_gesture_distribution(self, target: str, scene: dict) -> dict:
        """Create realistic gesture probability distribution"""
        objects = list(scene.keys())
        probs = {obj: 0.0 for obj in objects}
        
        # Correct target gets majority probability
        probs[target] = random.uniform(0.6, 0.95)
        remaining = 1 - probs[target]
        
        # Distribute remaining probability to similar-looking objects
        similar_objects = [obj for obj in objects 
                          if scene[obj].obj_type == scene[target].obj_type]
        for obj in random.sample(similar_objects, min(3, len(similar_objects))):
            if obj != target:
                alloc = remaining * random.uniform(0.2, 0.8)
                probs[obj] += alloc
                remaining -= alloc
                
        # Add small noise to other objects
        for obj in objects:
            if obj not in similar_objects and obj != target:
                probs[obj] += remaining / (len(objects) - len(similar_objects))
                
        return {k: round(v, 2) for k, v in probs.items() if v > 0}

    def generate_sample(self) -> dict:
        """Generate complete sample with enhanced realism"""
        scene = self.generate_scene()
        action = random.choice(self.cfg['actions'])
        
        # Generate true command with object uniqueness check
        if action == "stop":
            true_sentence = "stop"
        else:
            target_obj, dest_obj = None, None
            while target_obj == dest_obj:  # Ensure different objects
                target_obj = random.choice(list(scene.values()))
                if action in ["place", "pour"]:
                    dest_objs = [obj for obj in scene.values() if obj != target_obj]
                    dest_obj = random.choice(dest_objs) if dest_objs else None
            
            params = {
                "adjective": random.choice(self.cfg['adjectives']) 
                            if random.random() > 0.5 else "",
                "preposition": random.choice(self.cfg['prepositions']) 
                               if action in ["place", "pour"] else ""
            }
            
            true_parts = [
                params.get("adjective"),
                action,
                target_obj.obj_id,
                params.get("preposition"),
                dest_obj.obj_id if dest_obj else ""
            ]
            true_sentence = " ".join(filter(None, true_parts)).strip()

        # Generate inputs with alignment noise
        voice = self.generate_voice_command(true_sentence)
        gesture = self.generate_gesture_input(
            target_obj.obj_id if action != "stop" else "",
            scene,
            list(voice.keys())
        )
        gesture = self._apply_alignment_noise(voice, gesture)

        return Sample(
            {k: f"{v.properties['size']} {v.properties['color']} {v.obj_type}" 
                for k, v in scene.items()},
            voice,
            gesture,
            true_sentence,
            CONFIG
        )

class cc:
    H = '\033[95m'
    OK = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    W = '\033[93m'
    F = '\033[91m'
    E = '\033[0m'
    B = '\033[1m'
    U = '\033[4m'

@dataclass
class Sample():
    scene: str
    voice: str
    gesture: Dict[str, str]
    true_sentence: Dict[str, str]
    CONFIG: dict

    def __str__(self):
        s = ""
        s += "Generated Sample:\n"
        s += f"{cc.W}Scene:{cc.E} {self.scene} \n"
        s += f"{cc.W}Voice Input:{cc.E}\n"
        for ts, words in self.voice.items():
            s += f"{ts}: {words}\n"
        s += f"{cc.W}Gesture Input:{cc.E}\n"
        for ts, gword in self.gesture.items():
            s += f"{ts}: {gword}\n"
        s += f"True Sentence: {self.true_sentence}\n\n"
        return s
    
    def export(self, interpret_format):
        if interpret_format == "deterministic":
            return self.to_deterministic()
        elif interpret_format == "probabilistic":
            return self.to_probabilistic()
        else: raise Exception()

    def to_probabilistic(self):
        voice_stamped = [i for i in self.voice.items()]
        gesture_stamped = [i for i in self.gesture.items()]
        scene_list = [i for i in self.scene.items()]
        scene = ""
        for o in scene_list:
            scene += " is ".join(o) + ". "
        object_names = list(self.scene.keys())  
        return voice_stamped, gesture_stamped, scene, object_names, self.true_sentence, self.CONFIG
        
    def to_deterministic(self):
        voice_stamped = []
        for t, p in self.voice.items():
            voice_stamped.append([t, max(p, key=p.get)])
        gesture_stamped = []
        for t, p in self.gesture.items():
            gesture_stamped.append([t, max(p, key=p.get)])
        scene_list = [i for i in self.scene.items()]
        scene = ""
        for o in scene_list:
            scene += " is ".join(o) + ". "
        object_names = list(self.scene.keys())  
        
        return voice_stamped, gesture_stamped, scene, object_names, self.true_sentence, self.CONFIG
        

# ================= Usage Example =================
if __name__ == "__main__":
    generator = EnhancedDatasetGenerator()
    
    # Generate sample with high alignment noise
    generator.cfg['noise']['alignment_noise'] = 0.8  # 80% alignment noise
    sample = generator.generate_sample()

    print(sample)