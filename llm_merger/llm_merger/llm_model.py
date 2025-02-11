
import numpy as np


class LLMMerger():
    def __init__(self):

    
    def from_nlp(self, msg):
        """msg is whisperx
        """        
        stamped_sentence = from_whisperx(msg)


    def from_gestures():

        #["action gestures": ["grab", "pinch"], probs: [1.0, 0.9], "timestamps": [0.0, 1.0],
        # "point gestures": ["drawer", "cube"], "probs": [0.9,0.8], "timestamps": [0.4,0.4],
        #]
        pass


    def sort_merge(self, gesture_stamped, voice_stamped):
        """ gesture_stamped: [:,0] - timestamps, [:,1] - words 
            voice_stamped:   [:,0] - timestamps, [:,1] - words
        """      
        stacked = gesture_stamped.extend(voice_stamped)
        return sorted(stacked, key=lambda x: x[0])

    def forward(self, gesture_stamped, voice_stamped):
        sorted_sentence = self.sort_merge(gesture_stamped, voice_stamped)
        (sorted_sentence) # put to the llm


def from_whisperx(whisperx_dict):
    #[{"start": 0.028, "end": 1.289, "text": " It's the imagination.", 
    # "words": [{"word": "It's", "start": 0.028, "end": 0.128, "score": 0.218}, 
    # {"word": "the", "start": 0.148, "end": 0.248, "score": 0.683}, 
    # {"word": "imagination.", "start": 0.268, "end": 0.929, "score": 0.873}]},
    # ...
    # ]
    
    timestamped_words = []
    for sentence in whisperx_dict:
        for word in sentence['words']:
            timestamped_words.append([word["end"], word['word']])
    return timestamped_words