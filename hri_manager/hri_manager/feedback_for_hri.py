

from skills_manager.feedback import Feedback
from pynput.keyboard import KeyCode, Key

class Feedback_for_HRI(Feedback):
    def __init__(self):
        super(Feedback_for_HRI, self).__init__()
        self.is_recording = False
        
    def on_press(self, key):
        if key == KeyCode.from_char("r"):
            self.is_recording = True
            self.rec.start_recording()
        super()._on_press(key)

    def on_release(self, key):
        if key == KeyCode.from_char("r"):
            if self.is_recording:
                self.is_recording = False
                recording_name = self.rec.stop_recording()
                if recording_name is not None:
                    print("Processing started", flush=True)
                    self.nl_forward(recording_name)
