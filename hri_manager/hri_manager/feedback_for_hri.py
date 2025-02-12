

from skills_manager.feedback import Feedback
from pynput.keyboard import KeyCode, Key

from std_msgs.msg import String

class Feedback_for_HRI(Feedback):
    def __init__(self):
        super(Feedback_for_HRI, self).__init__()
        self.is_recording = False
        
    def _on_press(self, key):
        if key == KeyCode.from_char("+"):
            if not self.is_recording:
                self.is_recording = True
                self.rec.start_recording()
        super()._on_press(key)

    def _on_release(self, key):
        if key == KeyCode.from_char("+"):
            if self.is_recording:
                self.is_recording = False
                recording_name, start_time = self.rec.stop_recording()
                if recording_name is not None:
                    self.voicerecord_pub.publish(String(data=f'{{"file": "{recording_name}", "timestamp": {start_time} }}'))
                    print("Voice recorded and msg sent", flush=True)