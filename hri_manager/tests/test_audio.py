# test_audio_loopback.py
"""
End-to-end loop-back test:

1.  Generate a sound that is hard to detect by the mic-filter and play it through the
    default speakers with ``playsound``.
2.  Record simultaneously from the microphone device
    using *arecord* (adjust the command block if you use a different
    sound card).
3.  Analyse the captured WAV with ``audioop.rms`` and assert that the
    average volume is above a silence threshold – i.e. sound was heard.
"""
import random
import math
import struct
import time
import wave
from pathlib import Path
import audioop       # std-lib on Linux; part of CPython
import pytest
from playsound import playsound

import hri_manager
from natural_language_processing.speech_to_text.audio_recorder import AudioRecorder

def generate_tone(path: Path, freq: int = 440, dur_s: float = 2.0,
                  rate: int = 44_100, amp: float = 0.5) -> None:
    """Write a 16-bit mono sine wave to *path*."""
    n_frames = int(rate * dur_s)
    with wave.open(str(path), "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)         # 16-bit
        wf.setframerate(rate)
        for i in range(n_frames):
            sample = amp * math.sin(2 * math.pi * freq * i / rate)
            wf.writeframesraw(struct.pack("<h", int(sample * 32767)))

def rms_of_wav(path: Path) -> int:
    """Return overall RMS of a WAV file."""
    with wave.open(str(path), "rb") as wf:
        frames = wf.readframes(wf.getnframes())
        return audioop.rms(frames, wf.getsampwidth())

def make_hardly_filtered_sound(
        path: Path,
        dur_s: float = 3.0,
        fs: int = 44_100,
        f_start: int = 150,      # sweep start (Hz)
        f_end: int = 7_000,      # sweep end   (Hz)
        amp: float = 0.8,
        seg_ms: int = 30,        # tiny randomised AM segments
) -> None:
    """
    Write a wide-band, time-varying signal that defeats most echo-cancellers.

    * Linear chirp from `f_start` to `f_end` over `dur_s` seconds.
    * Random 30 ms amplitude-modulated segments force frequent spectral change.
    * Output: 16-bit mono WAV at `fs` Hz stored in *path*.
    """
    n_frames = int(dur_s * fs)
    chirp_frames = []

    # --- pre-compute chirp phase increment per sample ------------
    k = (f_end - f_start) / dur_s          # sweep rate (Hz per second)
    phase = 0.0
    for i in range(n_frames):
        t = i / fs
        inst_freq = f_start + k * t        # instantaneous frequency
        phase += 2 * math.pi * inst_freq / fs
        chirp_frames.append(math.sin(phase))

    # --- scramble with random amplitude modulation --------------
    frames_per_seg = int(fs * seg_ms / 1000)
    idx = 0
    while idx < n_frames:
        seg_len = min(frames_per_seg, n_frames - idx)
        seg_gain = random.uniform(0.3, 1.0)     # avoid total silence
        for j in range(seg_len):
            chirp_frames[idx + j] *= seg_gain
        idx += seg_len

    # --- normalise & write wav ----------------------------------
    wav_frames = (int(max(-1.0, min(1.0, s * amp)) * 32767)
                  for s in chirp_frames)
    with wave.open(str(path), "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(
            struct.pack("<%dh" % n_frames, *wav_frames)
        )

@pytest.mark.timeout(30)        # cancel the test if it hangs >30 s
def test_speaker_microphone_loopback(tmp_path: Path) -> None:
    """
    Fails if the speakers play nothing *or* the microphone captures
    only silence.
    """
    tone_file = tmp_path / "tone.wav"
    rec_file = tmp_path / "recorded.wav"
    
    make_hardly_filtered_sound(tone_file)

    recorder = AudioRecorder()
    recorder.start_recording(rec_file, duration=4)   # start listening

    time.sleep(0.5)                        # give ALSA a moment
    playsound(str(tone_file))              # play the test tone
    time.sleep(0.5)                        # trailing tail

    recorder.stop_recording()

    # --- very naïve detection: just check overall loudness ------------
    SILENCE_RMS_THRESHOLD = 100            # tweak if needed (0-32767)
    rms_val = rms_of_wav(rec_file)

    assert rms_val > SILENCE_RMS_THRESHOLD, (
        f"No audible signal detected (RMS={rms_val}). "
        "Check cabling, volume and device indexes."
    )

if __name__ == "__main__":
    test_speaker_microphone_loopback(tmp_path=Path(f"{hri_manager.package_path}"))