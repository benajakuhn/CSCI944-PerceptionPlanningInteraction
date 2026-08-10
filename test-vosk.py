from __future__ import annotations
import json
import queue
import sys
from pathlib import Path
import sounddevice as sd
from vosk import KaldiRecognizer, Model

MODEL_PATH = Path(r"C:\Code\vosk-models\vosk-model-small-en-us-0.15")
SAMPLE_RATE = 16000

audio_queue = queue.Queue()

def callback(indata, frames, time_info, status):
    if status:
        print(status, file=sys.stderr)
    audio_queue.put(bytes(indata))

def main():
    if not MODEL_PATH.exists():
        print(f"Error: Model not found at {MODEL_PATH}")
        return

    print("Loading Vosk model...")
    model = Model(str(MODEL_PATH))
    recognizer = KaldiRecognizer(model, SAMPLE_RATE)

    print("\nSpeak into your microphone! Press Ctrl+C to stop.\n")

    try:
        with sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=8000,
            dtype="int16",
            channels=1,
            callback=callback
        ):
            while True:
                data = audio_queue.get()
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get("text", "")
                    if text:
                        print("Recognised:", text)
    except KeyboardInterrupt:
        print("\nTest stopped.")

if __name__ == "__main__":
    main()