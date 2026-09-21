"""Lab 4 student controller: complete the TODO sections in order.

The unfinished scaffold is deliberately safe: it keeps the robot stopped until
the motor, speech-recognition, interpretation, and execution TODOs are filled.
Compare your work with lab4_reference.py only after attempting the tasks.
"""

from controller import Robot

import json
import queue
import re
import sys
from pathlib import Path

import sounddevice as sd
from vosk import KaldiRecognizer, Model, SetLogLevel


robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
speaker = robot.getDevice("speaker")

# TODO 1: Put both motors into velocity-control mode.
# Hint: set each target position to float("inf").

# TODO 2: Set both initial motor velocities to zero.


SAMPLE_RATE = 16000
BLOCK_SIZE = 8000
AUDIO_DEVICE = None  # Use an input-device index if the default is unsuitable.
ACTION_DURATION = 1.5
SPEAKER_VOLUME = 0.8

MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "vosk-model-small-en-us-0.15"
)

COMMAND_PHRASES = {
    "FORWARD": ("move forward", "go forward", "go ahead"),
    "BACKWARD": ("move backward", "go backward", "go back", "reverse"),
    "LEFT": ("turn left", "rotate left"),
    "RIGHT": ("turn right", "rotate right"),
    "STOP": ("stop", "halt", "wait", "stay still"),
}

ACTION_SPEEDS = {
    "FORWARD": (3.0, 3.0),
    "BACKWARD": (-3.0, -3.0),
    "LEFT": (-2.0, 2.0),
    "RIGHT": (2.0, -2.0),
    "STOP": (0.0, 0.0),
}

ACKNOWLEDGEMENTS = {
    "FORWARD": "Moving forward.",
    "BACKWARD": "Moving backward.",
    "LEFT": "Turning left.",
    "RIGHT": "Turning right.",
    "STOP": "Stopping.",
}

audio_queue = queue.Queue()


def audio_callback(indata, frames, time_info, status):
    """Receive one microphone block in sounddevice's audio thread."""
    del frames, time_info
    if status:
        print(f"Audio status: {status}", file=sys.stderr)

    # TODO 3: Copy the raw audio block into audio_queue.
    # Hint: the queue item should be bytes(indata). Keep this callback short.
    pass


def drain_audio_queue():
    while True:
        try:
            audio_queue.get_nowait()
        except queue.Empty:
            return


def interpret_command(text):
    """Return one action, AMBIGUOUS, or UNKNOWN."""
    # TODO 4: Normalise the recognised text and find matching phrases.
    # Requirements:
    #   1. Match complete phrases, not word fragments.
    #   2. Return an action only when exactly one action matches.
    #   3. Return "AMBIGUOUS" when multiple actions match.
    #   4. Return "UNKNOWN" when no action matches or text is negated.
    # Hint: re.sub(), lower(), split(), and a padded string are useful.
    del text
    return "UNKNOWN"


def set_wheel_speeds(action):
    """Apply the two motor speeds associated with an action."""
    # TODO 5: Read the (left, right) pair from ACTION_SPEEDS and apply it.
    # Hint: call setVelocity() on each wheel motor.
    del action
    pass


if not MODEL_PATH.is_dir():
    raise FileNotFoundError(
        f"Vosk model not found: {MODEL_PATH}. "
        "Run 'python download_vosk_model.py' from the project folder."
    )

SetLogLevel(-1)

# TODO 6: Load MODEL_PATH once and create a KaldiRecognizer at SAMPLE_RATE.
# Replace these safe placeholders with Model(...) and KaldiRecognizer(...).
model = None
recogniser = None

# TODO 7: Create a mono int16 sd.RawInputStream using SAMPLE_RATE,
# BLOCK_SIZE, AUDIO_DEVICE, and audio_callback, then start it before the loop.
audio_stream = None

current_action = "STOP"
action_end_time = 0.0
ready_message_spoken = False

try:
    while robot.step(TIME_STEP) != -1:
        # TODO 8: Speak "Speech control is ready." exactly once.
        # Hint: use ready_message_spoken to prevent repeated calls.

        # TODO 9: Stop a timed movement when robot.getTime() reaches
        # action_end_time. Do not use time.sleep().

        # TODO 10: While speaker.isSpeaking() is true, discard queued audio
        # so that Vosk does not recognise the robot's own acknowledgement.

        # TODO 11: Remove queued audio without blocking. For every complete
        # Vosk waveform, parse recogniser.Result(), print its "text", call
        # interpret_command(), validate the result, start the selected action,
        # set action_end_time, and speak one acknowledgement.
        # Safety rule: UNKNOWN and AMBIGUOUS must stop both motors.
        pass
finally:
    # TODO 12: Stop the wheels and safely stop/close audio_stream if it exists.
    pass

