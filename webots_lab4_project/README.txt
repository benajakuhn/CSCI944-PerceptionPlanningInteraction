CSCI944/CSCI444 LAB 4 - SPEECH INTERACTION
==========================================

CONTENTS
----------------

worlds/lab4_speech_interaction.wbt
    Webots R2025a world with an E-puck, two wheel motors, and a Speaker.

controllers/lab4_student/lab4_student.py
    Safe student scaffold with numbered TODO sections.

controllers/lab4_reference/lab4_reference.py
    Complete sample solution.

download_vosk_model.py
    Cross-platform downloader for the required Vosk speech model.

requirements.txt
    Python package requirements.

models/
    Destination for the separately downloaded Vosk model data.


QUICK START
-----------

1. Extract this ZIP while preserving its folder structure.

2. Use the Python interpreter configured in Webots to install the packages:

       python -m pip install -r requirements.txt

   Depending on the operating system, PortAudio may also be required by
   sounddevice. Confirm that a microphone is visible with:

       python -m sounddevice

3. Download the Vosk model from the project folder:

       python download_vosk_model.py

   Alternatively, manually download vosk-model-small-en-us-0.15 from
   https://alphacephei.com/vosk/models and extract its folder into models/.

4. Open worlds/lab4_speech_interaction.wbt in Webots.

5. Run the simulation in Real-time mode. The world selects lab4_student by
   default. Complete the TODO sections in numerical order.

6. To run the sample solution, stop the simulation, select the E-puck in the
   Scene Tree, change its controller field to lab4_reference, and reset/run.


SUPPORTED UTTERANCES
--------------------

Action      Example phrases
FORWARD     move forward; go forward; go ahead
BACKWARD    move backward; go backward; go back; reverse
LEFT        turn left; rotate left
RIGHT       turn right; rotate right
STOP        stop; halt; wait; stay still

The required controller accepts one action per utterance. UNKNOWN, ambiguous,
and negated text is rejected, and the robot stops. More complex instructions,
distances, timing sequences, and unsupported actions are outside this lab.


IMPORTANT NOTES
---------------

- sounddevice listens to the computer's physical microphone. It is not a
  simulated Webots Microphone device.
- Keep the sounddevice callback short. Webots API calls belong in the main
  controller loop.
- Do not use time.sleep() for movement timing; use robot.getTime().
- The controller drains microphone blocks while the Speaker talks to reduce
  feedback. Headphones further reduce echo.
- If the default input device is unsuitable, replace AUDIO_DEVICE = None in
  the selected controller with the device index shown by python -m sounddevice.
- Webots must use the same Python interpreter where vosk and sounddevice were
  installed (Tools > Preferences > Python command).


EXPECTED DEVICES
----------------

left wheel motor
right wheel motor
speaker


PRE-LAB CHECK - AUDIO DEVICE AND SAMPLING RATE
===============================================

Complete this check before starting the Webots simulation. The physical
microphone and its supported sampling rates depend on the computer, operating
system, audio driver and selected audio interface. Do not assume that every
computer accepts 16 kHz directly.

1. Activate the same Python environment configured in Webots.

       conda activate webots_lab4

   Replace webots_lab4 with the environment name used on your computer.

2. List the audio devices detected by sounddevice.

       python -m sounddevice

   Select a device whose description shows one or more input channels, for
   example "1 in" or "2 in". A device showing "0 in" cannot record speech.
   Avoid loopback devices such as "Stereo Mix" unless the task specifically
   requires recording the computer's playback. Record the index of the intended
   microphone.

3. Inspect the selected microphone. Replace 6 with its device index.

       python -c "import sounddevice as sd; d=sd.query_devices(6, 'input'); print(d); print('Native rate:', int(d['default_samplerate']))"

4. Test whether the device accepts the preferred 16 kHz format.

       python -c "import sounddevice as sd; sd.check_input_settings(device=6, channels=1, dtype='int16', samplerate=16000); print('16 kHz is supported')"

   If this command succeeds, the actual sampling rate can be 16000 Hz. If it
   reports "Invalid sample rate", test the device's native rate instead:

       python -c "import sounddevice as sd; d=sd.query_devices(6, 'input'); r=int(d['default_samplerate']); sd.check_input_settings(device=6, channels=1, dtype='int16', samplerate=r); print('Supported native rate:', r)"

5. Configure the controller with the microphone index.

       AUDIO_DEVICE = 6

   The controller should prefer 16000 Hz and fall back to the selected device's
   native rate when 16 kHz is unsupported. The actual sampling rate must be used
   consistently in both components:

       recogniser = KaldiRecognizer(model, actual_sample_rate)

       audio_stream = sd.RawInputStream(
           samplerate=actual_sample_rate,
           ...
       )

   Do not capture audio at one rate while configuring Vosk with another rate.

6. Start Webots in Real-time mode. A correct reference-controller startup
   prints information similar to:

       Microphone: Microphone (Realtek HD Audio Mic input) (device 6, 44100 Hz)
       Speech controller ready. Say one supported command.

Troubleshooting
---------------

- No input devices are listed: connect or enable a microphone, then enable
  Windows Settings > Privacy & security > Microphone > Microphone access and
  "Let desktop apps access your microphone".
- Error querying device -1: Windows has no default input device. Set
  AUDIO_DEVICE to a valid index shown by python -m sounddevice.
- Invalid sample rate: use the device's default_samplerate and pass it to both
  sounddevice and KaldiRecognizer.
- The wrong source is captured: select an actual microphone rather than Stereo
  Mix, a speaker, or another device showing 0 input channels.

