WEBOTS LAB 1 PROJECT
====================

Required software:
- Webots R2025a
- Python 3 supported/configured by Webots

Open the world:
1. Start Webots.
2. Choose File > Open World.
3. Open worlds/lab1_getting_started.wbt.
4. Press the Run button.

The world initially uses the completed reference controller:
  controller "lab1_reference"

To use the student starter controller:
1. Select the e-puck in the Scene Tree.
2. Change its controller field to "lab1_student".
3. Save and reset the simulation.

Project structure:
- worlds/lab1_getting_started.wbt
- controllers/lab1_student/lab1_student.py
- controllers/lab1_reference/lab1_reference.py

Troubleshooting:
- Keep this folder structure unchanged.
- Ensure the controller name exactly matches its folder name.
- If Webots cannot start Python, check Tools > Preferences > Python command.
- If a device is reported as missing, verify the e-puck model and device names.
