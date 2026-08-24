"""
Student skeleton controller — Perception, Planning and Interactions
Lab: Search for a red ball with Webots Camera Recognition
Webots R2025a / Python 3.12

Complete the TODO sections only. The camera display helper is supplied.
"""
from controller import Robot
import numpy as np
import cv2

TIME_STEP = 32
MAX_SPEED = 6.28
SEARCH_SPEED = 1.8
FORWARD_SPEED = 3.0
TURN_GAIN = 2.5
OBSTACLE_THRESHOLD = 100.0

robot = Robot()

# Devices
camera = robot.getDevice("camera")
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
proximity = [robot.getDevice(f"ps{i}") for i in range(8)]

camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)
for sensor in proximity:
    sensor.enable(TIME_STEP)

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


def show_camera_image():
    """Always show the 160 x 120 camera image."""
    image = camera.getImage()
    if image is None:
        return
    width = camera.getWidth()
    height = camera.getHeight()
    bgra = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
    bgr = bgra[:, :, :3]
    cv2.imshow("M-puck camera (160x120)", bgr)
    cv2.waitKey(1)


def set_wheel_speeds(left, right):
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)


def red_ball_object():
    """Return the recognised red-ball object, or None.

    TODO 1:
      - call camera.getRecognitionObjects()
      - inspect each object's recognition colours
      - identify the object whose first colour is approximately red
        (R high, G and B low)
    """
    # TODO 1: replace the next line.
    return None


def obstacle_ahead():
    """Return True when an obstacle is detected in front of the M-puck.

    TODO 2:
      - read the front-facing proximity sensors
      - a useful initial set is ps0, ps1, ps6 and ps7
      - compare their readings with OBSTACLE_THRESHOLD
    """
    # TODO 2: replace the next line.
    return False


def approach_ball(obj):
    """Steer toward the ball using its horizontal image position.

    TODO 3:
      1. obtain x pixel coordinate from obj.getPositionOnImage()
      2. compute horizontal error from image centre
      3. normalise error to roughly [-1, +1]
      4. convert error into differential wheel speeds

    Hint:
      turn = TURN_GAIN * error
      left  = FORWARD_SPEED -/+ turn
      right = FORWARD_SPEED +/- turn

    Check the sign experimentally: if the ball is on the left, the robot
    must turn left, not right.
    """
    # TODO 3: replace with your steering controller.
    set_wheel_speeds(0.0, 0.0)


print("Red-ball lab controller started.")
print("Camera resolution:", camera.getWidth(), "x", camera.getHeight())

while robot.step(TIME_STEP) != -1:
    # Requirement: show every acquired camera frame.
    show_camera_image()

    # Safety / task termination condition has priority.
    if obstacle_ahead():
        set_wheel_speeds(0.0, 0.0)
        print("Obstacle detected: M-puck stopped.")
        continue

    ball = red_ball_object()

    if ball is None:
        # SEARCH: rotate in place until the ball enters the camera FOV.
        # TODO 4: choose wheel signs so the robot rotates on the spot.
        set_wheel_speeds(0.0, 0.0)  # TODO 4
    else:
        # APPROACH: keep the recognised ball near the image centre.
        approach_ball(ball)

cv2.destroyAllWindows()
