"""PPI Assignment - Task 1 student skeleton.

Task 1: perception-driven search for a randomly placed green ball.

IMPORTANT RULE
--------------
The robot must locate the ball through the Webots camera recognition output.
Do NOT use Supervisor APIs, DEF lookup, the randomizer controller, or any other
method to read TARGET_BALL's world position in Task 1.

The supplied code already initialises the robot devices and displays the live
160 x 120 camera image. Complete the TODO sections that implement perception,
search, target approach, obstacle handling, anti-stuck recovery, and the main
behaviour/state logic.
"""

from controller import Robot
import math
import random

import cv2
import numpy as np


# =============================================================================
# 1. ROBOT AND DEVICE INITIALISATION - PROVIDED
# =============================================================================

# Task 1 deliberately uses Robot rather than Supervisor. This prevents the
# controller from directly querying the ground-truth location of TARGET_BALL.
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Differential-drive wheel motors. Setting position to infinity changes the
# motors from position-control mode to velocity-control mode.
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# The supplied camera has resolution 160 x 120. Webots recognition is enabled
# here; recognition occlusion is configured in the supplied PROTO/world.
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

# GPS is permitted in Task 1 for measuring the M-puck's OWN displacement, for
# example during the required approximately 0.5 m random relocation or for an
# anti-stuck progress test. It must not be combined with target ground truth.
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# The inertial unit provides yaw and is useful for measuring an actual 360 deg
# scan and for turning to a randomly selected heading.
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

# Eight proximity sensors are available. Decide which are appropriate for
# detecting obstacles in front of the M-puck and choose/test a threshold.
ps = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(TIME_STEP)
    ps.append(sensor)

MAX_SPEED = 6.28
BALL_RADIUS_M = 0.18


# =============================================================================
# 2. LIVE CAMERA DISPLAY - PROVIDED, DO NOT REMOVE
# =============================================================================

# The assignment requires the live camera image to be shown while Task 1 runs.
# WINDOW_AUTOSIZE shows the image at its native size; no resize/zoom is applied.
WINDOW_NAME = "M-puck Camera - Task 1"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)


def is_green(recognition_object):
    """Return True if a recognition object's configured colour is green.

    This helper is provided because Webots R2025a getColors() returns a ctypes
    pointer rather than a Python list. Therefore do not call len() on it.
    getNumberOfColors() tells us how many RGB triples may safely be indexed.

    Students may refine the green test if desired, but should explain any
    changed thresholds in the report.
    """
    n_colours = recognition_object.getNumberOfColors()
    if n_colours <= 0:
        return False

    colours = recognition_object.getColors()
    for i in range(n_colours):
        r = colours[3 * i]
        g = colours[3 * i + 1]
        b = colours[3 * i + 2]
        if g > 0.65 and g > r + 0.25 and g > b + 0.25:
            return True
    return False


def show_camera_image():
    """Display the current Webots camera image at the original 160 x 120 size.

    A green recognition bounding box is drawn when the target is recognised.
    The overlay is for observation/debugging only; the robot controller should
    still use recognition data, not pixels from this displayed overlay.
    """
    raw = camera.getImage()
    if raw is None:
        return

    width = camera.getWidth()
    height = camera.getHeight()

    # Webots camera bytes are BGRA. Convert to the BGR format expected by
    # OpenCV. No cv2.resize() call is made, so the displayed image stays native.
    bgra = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

    for obj in camera.getRecognitionObjects():
        if not is_green(obj):
            continue

        u, v = obj.getPositionOnImage()
        box_w, box_h = obj.getSizeOnImage()
        u, v = int(u), int(v)
        box_w, box_h = int(box_w), int(box_h)

        x1 = max(0, u - box_w // 2)
        y1 = max(0, v - box_h // 2)
        x2 = min(width - 1, u + box_w // 2)
        y2 = min(height - 1, v + box_h // 2)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.circle(frame, (u, v), 2, (0, 255, 0), -1)
        cv2.putText(frame, "GREEN BALL", (x1, max(10, y1 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 255, 0), 1,
                    cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME, frame)
    cv2.waitKey(1)  # required so the OpenCV GUI can refresh


def simulation_step():
    """Advance Webots one step and refresh the required live camera window."""
    running = robot.step(TIME_STEP) != -1
    if running:
        show_camera_image()
    return running


# =============================================================================
# 3. BASIC MOTION / POSE HELPERS - PROVIDED
# =============================================================================

def set_speed(left, right):
    """Set left/right wheel angular velocities, clipped to the motor limit."""
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi). Useful for IMU heading differences."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw():
    """Return current M-puck yaw in radians from the inertial unit."""
    return imu.getRollPitchYaw()[2]


def xy():
    """Return current M-puck world x,y position from GPS."""
    p = gps.getValues()
    return p[0], p[1]


# =============================================================================
# 4. TASK 1 FUNCTIONS TO COMPLETE
# =============================================================================

def front_obstacle():
    """Return True when the selected front proximity sensors indicate blockage.


      * decide which proximity sensors represent the forward direction;
      * choose/calibrate threshold(s)
    TODO:
      * justify the choices in the report.
    """
    # Robot has 8 sensors ps0, ps1 are front right and ps6 and ps7 are front left
    front_sensors = [0,1,6,7]

    # return true if any of the front sensors are above the threshold (100), meaning there is an object
    return any(ps[i].getValue() > 100 for i in front_sensors)


def find_green_ball():
    """Return the currently visible green-ball recognition object, else None.

    Hints:
      * iterate over camera.getRecognitionObjects();
      * the supplied is_green() helper can be used;
      * do not use TARGET_BALL ground truth in this task.
    """
    # loop through all objects and return green ball if found otherwise return None
    for obj in camera.getRecognitionObjects():
        if is_green(obj):
            return obj
    return None


def search_360():
    """Search visually through approximately one full 360 degree rotation.

    The function should rotate while repeatedly checking find_green_ball(). If
    the ball becomes visible, stop the rotation and return the object. If one
    full scan finishes without detection, stop and return None.

      * use IMU yaw rather than relying only on a fixed time;
      * correctly handle wraparound at -pi/+pi;
      * accumulate rotation until approximately 2*pi radians has been covered;
      * call simulation_step(), not robot.step(), so the live image remains on.
    """
    # keep track of the last yaw and total yaw covered
    last_yaw = yaw()
    total_yaw = 0.0
    target_yaw = 2 * math.pi  # target yaw after full rotation

    # rotate in place
    set_speed(0.25 * MAX_SPEED, -0.25 * MAX_SPEED)

    while simulation_step():
        # check if the ball is visible
        ball = find_green_ball()
        if ball:
            set_speed(0.0, 0.0)
            return ball

        # calculate change in yaw and accumulate total yaw covered
        current_yaw = yaw()
        yaw_diff = wrap_angle(current_yaw - last_yaw)
        total_yaw += abs(yaw_diff)
        last_yaw = current_yaw

        # check if total rotation is complete
        if total_yaw >= target_yaw:
            break

    set_speed(0.0, 0.0)
    return None


def approach_ball(obj):
    """Visually steer toward a recognised ball until reached/lost/blocked.

    A convenient steering error is based on the horizontal target coordinate:

        error = (u_ball - image_centre_x) / image_centre_x

    Remember the differential-drive steering sign: if the target is to the
    RIGHT of the image, the LEFT wheel should generally run faster than the
    right wheel so the robot turns right.

    The reference solution also avoids treating the target itself as an
    obstacle during the final close approach. Design and explain your own
    reliable logic.

    TODO:
      * centre the recognised ball and move toward it;
      * choose forward/turn gains and speed limits;
      * define a practical "reached" criterion for a radius-0.18 m ball;
      * handle temporary loss of recognition;
      * integrate obstacle handling without continuously turning away from the
        ball when the ball itself activates a front proximity sensor;
      * call simulation_step() during motion.

    Return a useful status (for example reached/lost/blocked) or design an
    equivalent state transition mechanism.
    """

    while simulation_step():
        obj = find_green_ball()

        # Ball not visible -> lost
        if obj is None:
            set_speed(0.0, 0.0)
            return "lost"

        # check if ball is reached
        _ , ball_h = obj.getSizeOnImage() #size of the ball in the image
        # check if the ball is close to the camera (ball fills 60% of the image vertically)
        if ball_h > 0.6 * camera.getHeight():
            set_speed(0.0, 0.0)
            return "reached"

        # get position of ball relative to camera
        ball_pos = obj.getPosition()
        # check if ball is blocked by obstacle & obstacle is not the ball
        if front_obstacle() and ball_pos[2] > 0.5:
            set_speed(0.0, 0.0)
            return "blocked"

        # find the horizontal position of the ball in the image
        u_ball, _ = obj.getPositionOnImage()
        # find the horizontal centre of the image
        image_centre_x = camera.getWidth() // 2

        # calculate the steering error based on the ball's position relative to the image centre
        error = (u_ball - image_centre_x) / image_centre_x

        speed_diff = 2.0
        base_speed = 2.5


        # set the wheel speeds based on the steering error
        left_wheel_speed = base_speed + speed_diff * error
        right_wheel_speed = base_speed - speed_diff * error
        set_speed(left_wheel_speed, right_wheel_speed)

    return "lost"

consecutive_blocks = 0
last_block_pos = None
def avoid_and_recover():
    """Execute obstacle avoidance plus an explicit anti-stuck strategy.

    A single repeated rule such as "if obstacle then turn left" is not enough.
    Your design should deliberately create clearance and/or detect lack of
    progress. Possible ideas include:
      * reverse before turning;
      * randomise turn direction and angle;
      * monitor GPS displacement over a time window;
      * escalate the recovery turn after repeated blockage;
      * remember recent failed escape directions.

    TODO: implement and document your strategy.
    """
    global consecutive_blocks, last_block_pos
    curr_x, curr_y = xy()

    # Update escalation count based on distance traveled since last blockage
    if last_block_pos is not None:
        dist_moved = math.hypot(curr_x - last_block_pos[0], curr_y - last_block_pos[1])
        # Reset block count if there has been more than 0.35m movement since last blockage
        if dist_moved >= 0.35:
            consecutive_blocks = 1
        else:
            consecutive_blocks += 1
    else:
        consecutive_blocks = 1

    last_block_pos = (curr_x, curr_y) # update last block position


    # reverse 0.1m to get clearance
    start_x, start_y = xy()
    set_speed(-2, -2)
    while simulation_step():
        curr_x, curr_y = xy()
        if math.hypot(curr_x - start_x, curr_y - start_y) > 0.1:
            set_speed(0.0, 0.0)
            break

    # pick turn direction away from the obstacle
    left_measure = ps[6].getValue() + ps[7].getValue()
    right_measure = ps[0].getValue() + ps[1].getValue()
    if right_measure >= left_measure:
        turn_direction = -1
    else:
        turn_direction = 1

    # escalate random turn angle after repeated blockage
    if consecutive_blocks <= 2:
        target_yaw = random.uniform(math.pi / 3, math.pi / 2)  # 60 to 90 deg
    else:
        target_yaw = random.uniform(2 * math.pi / 3, math.pi)  # 120 to 180 deg

    last_yaw = yaw()
    total_yaw = 0.0
    set_speed(2 * turn_direction, -2 * turn_direction)

    # execute the turn
    while simulation_step():
        current_yaw = yaw()
        yaw_diff = wrap_angle(current_yaw - last_yaw)
        total_yaw += abs(yaw_diff)
        last_yaw = current_yaw

        if total_yaw >= target_yaw:
            set_speed(0.0, 0.0)
            break

    # move forward 0.5m to clear the obstacle
    start_x, start_y = xy()
    set_speed(2.0, 2.0)
    while simulation_step():
        # avoid getting stuck by avoiding if a new obstacle is detected
        if front_obstacle():
            set_speed(0.0, 0.0)
            avoid_and_recover()
        curr_x, curr_y = xy()
        if math.hypot(curr_x - start_x, curr_y - start_y) > 0.5:
            break

    set_speed(0.0, 0.0)
    pass


def random_relocation(distance_m=1):
    """Move about distance_m in a randomly selected direction, then stop.

    This behaviour is used only after a complete 360-degree search fails.

    TODO:
      * choose a random heading/direction;
      * turn using IMU feedback;
      * use GPS displacement to measure approximately 0.5 m travel;
      * interrupt and recover if an obstacle is encountered;
      * detect lack of progress so the robot does not remain stuck;
      * keep the camera display active by calling simulation_step().
    """
    target_direction = wrap_angle(yaw() + random.uniform(-math.pi, math.pi))
    turn_speed = 1.5

    while simulation_step():
        # error from target direction
        error = wrap_angle(target_direction - yaw())

        #if the error is small enough stop turning
        if abs(error) < 0.05:
            set_speed(0.0, 0.0)
            break
        elif error > 0:
            # turn left
            set_speed(-turn_speed, turn_speed)
        else:
            # turn right
            set_speed(turn_speed, -turn_speed)

    start_x, start_y = xy()
    window_x, window_y = start_x, start_y
    steps = 0

    set_speed(2.0, 2.0) # move forward

    while simulation_step():
        steps += 1
        curr_x, curr_y = xy() #current postion
        displacement = math.hypot(curr_x - start_x, curr_y - start_y) # total distance travelled

        # check if the robot has travelled target distance
        if displacement >= distance_m:
            set_speed(0.0, 0.0)
            return "relocated"

        # detect lack of progress by checking every 40 steps if the robot has moved at least 2 cm
        if steps > 40:
            window_displacement = math.hypot(curr_x - window_x, curr_y - window_y)
            if window_displacement < 0.02:  # Moved less than 2 cm despite wheels driving
                set_speed(0.0, 0.0)
                return "stuck"
            window_x, window_y = curr_x, curr_y
            steps = 0

        # detect obstacle blocking robot
        if front_obstacle():
            set_speed(0.0, 0.0)
            return "blocked"

    set_speed(0.0, 0.0)
    return "stuck"


# =============================================================================
# 5. MAIN TASK 1 BEHAVIOUR - COMPLETE THE STATE/CONTROL LOGIC
# =============================================================================

# Suggested behaviour loop:
#   SEARCH    -> if target seen: APPROACH
#   SEARCH    -> if full scan fails: RELOCATE
#   APPROACH  -> if reached: DONE
#   APPROACH  -> if target lost: SEARCH
#   any motion-> if blocked/stuck: AVOID/RECOVER, then resume search
#
# You may implement this with explicit state strings/enums or with structured
# function calls. The submitted design should be clear and explained in the PDF.

state = "SEARCH"
ball = None
done = False
while simulation_step():
    if state == "SEARCH":
        print("Starting SEARCH")
        ball = search_360()
        if ball is not None:
            print("[SEARCH] Target ball detected. Switching to APPROACH.")
            state = "APPROACH"
        else:
            print("[SEARCH] No ball found in 360 scan. Switching to RELOCATE.")
            state = "RELOCATE"

    elif state == "APPROACH":
        print("starting APPROACH")
        result = approach_ball(ball)
        if result == "reached":
            print("[APPROACH] Target successfully reached! Transitioning to DONE.")
            state = "DONE"
        elif result == "blocked":
            print("[APPROACH] Blocked by obstacle. Switching to AVOID.")
            state = "AVOID"
        else:
            print("[APPROACH] Target lost from view. Returning to SEARCH.")
            ball = None
            state = "SEARCH"

    elif state == "RELOCATE":
        print("starting RELOCATE")
        res = random_relocation()
        if res == "blocked" or res == "stuck":
            print(f"[RELOCATE] Relocation interrupted ({res}). Switching to AVOID.")
            state = "AVOID"
        else:
            print("[RELOCATE] Relocation complete. Returning to SEARCH.")
            state = "SEARCH"

    elif state == "AVOID":
        print("starting AVOID")
        avoid_and_recover()
        print("[AVOID] Clearance created. Returning to SEARCH.")
        ball = None
        state = "SEARCH"

    elif state == "DONE":
        set_speed(0.0, 0.0)
        if not done:
            print("[STATE] Task complete. Motors stopped.")
            done = True
# Stop motors and close the OpenCV window if the simulation exits normally.
set_speed(0.0, 0.0)
cv2.destroyAllWindows()
