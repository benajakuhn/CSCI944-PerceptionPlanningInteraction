"""CSCI944-style lab skeleton: GPS + IMU odometry and robot kinematics.
Webots R2025a / Python 3.12

Complete the TODO sections. The robot should wander through the break-room,
avoid obstacles, and report odometry and kinematic quantities derived from
GPS position and InertialUnit yaw.
"""

from controller import Robot
import math
import random

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
DT = TIME_STEP / 1000.0

# M-puck geometry taken from the supplied PROTO.
WHEEL_RADIUS = 0.05       # m
AXLE_LENGTH = 0.13        # m (wheel centres at y = +/-0.065 m)
MAX_WHEEL_SPEED = 6.28    # rad/s for version 1

# ---------------------------------------------------------------------------
# 1. Motors and proximity sensors
# ---------------------------------------------------------------------------
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ps = [robot.getDevice(f"ps{i}") for i in range(8)]
for sensor in ps:
    sensor.enable(TIME_STEP)

# ---------------------------------------------------------------------------
# 2. GPS and IMU
# ---------------------------------------------------------------------------
# TODO 1: obtain the devices named "gps" and "inertial unit" and enable them.
# gps = ...
# imu = ...

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
def wrap_to_pi(angle):
    """Return an angle in [-pi, pi)."""
    # TODO 2
    return angle


def world_to_initial_frame(dx_world, dy_world, yaw0):
    """Rotate a world-frame displacement into the robot's initial odom frame."""
    # TODO 3: implement the 2-D rotation.
    x_odom = dx_world
    y_odom = dy_world
    return x_odom, y_odom


def measured_kinematics(x, y, yaw, prev_x, prev_y, prev_yaw, dt):
    """Estimate translational and angular velocities from GPS/IMU differences."""
    # TODO 4:
    #   vx_world, vy_world
    #   speed = sqrt(vx_world^2 + vy_world^2)
    #   v_forward and v_lateral by rotating world velocity into the robot frame
    #   yaw_rate from wrapped yaw difference
    vx_world = 0.0
    vy_world = 0.0
    speed = 0.0
    v_forward = 0.0
    v_lateral = 0.0
    yaw_rate = 0.0
    return vx_world, vy_world, speed, v_forward, v_lateral, yaw_rate


def command_kinematics(w_left, w_right):
    """Ideal differential-drive kinematics from commanded wheel angular speeds."""
    # TODO 5: compute ideal linear speed and yaw rate.
    v_cmd = 0.0
    omega_cmd = 0.0
    return v_cmd, omega_cmd


# ---------------------------------------------------------------------------
# 3. Random wandering state
# ---------------------------------------------------------------------------
BASE_SPEED = 3.0
OBSTACLE_THRESHOLD = 100.0
turn_until = 0.0
next_random_turn = 3.0
turn_direction = 1.0

# Odometry state: initialize after the first valid GPS/IMU sample.
initialized = False
x0 = y0 = yaw0 = 0.0
prev_x = prev_y = prev_yaw = 0.0
path_length = 0.0
last_report_time = -1.0

# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------
while robot.step(TIME_STEP) != -1:
    t = robot.getTime()

    # TODO 6: read GPS [x, y, z] and IMU [roll, pitch, yaw].
    # x, y, z = gps.getValues()
    # roll, pitch, yaw = imu.getRollPitchYaw()

    # Remove these placeholder values after TODO 6 is completed.
    x = y = z = roll = pitch = yaw = 0.0

    if not initialized:
        # TODO 7: store the initial pose and previous pose.
        # The odometry frame should begin at (0, 0, 0).
        initialized = True

    # TODO 8: compute relative odometry pose (x_odom, y_odom, yaw_odom).
    x_odom = 0.0
    y_odom = 0.0
    yaw_odom = 0.0

    # TODO 9: compute measured kinematics and accumulate travelled distance.
    vx_world = vy_world = speed = v_forward = v_lateral = yaw_rate = 0.0

    # -----------------------------------------------------------------------
    # Random navigation + obstacle avoidance
    # -----------------------------------------------------------------------
    values = [sensor.getValue() for sensor in ps]
    front_right = max(values[0], values[1])
    front_left = max(values[6], values[7])
    obstacle = max(front_left, front_right) > OBSTACLE_THRESHOLD

    if obstacle and t >= turn_until:
        # TODO 10: choose a turn away from the stronger obstacle side.
        # If both sides are similar, choose randomly.
        pass

    # Occasional random heading change even when the path is clear.
    if t >= next_random_turn and t >= turn_until:
        turn_direction = random.choice([-1.0, 1.0])
        turn_until = t + random.uniform(0.25, 0.70)
        next_random_turn = t + random.uniform(3.0, 6.0)

    if t < turn_until:
        w_left = -turn_direction * 2.5
        w_right = turn_direction * 2.5
    else:
        # Small random bias produces a wandering trajectory rather than a line.
        bias = random.uniform(-0.15, 0.15)
        w_left = BASE_SPEED - bias
        w_right = BASE_SPEED + bias

    w_left = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, w_left))
    w_right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, w_right))
    left_motor.setVelocity(w_left)
    right_motor.setVelocity(w_right)

    v_cmd, omega_cmd = command_kinematics(w_left, w_right)

    # -----------------------------------------------------------------------
    # Report once per 0.5 s
    # -----------------------------------------------------------------------
    if last_report_time < 0 or t - last_report_time >= 0.5:
        # TODO 11: print a concise one-line report containing at least:
        # time, odom x/y/yaw, GPS z, speed, v_forward, yaw_rate,
        # path_length, commanded v and commanded omega.
        print("TODO: odometry report")
        last_report_time = t

    # TODO 12: update previous pose for the next finite-difference step.
