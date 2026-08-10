"""Reference controller for Lab 1: Getting Started with Webots and Python.

Robot: e-puck
Behaviour: drive forward and turn away from nearby obstacles.
"""

from controller import Robot

# Create the Webots Robot object and obtain the world's basic time step.
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Retrieve the two wheel motors by their device names in the e-puck model.
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Velocity control: infinite target position means that motor speed is controlled
# directly using setVelocity().
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Enable the eight infrared proximity sensors (ps0 ... ps7).
proximity_sensors = []
for index in range(8):
    sensor = robot.getDevice(f"ps{index}")
    sensor.enable(TIME_STEP)
    proximity_sensors.append(sensor)

MAX_SPEED = 6.28       # Approximate maximum wheel angular velocity (rad/s).
CRUISE_SPEED = 0.55 * MAX_SPEED
OBSTACLE_THRESHOLD = 80.0
PRINT_EVERY_STEPS = 16
step_count = 0

print("Lab 1 reference controller started.")
print("The robot will drive forward and avoid obstacles.")

while robot.step(TIME_STEP) != -1:
    values = [sensor.getValue() for sensor in proximity_sensors]

    # Front-right sensors: ps0, ps1 and ps2.
    right_obstacle = max(values[0], values[1], values[2])
    # Front-left sensors: ps5, ps6 and ps7.
    left_obstacle = max(values[5], values[6], values[7])

    left_speed = CRUISE_SPEED
    right_speed = CRUISE_SPEED

    if left_obstacle > OBSTACLE_THRESHOLD or right_obstacle > OBSTACLE_THRESHOLD:
        # Turn away from the side reporting the stronger obstacle signal.
        if left_obstacle > right_obstacle:
            left_speed = 0.45 * MAX_SPEED
            right_speed = -0.25 * MAX_SPEED
        else:
            left_speed = -0.25 * MAX_SPEED
            right_speed = 0.45 * MAX_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    if step_count % PRINT_EVERY_STEPS == 0:
        print(
            f"front-left={left_obstacle:7.1f}, "
            f"front-right={right_obstacle:7.1f}, "
            f"wheel speeds=({left_speed:5.2f}, {right_speed:5.2f})"
        )
    step_count += 1
