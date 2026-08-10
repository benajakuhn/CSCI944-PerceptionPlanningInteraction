"""Student starter controller for Lab 1.

Complete the TODO sections during the laboratory.
"""

from controller import Robot

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

proximity_sensors = []
for index in range(8):
    sensor = robot.getDevice(f"ps{index}")
    # TODO 3: Enable each sensor using TIME_STEP.
    proximity_sensors.append(sensor)

MAX_SPEED = 6.28
CRUISE_SPEED = 0.55 * MAX_SPEED
OBSTACLE_THRESHOLD = 80.0

print("My first Webots controller is running")
while robot.step(TIME_STEP) != -1:
    # print(f"simulation time = {robot.getTime():.2f} s")
    # TODO 4: Read all eight sensor values.
    values = [0.0] * 8

    # TODO 5: Compute obstacle strengths on the right and left sides.
    right_obstacle = 0.0
    left_obstacle = 0.0

    #left_speed = CRUISE_SPEED
    #right_speed = CRUISE_SPEED
    
    left_speed = -2.0
    right_speed = 2.0
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # TODO 6: If an obstacle is detected, turn away from the stronger signal.

    # TODO 7: Send the calculated speeds to the wheel motors.
    pass
