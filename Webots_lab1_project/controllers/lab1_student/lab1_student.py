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
    sensor.enable(TIME_STEP)
    proximity_sensors.append(sensor)

MAX_SPEED = 6.28
CRUISE_SPEED = 0.55 * MAX_SPEED
OBSTACLE_THRESHOLD = 80.0

print("My first Webots controller is running")
while robot.step(TIME_STEP) != -1:
    # print(f"simulation time = {robot.getTime():.2f} s")

    values = [0.0] * 8
    values = [sensor.getValue() for sensor in proximity_sensors] 
    print([round(value, 1) for value in values])

    right_obstacle = max(values[0], values[1], values[2])
    left_obstacle = max(values[5], values[6], values[7])
    
    left_speed = CRUISE_SPEED
    right_speed = CRUISE_SPEED
    
    if left_obstacle > OBSTACLE_THRESHOLD or right_obstacle > OBSTACLE_THRESHOLD:
        if left_obstacle > right_obstacle:
            # Obstacle is stronger on the left: turn right. 
            left_speed = 0.45 * MAX_SPEED
            right_speed = -0.25 * MAX_SPEED
        else:
            # Obstacle is stronger on the right: turn left. 
            left_speed = -0.25 * MAX_SPEED
            right_speed = 0.45 * MAX_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    pass
