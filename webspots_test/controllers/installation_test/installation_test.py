from controller import Robot
import cv2
import numpy as np
import open3d as o3d
import scipy
import sklearn

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("The Webots controller API imported successfully.")
print("NumPy:", np.__version__)
print("OpenCV:", cv2.__version__)
print("Open3D:", o3d.__version__)
print("SciPy:", scipy.__version__)
print("Scikit-learn:", sklearn.__version__)
print("Basic timestep:", timestep, "ms")

steps = 0
while robot.step(timestep) != -1:
    steps += 1
    if steps == 1:
        print("The Webots simulation loop is running.")
    if steps >= 100:
        print("Installation test completed successfully.")
        break