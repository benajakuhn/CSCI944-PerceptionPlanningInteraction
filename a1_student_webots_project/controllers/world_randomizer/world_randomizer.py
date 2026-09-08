"""Supervisor that randomizes TARGET_BALL at every simulation start/reset.
Students do not need to modify this file.
"""
from controller import Supervisor
import math
import random
import time
from pathlib import Path
import numpy as np

robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())
random.seed(time.time_ns())

RES = 0.02
X_MIN = -6.187
Y_MIN = -2.92
BALL_Z = 0.18
MIN_START_DISTANCE = 1.0
PLACEMENT_CLEARANCE_M = 0.20

project_root = Path(__file__).resolve().parents[2]
grid = np.load(project_root / "break_room_occupancy_inflated_002m.npy")
ball = robot.getFromDef("TARGET_BALL")
robot_node = robot.getFromDef("M_PUCK")  # optional; DEF may not exist in student world

# The M-puck starts at (0,0); keep the target away from the initial pose.
start_xy = (0.0, 0.0)
clearance_cells = int(math.ceil(PLACEMENT_CLEARANCE_M / RES))

def cell_is_clear(r, c):
    if not (clearance_cells <= r < grid.shape[0]-clearance_cells and
            clearance_cells <= c < grid.shape[1]-clearance_cells):
        return False
    patch = grid[r-clearance_cells:r+clearance_cells+1,
                 c-clearance_cells:c+clearance_cells+1]
    return np.all(patch == 0)

def cell_to_world(r, c):
    x = X_MIN + (c + 0.5) * RES
    y = Y_MIN + (r + 0.5) * RES
    return x, y

free = np.argwhere(grid == 0)
rng = np.random.default_rng(time.time_ns())
chosen = None
for idx in rng.permutation(len(free)):
    r, c = free[idx]
    if not cell_is_clear(int(r), int(c)):
        continue
    x, y = cell_to_world(int(r), int(c))
    if math.hypot(x-start_xy[0], y-start_xy[1]) < MIN_START_DISTANCE:
        continue
    chosen = (x, y)
    break

if chosen is None:
    chosen = (1.0, 1.0)

ball.getField("translation").setSFVec3f([chosen[0], chosen[1], BALL_Z])
ball.resetPhysics()
print(f"[world_randomizer] target randomized to x={chosen[0]:.3f}, y={chosen[1]:.3f}")

while robot.step(TIME_STEP) != -1:
    pass
