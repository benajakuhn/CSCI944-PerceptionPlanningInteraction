"""Instructor utility: randomise the red ball at each simulation/controller start."""
from controller import Supervisor
import random

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
ball = robot.getFromDef("RED_BALL")

# Pre-selected floor locations chosen to give varied visibility and obstacle layouts.
# Instructors may adjust these after inspecting their exact break_room revision.
SAFE_POSITIONS = [
    (-5.20,  3.75, 0.18),
    (-4.90,  1.00, 0.18),
    (-4.70, -1.60, 0.18),
    (-1.40,  3.85, 0.18),
    ( 0.80,  3.80, 0.18),
    ( 1.65,  1.45, 0.18),
    ( 0.85, -1.55, 0.18),
    ( 4.70, -2.10, 0.18),
    ( 5.30,  0.85, 0.18),
    ( 5.05,  3.55, 0.18),
]

if ball is None:
    print("[ball_randomizer] ERROR: DEF RED_BALL not found.")
else:
    xyz = random.choice(SAFE_POSITIONS)
    ball.getField("translation").setSFVec3f(list(xyz))
    ball.resetPhysics()
    print(f"[ball_randomizer] Red ball placed at x={xyz[0]:.2f}, y={xyz[1]:.2f} m")

# Keep the supervisor alive without changing the ball again.
while robot.step(timestep) != -1:
    pass
