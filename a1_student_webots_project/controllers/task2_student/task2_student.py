"""PPI Assignment - Task 2 student skeleton.

Task 2: A* planning on the supplied inflated occupancy grid, followed by
closed-loop GPS/IMU waypoint navigation to the randomized green ball.

Unlike Task 1, Task 2 IS ALLOWED to use the green ball's ground-truth position.
The M-puck therefore runs as a Supervisor in the supplied Task 2 world. A
ready-to-use get_ground_truth_ball_location() helper is provided below.

The separate world_randomizer Supervisor controller is also supplied to
students. It randomizes TARGET_BALL at each simulation start/reset. Students do
not need to write or modify that controller.
"""

from controller import Supervisor
from pathlib import Path
import heapq
import math

import matplotlib.pyplot as plt
import numpy as np


# =============================================================================
# 1. SUPERVISOR, ROBOT DEVICES AND SUPPLIED MAP - PROVIDED
# =============================================================================

# Task 2 uses Supervisor because reading TARGET_BALL ground truth is explicitly
# permitted and required by the task specification.
robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# GPS provides the robot's current world position. The inertial unit provides
# orientation (yaw) for closed-loop path following.
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

MAX_SPEED = 6.28

# The occupancy map is pre-generated and already inflated for the M-puck. Map
# generation/inflation is NOT part of the assignment. 0 means traversable/free;
# non-zero values represent occupied/inflated cells.
RES = 0.02
X_MIN = -6.187
Y_MIN = -2.920

project_root = Path(__file__).resolve().parents[2]
grid = np.load(project_root / "break_room_occupancy_inflated_002m.npy")


# =============================================================================
# 2. SUPERVISOR / GROUND-TRUTH HELPERS - PROVIDED
# =============================================================================

def get_ground_truth_ball_location():
    """Return the current TARGET_BALL ground-truth world position (x, y).

    TARGET_BALL is given a DEF name in the supplied world. Supervisor.getFromDef
    obtains that node, and its translation field contains [x, y, z] in metres.

    This function is PROVIDED for Task 2 and may be called directly. Do not copy
    this ground-truth approach into Task 1, where it is prohibited.
    """
    target = robot.getFromDef("TARGET_BALL")
    if target is None:
        raise RuntimeError("TARGET_BALL DEF was not found in the Task 2 world")

    translation = target.getField("translation").getSFVec3f()
    return translation[0], translation[1]


def get_robot_xy():
    """Return the M-puck's current GPS world position (x, y)."""
    p = gps.getValues()
    return p[0], p[1]


def get_robot_yaw():
    """Return the M-puck yaw angle in radians."""
    return imu.getRollPitchYaw()[2]


def set_speed(left, right):
    """Set wheel speeds while respecting the M-puck motor limit."""
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))


def wrap_angle(angle):
    """Wrap an angular error to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# =============================================================================
# 3. WORLD <-> OCCUPANCY-GRID CONVERSION - PROVIDED
# =============================================================================

def world_to_grid(x, y):
    """Convert Webots world coordinates (metres) to grid (row, column)."""
    col = int(math.floor((x - X_MIN) / RES))
    row = int(math.floor((y - Y_MIN) / RES))
    return row, col


def grid_to_world(row, col):
    """Return the world x,y coordinates of the centre of a grid cell."""
    x = X_MIN + (col + 0.5) * RES
    y = Y_MIN + (row + 0.5) * RES
    return x, y


def cell_is_free(occ, cell):
    """Check that a (row,col) cell is in bounds and traversable."""
    row, col = cell
    return (0 <= row < occ.shape[0] and
            0 <= col < occ.shape[1] and
            occ[row, col] == 0)


def nearest_free(occ, cell):
    """Optional helper: snap an occupied/out-of-bounds cell to nearby free space.

    TODO: implement a documented method if you choose to use this helper. The
    randomizer normally places the target in free space, but discretisation can
    still make robust validation useful.
    """
    # TODO: search outward from cell and return a valid free (row,col).
    return cell


# =============================================================================
# 4. A* PATH PLANNING - STUDENT IMPLEMENTATION
# =============================================================================

def astar(occ, start, goal):
    """Return a collision-free path [(row,col), ...] from start to goal.

    Recommended design, matching the assignment specification:
      * 8-connected neighbourhood;
      * cost 1 for horizontal/vertical moves;
      * cost sqrt(2) for diagonal moves;
      * Euclidean heuristic;
      * heapq priority queue storing f = g + h;
      * came-from/predecessor dictionary for reconstruction;
      * reject occupied cells;
      * if diagonals are allowed, prevent corner-cutting by checking the two
        adjacent cardinal cells.

    TODO: implement A* explicitly. Do not call an external path-planning library.
    Return [] when no path exists.
    """
    # make sure start and goal are free
    if not cell_is_free(occ, start) or not cell_is_free(occ, goal):
        return []

    # helper function to compute heuistic value of a cell
    def heuristic(cell):
        # Euclidean distance between cell and goal
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    # create an open list
    open_list = []
    # store it as a min heap (f(n), g(n), cell) - is sorted by lowest f first
    heapq.heappush(open_list, (heuristic(start), 0.0, start))

    # create a closed list
    closed_list = set()

    # cost lookup table
    g_cost = {start: 0.0}

    # predecessor dictionary for path reconstruction
    came_from = {}

    # possible moves and their costs
    moves = [
        # horizontal/vertical, cost = 1.0
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        # diagonal: cost = sqrt(2)
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]

    while open_list:
        # get node with lowest f(n)
        f_curr, g_curr, curr = heapq.heappop(open_list)

        # if the node is already closed, skip it
        if curr in closed_list:
            continue

        # goal has been reached, reconstruct path and return it
        if curr == goal:
            path = []
            # path reconstruction
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            # reverse path to represent the robots route
            path.reverse()
            return path

        # add curr to closed list
        closed_list.add(curr)

        # explore neighbors
        for dr, dc, cost in moves:
            neighbor = (curr[0] + dr, curr[1] + dc)

            # check if the neighbor is valid (free and in bounds)
            if not cell_is_free(occ, neighbor):
                continue

            # check if a neighbor is already closed
            if neighbor in closed_list:
                continue

            # check if there is no diagonal cutting
            if dr != 0 and dc != 0:
                if not cell_is_free(occ, (curr[0] + dr, curr[1])) or not cell_is_free(occ, (curr[0], curr[1] + dc)):
                    continue

            # tentative g score to reach this neighbor via curr
            tentative_g = g_curr + cost

            # if we have a better path to this neighbor, update it
            if tentative_g <  g_cost.get(neighbor, float('inf')):
                # update in cost lookup table
                g_cost[neighbor] = tentative_g
                # calculate f(n) = g(n) + h(n)
                f = tentative_g + heuristic(neighbor)
                # update in open list
                heapq.heappush(open_list, (f, tentative_g, neighbor))
                # update predecessor
                came_from[neighbor] = curr
    return []


def compress_path(path):
    """Optional helper to reduce a dense cell-by-cell path to turning points.

    The raw A* route can contain a waypoint every 0.02 m. Following every cell
    is unnecessary. The reference solution retains the first cell, direction-
    change cells, and the final cell.

    TODO: implement this optional path reduction, or justify another waypoint
    selection/smoothing strategy.
    """
    return path


# =============================================================================
# 5. PATH VISUALISATION - STUDENT IMPLEMENTATION
# =============================================================================

def save_path_png(occ, path, start, goal, filename="planned_path.png"):
    """Save the occupancy map, start, goal and planned path to a PNG file.

    The figure should be easy to interpret and should correspond to the CURRENT
    randomized run. Metric x/y axes or an equally clear coordinate convention
    should be shown.

    Hints:
      * plt.imshow(..., origin='lower', cmap='gray_r', extent=[...]) lets the
        plot axes be expressed directly in Webots metres;
      * convert path cells to world coordinates using grid_to_world();
      * clearly mark start and goal.

    TODO: create and save the required path visualisation.
    """
    pass


# =============================================================================
# 6. CLOSED-LOOP GPS/IMU PATH FOLLOWING - STUDENT IMPLEMENTATION
# =============================================================================

def follow_path(path_xy):
    """Drive through world-coordinate waypoints until the green ball is reached.

    At each simulation step, a simple feedback controller can compute:

        desired_heading = atan2(y_waypoint - y_robot,
                                x_waypoint - x_robot)
        heading_error   = wrap_angle(desired_heading - yaw)

    For a large heading error, turning in place is often more reliable. For a
    smaller error, drive forward while applying differential steering. Advance
    to the next waypoint when the robot is within a chosen tolerance.

    The final tolerance should take into account the ball radius (0.18 m) and
    the robot geometry rather than requiring the robot centre to occupy exactly
    the ball-centre grid cell.

    TODO:
      * implement GPS/IMU feedback control;
      * choose/document steering gains and waypoint tolerances;
      * stop the motors at success;
      * keep calling robot.step(TIME_STEP) while following the route.
    """
    pass


# =============================================================================
# 7. MAIN TASK 2 SEQUENCE - STRUCTURE PROVIDED
# =============================================================================

# world_randomizer is a separate supplied Supervisor controller. Give it a few
# simulation steps to place TARGET_BALL before reading the target translation.
for _ in range(5):
    if robot.step(TIME_STEP) == -1:
        raise SystemExit

# Read the robot start pose and target ground truth using the supplied helpers.
start_xy = get_robot_xy()
goal_xy = get_ground_truth_ball_location()

print(f"Task 2 start:  x={start_xy[0]:.3f}, y={start_xy[1]:.3f}")
print(f"Task 2 target: x={goal_xy[0]:.3f}, y={goal_xy[1]:.3f}")

# Convert continuous Webots positions to discrete planning cells.
start_rc = world_to_grid(*start_xy)
goal_rc = world_to_grid(*goal_xy)

# TODO: validate start_rc and goal_rc. If either is not free, apply and explain
# a nearest-free-cell policy rather than allowing A* to start/end in an obstacle.

path_rc = astar(grid, start_rc, goal_rc)
if not path_rc:
    set_speed(0.0, 0.0)
    print("No path found")
    raise SystemExit

# The PNG must show the route from this randomized run.
save_path_png(grid, path_rc, start_rc, goal_rc)

# Optionally reduce the dense grid route before converting it to world-space
# navigation waypoints.
waypoint_cells = compress_path(path_rc)
path_xy = [grid_to_world(r, c) for r, c in waypoint_cells]

# Execute the planned route using feedback from GPS and IMU.
follow_path(path_xy)
set_speed(0.0, 0.0)
