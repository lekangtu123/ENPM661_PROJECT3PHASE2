import math
import numpy as np  
from queue import PriorityQueue  

# Map and obstacles
MAP_W, MAP_H = 540, 300   
ROBOT_R_CM    = 10.5      
CLEARANCE_CM  = 1.0       


# Each element: ((xmin, xmax), (ymin, ymax)), with origin at bottom-left
OBSTACLES_CM = [
    ((100, 110), (0, 200)),  
    ((200, 210), (100, 300)), 
    ((300, 310), (0, 100)),   
    ((300, 310), (200, 300)), 
    ((400, 410), (0, 200)),   
]

def _bottom_left(p):
    # FalconSim uses bottom-left as (0,0)
    return p[0], MAP_H - p[1]

def _inside_rect(x, y, rect):
    # Check if the point (x, y) is inside the given rectangle.
    (xmin, xmax), (ymin, ymax) = rect
    return xmin <= x <= xmax and ymin <= y <= ymax

def is_valid(node, robot_r=ROBOT_R_CM, clr=CLEARANCE_CM):
    # Validate if node (x, y, theta, *, *) lies in the free (valid) space.
    x, y = _bottom_left(node[:2])  # Remap coordinates to bottom-left origin

    # Check boundaries with respect to robot radius and clearance
    if (x < robot_r + clr or x > MAP_W - robot_r - clr or
        y < robot_r + clr or y > MAP_H - robot_r - clr):
        return False

    # Check if the node is inside any expanded obstacle region
    for rect in OBSTACLES_CM:
        exp_rect = ((rect[0][0] - robot_r - clr, rect[0][1] + robot_r + clr),
                    (rect[1][0] - robot_r - clr, rect[1][1] + robot_r + clr))
        if _inside_rect(x, y, exp_rect):
            return False
    return True

# KE part
def _neighbors(state, rpm_l, rpm_r, dt=0.01):
    """
    For the given state (x, y, theta), simulate 1 second of motion using
    8 sets of wheel speeds (rpm1, rpm2) and return a list of valid neighbor nodes.
    Each neighbor is returned as (x, y, theta, cost, (rpm1, rpm2)).
    """
    Xi, Yi, Thetai = state
    R, L = 3.3, 28.7         
    acts = np.array([
        [rpm_r, rpm_r],
        [0, rpm_l],
        [rpm_l, 0],
        [rpm_l, rpm_l],
        [0, rpm_r],
        [rpm_r, 0],
        [rpm_l, rpm_r],
        [rpm_r, rpm_l]
    ])
    nbrs = []
    for rpm1, rpm2 in acts:
        # Convert RPM to angular velocities in rad/s
        w1, w2 = rpm1 * math.pi / 30, rpm2 * math.pi / 30
        # Calculate linear velocity v and angular velocity w
        v      = R / 2 * (w1 + w2)
        w      = R / L * (w2 - w1)

        x, y, th = Xi, Yi, math.radians(Thetai)
        cost, t  = 0.0, 0.0
        while t < 1.0:
            # Incrementally update position and orientation
            x_n, y_n = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt
            th      += w * dt
            # Break early if the new position is invalid
            if not is_valid((x_n, y_n, 0, 0, 0)):
                break
            # Accumulate cost as the Euclidean distance traveled
            cost += math.hypot(x_n - x, y_n - y)
            x, y  = x_n, y_n
            t    += dt
        else:  # If no collision occurred within 1 second
            nbrs.append((round(x, 3), round(y, 3), round(math.degrees(th) % 360, 3),
                         round(cost, 3), (rpm1, rpm2)))
    return nbrs

# A* alogrithm
def _euclid(p1, p2):
    # Compute Euclidean distance between two points
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def _a_star(start, goal, thr, rpm1, rpm2):
    # A* search to find path from start to goal within a threshold
    pq, g_cost, parent = PriorityQueue(), {}, {}
    pq.put((0, start))
    g_cost[(int(start[0]), int(start[1]))] = 0
    parent[(int(start[0]), int(start[1]))] = None

    while not pq.empty():
        _, cur = pq.get()
        # Check if current node is close enough to the goal
        if _euclid(cur, goal) <= thr:
            break
        for nxt in _neighbors(cur, rpm1, rpm2):
            nxy = (int(nxt[0]), int(nxt[1]))
            tentative = g_cost[(int(cur[0]), int(cur[1]))] + nxt[3]
            if nxy not in g_cost or tentative < g_cost[nxy]:
                g_cost[nxy] = tentative
                f = tentative + 10 * _euclid(nxt, goal)  # f = g + h
                pq.put((f, nxt[:3]))
                parent[nxy] = (int(cur[0]), int(cur[1]), nxt[4])

    # Backtrack to build the path from goal to start
    path, cur_i = [], (int(cur[0]), int(cur[1]))
    while parent[cur_i]:
        px, py, act = parent[cur_i]
        path.append(((cur_i[0], cur_i[1], 0), act))
        cur_i = (px, py)
    path.reverse()
    return path

def plan_path(start_cm, goal_cm,
              robot_r_cm, clr_cm, dt,
              goal_thr_cm, wheel_r_cm, wheel_base_cm,
              rpm1, rpm2, obstacles=None):
    """
    Parameters
    ----------
    start_cm, goal_cm : tuple
        (x, y, theta) in centimeters, with bottom-left as (0,0).
    robot_r_cm, clr_cm : float
        Robot radius and desired clearance in cm.
    dt : float
        Discrete time step (seconds); not used internally (fixed dt=0.01 s in algorithm).
    goal_thr_cm : float
        Threshold to consider goal reached in cm.
    wheel_r_cm, wheel_base_cm : float
        Wheel radius and wheel base in cm; parameters are placeholders.
    rpm1, rpm2 : float
        The two RPMs used in the discrete action set for left and right wheels.
    obstacles : ignored
        Obstacles are defined in the constant OBSTACLES_CM; parameter retained for interface consistency.

    Returns
    -------
    List[List[dx, dy, dtheta]]
        An incremental path as a list of [dx, dy, dtheta] increments,
        where dx/dy are in cm and dtheta is in radians.
        For example: [[8, 0, 0.0], [14, 14, 0.7854], …]
    """
    # Record global parameters
    global ROBOT_R_CM, CLEARANCE_CM
    ROBOT_R_CM   = robot_r_cm
    CLEARANCE_CM = clr_cm


    # Call A* to obtain an absolute coordinate path 
    thr_cm = max(1, int(goal_thr_cm))  # At least 1 cm
    raw_path = _a_star(start_cm, goal_cm, thr_cm, rpm1, rpm2)
    # Each element in raw_path is ((x, y, theta), action)
    poses = [raw_path[0][0]] if raw_path else [start_cm]  # Ensure non-empty path
    for item in raw_path[1:]:
        poses.append(item[0])

    # Convert absolute coordinates to incremental changes [dx, dy, dθ] (dθ in radians)
    deltas, last = [], poses[0]
    for nxt in poses[1:]:
        dx = nxt[0] - last[0]
        dy = nxt[1] - last[1]
        dtheta_deg = (nxt[2] - last[2] + 540) % 360 - 180
        dtheta_rad = math.radians(dtheta_deg)
        deltas.append([dx, dy, dtheta_rad])
        last = nxt

    # Output incremental path to terminal
    formatted = [[round(d[0], 2), round(d[1], 2), round(d[2], 4)] for d in deltas]
    import rclpy.logging
    rclpy.logging.get_logger('astar_planner').info(
        f'A* incremental path (cm, rad): {formatted}')
    

    return deltas

