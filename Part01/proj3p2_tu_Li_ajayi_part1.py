# Github link: https://github.com/lekangtu123/ENPM661_PROJECT3PHASE2

import numpy as np
import cv2
import math
from queue import PriorityQueue

# Function to map coordinates to the bottom left of the image
def map_to_bottom_left(point):
    # Define image dimensions (height and width)
    height, width = 600, 1080
    # x-coordinate remains the same
    bottom_left_x = point[0]
    # Invert y-coordinate because origin is at the top left by default
    bottom_left_y = height - point[1]
    # Return the mapped coordinates
    return (bottom_left_x, bottom_left_y)

# Function to check if a point is a valid neighbor
def is_valid_neighbor(point):
    global Robot_radius
    global clearance
    global width, height

    # Unpack the point tuple; additional parameters are not used here
    x, y, _, _, _ = point

    # Map the point's coordinate system
    x, y = map_to_bottom_left((x, y))

    # Check if the point is within the valid boundary considering robot size and clearance
    if (x < clearance + Robot_radius 
        or x > width - (clearance + Robot_radius) 
        or y < clearance + Robot_radius 
        or y > height - (clearance + Robot_radius)):
        return False

    # Check rectangle obstacle 1: x in [200,220], y in [0,400]
    if ( x >= (200 - clearance - Robot_radius) and 
         x <= (220 + clearance + Robot_radius) and
         y >= 0 and 
         y <= (400 + clearance + Robot_radius) ):
        return False

    # Check rectangle obstacle 2: x in [400,420], y in [200,600]
    if ( x >= (400 - clearance - Robot_radius) and 
         x <= (420 + clearance + Robot_radius) and
         y >= (200 - clearance - Robot_radius) and
         y <= (600 + clearance + Robot_radius) ):
        return False

    # Check rectangle obstacle 3: x in [600,620], y in [0,200]
    if ( x >= (600 - clearance - Robot_radius) and
         x <= (620 + clearance + Robot_radius) and
         y >= 0 and
         y <= (200 + clearance + Robot_radius) ):
        return False

    # Check rectangle obstacle 4: x in [600,620], y in [400,600]
    if ( x >= (600 - clearance - Robot_radius) and
         x <= (620 + clearance + Robot_radius) and
         y >= (400 - clearance - Robot_radius) and
         y <= (600 + clearance + Robot_radius) ):
        return False

    # Check rectangle obstacle 5: x in [800,820], y in [0,400]
    if ( x >= (800 - clearance - Robot_radius) and
         x <= (820 + clearance + Robot_radius) and
         y >= 0 and
         y <= (400 + clearance + Robot_radius) ):
        return False

    # Check bottom boundary
    if y <= (clearance + Robot_radius):
        return False
    # Check top boundary
    if y >= (600 - (clearance + Robot_radius)):
        return False
    # Check left boundary
    if x <= (clearance + Robot_radius):
        return False
    # Check right boundary
    if x >= (1080 - (clearance + Robot_radius)):
        return False
    # Return True if all conditions are met
    return True

# Function to get neighboring points for a given point on the grid
def get_neighbors(point, obstacles, r1, r2):
    global print_interval

    # Define constants for robot motion
    R = 33/5  # Radius of the wheels
    L = 287/5  # Distance between the wheels (wheelbase)
    dt = 0.01  # Time step for simulation
    neighbors_l = []  # List to store valid neighbor points
    Xi, Yi, Thetai = point  # Initial robot state

    R_over_2 = R / 2
    R_over_L = R / L

    # Define actions as pairs of wheel RPMs
    actions = np.array([[r2, r2], [0, r1], [r1, 0], [r1, r1], [0, r2], [r2, 0], [r1, r2], [r2, r1]])
    for action in actions:
        # Get wheel RPMs for the current action
        rpm1, rpm2 = action
        # Convert RPMs to angular velocities (rad/s)
        omega1 = rpm1 * (np.pi / 30)
        omega2 = rpm2 * (np.pi / 30)

        # Calculate linear velocity and angular velocity
        v = R_over_2 * (omega1 + omega2)
        omega = R_over_L * (omega2 - omega1)
        t = 0  # Initialize time
        a = Xi
        b = Yi
        # Set initial state values (x, y, theta)
        x, y, theta = Xi, Yi, Thetai * np.pi / 180
        cost = 0  # Initialize cost for this action
        flag = 0  # Flag to indicate if path is invalid
        i_n = []  # List to store new coordinates
        i_n2 = []  # List to store previous coordinates

        # Simulate motion for 1 second in increments of dt
        while True:
            if t >= 1.0:
                break
            # Update heading
            theta = theta + (omega * dt)

            # Calculate increments for x and y
            cos_theta_dt = np.cos(theta) * dt
            sin_theta_dt = np.sin(theta) * dt

            # Save current coordinates before updating
            a = x
            b = y
            # Update x and y using motion equations
            x = x + v * cos_theta_dt
            y = y + v * sin_theta_dt
            # Append the floor values of new positions to the list
            i_n.append((int(math.floor(x)), int(math.floor(y))))
            i_n2.append((int(math.floor(a)), int(math.floor(b))))

            # Check if the new point is valid; if not, mark flag and exit loop
            if is_valid_neighbor((x, y, 0, 0, 0)) == False:
                flag = 1
                break
            t += dt
            # Accumulate cost using Euclidean distance between successive points
            if rpm1 == rpm2:
                cost += euclidean_distance((a, b), (x, y))
            else:
                cost += euclidean_distance((a, b), (x, y))

        # If path is valid, draw the segment on the obstacle map
        if flag == 0:
            for i in range(len(i_n)):
                cv2.line(obstacle_map, i_n2[i], i_n[i], (0, 0, 0), 1)
        # Create a neighbor tuple with position, orientation, cost, and action used
        neighbor = (round(x, 3), round(y, 3), round(np.degrees(theta) % 360, 3), round(cost, 3), (action[0], action[1]))

        # Check if neighbor is valid and no invalid flag was raised
        if is_valid_neighbor(neighbor) and flag == 0:
            # Display the map at set intervals
            if print_interval % 1000 == 0:
                cv2.imshow("Shortest Path", obstacle_map)
                out.write(obstacle_map)
                cv2.waitKey(1)
            # Append the valid neighbor to the list
            neighbors_l.append(neighbor)

    # Return list of valid neighbors
    return neighbors_l

# Function to draw a line on the image
def draw_line(img, start_point, end_point, color, thickness):
    # Draw a line on the image from start_point to end_point using OpenCV
    cv2.line(img, start_point, end_point, color, thickness)

# Function to draw obstacles using half-plane equations
def draw_obstacles(obstacle_map, obstacles):
    # Iterate over each obstacle in the list
    for obstacle in obstacles:
        shape = obstacle.get('shape')
        # Check if the shape is a rectangle
        if shape == 'rectangle':
            color = obstacle.get('color', (0, 0, 0))  # Set default color as black
            thickness = obstacle.get('thickness', 1)  # Set default thickness as 1
            vertices = obstacle['vertices']
            # Draw the edges of the rectangle
            for i in range(len(vertices)):
                draw_line(obstacle_map, map_to_bottom_left(vertices[i]), map_to_bottom_left(vertices[(i + 1) % len(vertices)]), color, thickness)
            # Fill the rectangle with the specified color
            cv2.fillPoly(obstacle_map, np.array([[map_to_bottom_left(point) for point in vertices]]), color)

        # Check if the shape is a circle
        if shape == 'circle':
            vertices = obstacle['vertices']
            center, radius = vertices[0], vertices[1]
            color = obstacle.get('color', (0, 0, 0))  # Default color is black
            thickness = obstacle.get('thickness', -1)  # Fill the circle if thickness is -1
            # Draw the circle on the map
            cv2.circle(obstacle_map, map_to_bottom_left(center), radius, color, thickness)

# Function to calculate the Euclidean distance between two points
def euclidean_distance(p1, p2):
    # Compute the Euclidean distance using the square root of the sum of squared differences
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Function to ask for start point input and validate it against the obstacle map
def ask_for_start_point(message, default=None):
    while True:
        # Request user input for starting point with default value prompt
        user_input = input(f"{message} (default: {default[0]},{default[1]},{default[2]}): ")
        default[0], default[1] = default[0]/5, default[1]/5
        # Use default if no input provided
        if user_input.strip() == "":
            if default is None:
                raise ValueError("No default value provided.")
            else:
                x, y, theta = default
                x, y = map_to_bottom_left((x, y))
        else:
            # Parse the user input
            x, y, theta = map(int, user_input.split(','))
            x, y = map_to_bottom_left((x, y))

        # Check if input is within boundaries and valid with the required angle constraints
        if 0 <= x < width and 0 <= y < height and is_valid_neighbor((x, y, 0, 0, 0)) and theta % 30 == 0:
            return x, y, theta
        elif theta % 30 != 0:
            print("Enter angle in multiples of 30 degrees")
        else:
            print("Point is invalid.")

# Function to ask for goal point input and validate it
def ask_for_goal_point(message, default=None):
    while True:
        # Request user input for goal point with default value prompt
        user_input = input(f"{message} (default: {default[0]},{default[1]}: ")
        default[0], default[1] = default[0]/5, default[1]/5
        # Use default if no input provided
        if user_input.strip() == "":
            if default is None:
                raise ValueError("No default value provided.")
            else:
                x, y = default
                x, y = map_to_bottom_left((x, y))
        else:
            # Parse the user input
            x, y = map(int, user_input.split(','))
            x, y = map_to_bottom_left((x, y))
            
        # Check if the goal point is within valid region
        if 0 <= x < width and 0 <= y < height and is_valid_neighbor((x, y, 0, 0, 0)):
            return x, y
        else:
            print("Point is invalid.")

# Function to ask for RPM values and validate them as positive integers
def ask_for_rpm(message, default=None):
    while True:
        # Request user input for RPM values with default value prompt
        user_input = input(f"{message} (default: {default[0]},{default[1]}): ")
        # Use default if no input provided
        if user_input.strip() == "":
            if default is None:
                raise ValueError("No default value provided.")
            else:
                rpm1, rpm2 = default
        else:
            # Parse the input RPM values
            rpm1, rpm2 = map(int, user_input.split(','))

        # Validate that both RPMs are positive
        if rpm1 > 0 and rpm2 > 0:
            return rpm1, rpm2
        else:
            print("Enter positive values for RPMs.")

# Function to ask for clearance value input and validate it
def ask_clearence(message, default=None):
    # Function to ask for clearance with additional input prompt
    def ask_clearence(message, default=None):
        while True:
            user_input = input(f"{message} (default: {default}): ")
            # If user provides no input, use default value
            if not user_input:
                return default
            try:
                clearance = int(user_input)
                if clearance > 0:
                    return clearance
                else:
                    print("Enter a positive value for clearance")
            except ValueError:
                print("Invalid input. Please enter a number or press Enter for default.")
    print("Click ENTER for entering default value ")
    while True:
        user_input = input(f"{message} (default: {default}): ")
        # Use default if no input is provided
        if not user_input:
            return default
        try:
            clearance = int(user_input)
            if clearance > 0:
                return clearance
            else:
                print("Enter a positive value for clearance")
        except ValueError:
            print("Invalid input. Please enter a number or press Enter for default.")

# A* algorithm to find the shortest path from start to goal
def a_star(start, goal, obstacles, threshold, rpm1, rpm2):
    # Priority queue for the frontier nodes; lower cost nodes have higher priority
    frontier = PriorityQueue()
    frontier.put((0, start))

    # Dictionaries to track cost so far and the path taken
    cost_so_far = {(start[0], start[1]): 0}
    came_from = {(start[0], start[1]): None}

    # Main loop of the A* algorithm
    while not frontier.empty():
        current_cost, current_node = frontier.get()

        # Check if current node is within threshold of the goal
        if (current_node[0] > goal[0] - threshold and current_node[0] < goal[0] + threshold) and (current_node[1] > goal[1] - threshold and current_node[1] < goal[1] + threshold):
            print("Goal Threshold reached orientation: " + "(" + str(current_node[0]) + "," + str(width - current_node[1]) + "," + str(360 - current_node[2]) + ")")
            break

        # Expand neighbors of the current node
        for next_node_with_cost in get_neighbors(current_node, obstacles, rpm1, rpm2):
            next_node = next_node_with_cost[:3]
            current_node_int = (int(current_node[0]), int(current_node[1]))
            # Calculate new cost to reach neighbor
            new_cost = cost_so_far[current_node_int] + next_node_with_cost[3]
            new_cost_check = new_cost + 10 * euclidean_distance(next_node, goal)
            next_node_int = (int(next_node[0]), int(next_node[1]))
            
            # If this path to neighbor is better, record it
            if next_node_int not in cost_so_far or new_cost_check < cost_so_far[(int(next_node[0]), int(next_node[1]))]:
                cost_so_far[(int(next_node[0]), int(next_node[1]))] = new_cost
                priority = round(new_cost + 10 * euclidean_distance(next_node, goal), 3)  # f = g + h
                frontier.put((priority, next_node))
                # Record where the neighbor came from, including the action taken
                came_from[(int(math.floor(next_node[0])), int(math.floor(next_node[1])))] = (int(current_node[0]), int(current_node[1]), next_node_with_cost[4])
    path = []
    start_int = (int(start[0]), int(start[1]))
    print("Start: ", start_int)
    print("Current Node Int: ", current_node_int)
    # Reconstruct the path by backtracking from the goal to the start
    while True:
        current_node_int = (int(current_node[0]), int(current_node[1]))
        if current_node_int == start_int:
            break
        path.append(((int(current_node[0]), int(current_node[1])), came_from[(int(current_node[0]), int(current_node[1]))]))
        current_node = came_from[(int(current_node[0]), int(current_node[1]))]
    
    path.reverse()  # Reverse the path to start from the beginning
    print("Path: ", path)

    # Return the computed shortest path
    return path

# Define image dimensions
width = 1080
height = 600

# Create a blank image filled with white color
obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255

# Ask the user for clearance input and convert it to the proper scale
clearance = ask_clearence("Enter clearance in mm: ", (55))
clearance = int(clearance / 5)
Robot_radius = 210 / 5  # Set robot radius

path_interval = 0

# Define obstacles with their shapes, vertices, and styles
obstacles = [
    # Rectangle obstacle 1
    {'shape': 'rectangle', 'vertices': [(200 - clearance, 0), (200 - clearance, 400 + clearance), (220 + clearance, 400 + clearance), (220 + clearance, 0)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(200, 0), (200, 400), (220, 400), (220, 0)], 'color': (0, 0, 0), 'thickness': 1},
    
    # Rectangle obstacle 2
    {'shape': 'rectangle', 'vertices': [(400 - clearance, 200 - clearance), (400 - clearance, 600 + clearance), (420 + clearance, 600 + clearance), (420 + clearance, 200 - clearance)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(400, 200), (400, 600), (420, 600), (420, 200)], 'color': (0, 0, 0), 'thickness': 1},
    
    # Rectangle obstacle 3
    {'shape': 'rectangle', 'vertices': [(600 - clearance, 0), (600 - clearance, 200 + clearance), (620 + clearance, 200 + clearance), (620 + clearance, 0)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(600, 0), (600, 200), (620, 200), (620, 0)], 'color': (0, 0, 0), 'thickness': 1},
    
    # Rectangle obstacle 4
    {'shape': 'rectangle', 'vertices': [(600 - clearance, 400 - clearance), (600 - clearance, 600 + clearance), (620 + clearance, 600 + clearance), (620 + clearance, 400 - clearance)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(600, 400), (600, 600), (620, 600), (620, 400)], 'color': (0, 0, 0), 'thickness': 1},
    
    # Rectangle obstacle 5
    {'shape': 'rectangle', 'vertices': [(800 - clearance, 0), (800 - clearance, 400 + clearance), (820 + clearance, 400 + clearance), (820 + clearance, 0)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(800, 0), (800, 400), (820, 400), (820, 0)], 'color': (0, 0, 0), 'thickness': 1},
    
    # Four lines rectangle obstacle 
    {'shape': 'rectangle', 'vertices': [(0, 0), (0, clearance), (1080, clearance), (1080, 0)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(0, 0), (0, 600), (clearance, 600), (clearance, 0)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(1080 - clearance, 0), (1080 - clearance, 600), (1080, 600), (1080, 0)], 'color': (128, 128, 128), 'thickness': 1},
    {'shape': 'rectangle', 'vertices': [(0, 600 - clearance), (0, 600), (1080, 600), (1080, 600 - clearance)], 'color': (128, 128, 128), 'thickness': 1},
]
print_interval = 0

# Draw obstacles on the obstacle map
draw_obstacles(obstacle_map, obstacles)

# Ask user to input the start and goal points
start = ask_for_start_point("Enter start point (x, y,theta): ", [300, 1500, 0])
goal = ask_for_goal_point("Enter goal point (x, y,theta): ", [5000, 1800])

# Ask user to input the RPM values for the wheels
rpm1, rpm2 = ask_for_rpm("Enter RPM1 and RPM2 separated by comma: ", (50, 100))

# Mark the goal on the obstacle map with a small red circle
cv2.circle(obstacle_map, (int(goal[0]), int(goal[1])), 3, (0, 0, 255), -1)

# Setup video writer to save the shortest path visualization
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('shortestpath.mp4', fourcc, 60.0, (width, height))

# Draw goal threshold circle for visualizing reachability
threshold = int((0.5 * Robot_radius))
cv2.circle(obstacle_map, (int(goal[0]), int(goal[1])), int(threshold), (0, 0, 255), 1)

# Run A* algorithm to compute the shortest path
shortest_path = a_star(start, goal, obstacles, threshold, rpm1, rpm2)

theta_draw = 0
# Mark the shortest path on the obstacle map
for point in shortest_path:
    # Define robot motion parameters for drawing path
    R = 33/5  # Wheel radius (scaled)
    L = 287/5  # Wheelbase (scaled)
    dt = 0.01  # Time step
    R_over_2 = R / 2
    R_over_L = R / L
    rpm1, rpm2 = point[1][2]
    # Convert wheel RPMs to angular velocities (rad/s)
    omega1 = rpm1 * (np.pi / 30)
    omega2 = rpm2 * (np.pi / 30)
    
    # Calculate linear and angular velocities
    v = R_over_2 * (omega1 + omega2)
    omega = R_over_L * (omega2 - omega1)
    t = 0
    # Initialize starting point for drawing the motion segment
    a, b = (int(point[1][0]), int(point[1][1]))
    x, y, theta = a, b, theta_draw * np.pi / 180
    cost = 0
    # Simulate the motion to draw the segment on the map
    while True:
        if t >= 1.0:
            break
        cos_theta_dt = np.cos(theta) * dt
        sin_theta_dt = np.sin(theta) * dt

        a = x
        b = y
        x = x + v * cos_theta_dt
        y = y + v * sin_theta_dt
        theta = theta + (omega * dt)

        t += dt
        # Draw the line segment for the current motion step
        cv2.line(obstacle_map, (int(a), int(b)), (int(x), int(y)), (255, 0, 0), 2)
        if path_interval % 10 == 0:
            cv2.imshow("Shortest Path", obstacle_map)
            out.write(obstacle_map)
            cv2.waitKey(1)
        path_interval += 1

    # Update theta_draw based on current heading
    theta_draw = np.degrees(theta) % 360

velocity_with_position = []
theta_draw = 0
f = 0
# Compute velocity and position data along the path
for point in shortest_path:
    R = 33   # Wheel radius (original scale)
    L = 287  # Wheelbase (original scale)
    dt = 0.01  # Time step
    R_over_2 = R / 2
    R_over_L = R / L
    rpm1, rpm2 = point[1][2][0], point[1][2][1]
    # Convert RPMs to angular velocities (rad/s)
    omega1 = rpm1 * (np.pi / 30)
    omega2 = rpm2 * (np.pi / 30)

    v = R_over_2 * (omega1 + omega2)
    omega = R_over_L * (omega2 - omega1)
    t = 0
    a, b = (0), (0)
    if f == 0:
        x, y, theta = a, b, theta_draw * np.pi / 180
        f = 1
    cost = 0
    # Simulate motion to update position and orientation
    while True:
        if t >= 1.0:
            break

        cos_theta_dt = np.cos(theta) * dt
        sin_theta_dt = np.sin(theta) * dt

        x = x + v * cos_theta_dt
        y = y + v * sin_theta_dt
        theta = theta + (omega * dt)

        t += dt

    # Save the velocity and position data
    velocity_with_position.append(((round(x / 1000, 2), round((y) / 1000, 2), round(theta, 2)), (round(v / 1000, 2), round(-omega, 2))))
    theta_draw = np.degrees(theta) % 360
print()
print('velocity_with_position:' , velocity_with_position)
# Display the obstacle map with the shortest path
cv2.imshow("Shortest Path", obstacle_map)
out.write(obstacle_map)
out.release()

cv2.waitKey(0)
cv2.destroyAllWindows()