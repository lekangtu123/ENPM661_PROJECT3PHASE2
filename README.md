# ENPM661 PROJECT3 PHASE2
### 
    Author 1: Lekang Tu UID: 121227287 Directory ID: ltu
    Author 2: Xinze Li UID: 120278136 Directory ID: starli98
    Author 3: Opeyemi Ajayi UID: 117001969 Directory ID: dajayi1

## The libraries/dependencies used in part01(proj3p2_tu_Li_ajayi_part1.py):

- numpy (import numpy as np): Used for efficient numerical operations and array manipulations, particularly for handling coordinates, grid maps, and mathematical computations.
- cv2 (import cv2): Provided by OpenCV, used for image processing tasks such as visualization, drawing maps or paths, and working with images in simulations.
- math (import math): Standard Python library for mathematical operations, used for functions like sin, cos, sqrt, and atan2 involved in geometry and kinematics.
- queue.PriorityQueue (from queue import PriorityQueue): Used to implement the open list in A* pathfinding, ensuring efficient selection of the node with the lowest cost.


## The libraries/dependencies used in part02 gazebo(proj3p2_tu_Li_ajayi_part2.py):

- numpy (import numpy as np): Used for numerical operations, array manipulations, and mathematical processing involved in robot motion and grid map computations.
- math (import math): Built-in Python math library used for trigonometric and geometric calculations (e.g., sin, cos, sqrt, atan2).
- cv2 (import cv2): From OpenCV, used to visualize paths, draw on maps, and handle image-based input or debug visualization.
- queue.PriorityQueue (from queue import PriorityQueue): Implements the priority queue used in the A* algorithm for efficient node expansion based on lowest cost.
- time (import time): Standard Python module used for time measurements, delays, and performance profiling.
- rclpy (import rclpy, from rclpy.node import Node): Python client library for ROS2, used to create and manage ROS nodes in this project.
- geometry_msgs.msg.Twist: Message type used to publish robot velocity commands (/cmd_vel), controlling linear and angular movement.
- nav_msgs.msg.Odometry: Message type used to subscribe to the robot’s odometry data (/odom), providing real-time position and orientation feedback.

## The libraries/dependencies used in part02 falconsim(astar_planner.py):
- math (import math): Standard Python math library used for performing mathematical operations such as trigonometric functions, square roots, and angle calculations, which are essential for robot kinematics and geometry.
- numpy (import numpy as np): A powerful numerical computing library used for array operations, matrix manipulation, and efficient mathematical computations required in path planning and coordinate transformations.
- queue.PriorityQueue (from queue import PriorityQueue): Provides a priority queue implementation for managing the open list in A* algorithm, ensuring nodes are explored in order of lowest cost.


## The way to run:

For Part01:
type in your terminal: 

```sh
cd ~/Desktop/proj3p2_tu_Li_ajayi/project3_ws/src/turtlebot3_project3/scripts
```
```sh
python3 proj3p2_tu_Li_ajayi_part1.py
```

For Part02 gazebo:
type in your terminal:
```sh
cd ~/Desktop/proj3p2_tu_Li_ajayi/project3_ws
```
```sh
colcon build
```
```sh
source /opt/ros/humble/setup.bash
```
```sh
source install/setup.bash
```
```sh
open other terminal, do same operation
```
```sh
In first one: export TURTLEBOT3_MODEL=waffle
```
```sh
In first one: ros2 launch turtlebot3_project3 competition_world.launch.py
```
```sh
In second one: ros2 run turtlebot3_project3 proj3p2_tu_Li_ajayi_part2.py
```

For Part02 falconsim:
type in your terminal:
```sh
cd ~/Desktop/proj3p2_tu_Li_ajayi_falcon/ROS2/falcon_turtlebot3_project_ws
```
```sh
colcon build
```
```sh
source /opt/ros/humble/setup.bash
```
```sh
source install/setup.bash
```
```sh
ros2 launch astar_falcon_planner ros_falcon_astar.launch.py     start_position:="[0.5,1.5,0.0]"     end_position:="[5.0,1.5,0.0]"     robot_radius:=0.105     clearance:=0.10     delta_time:=0.1     wheel_radius:=0.033     wheel_distance:=0.16     rpms:="[60.0,30.0]"
```

