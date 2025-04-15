# ENPM661 PROJECT3 PHASE2
### 
    Author 1: Lekang Tu UID: 121227287 Directory ID: ltu
    Author 2: Xinze Li UID: 120278136 Directory ID: starli98
    Author 3: Opeyemi Ajayi UID: 117001969 Directory ID: dajayi1


The all libraries/dependencies used in this project

- numpy              : numerical computations
- OpenCV (cv2)       : image processing
- math               : mathematical functions
- queue.PriorityQueue: for implementing pathfinding (e.g., A*)
- rclpy              : ROS 2 Python client library
- geometry_msgs/Twist: for publishing velocity commands
- nav_msgs/Odometry  : for subscribing to robot odometry
- time               : delay handling and timestamping
- queue.PriorityQueue: priority-based task or node scheduling (used in A* search)

For Part01:
type in your terminal: 
1. cd ~/Desktop/proj3p2_tu_Li_ajayi/project3_ws/src/turtlebot3_project3/scripts
2. python3 proj3p2_tu_Li_ajayi_part1.py

For Part02 gazebo:
type in your terminal:
1. cd ~/Desktop/proj3p2_tu_Li_ajayi/project3_ws
2. colcon build
3. source /opt/ros/humble/setup.bash
4. source install/setup.bash
5. open other terminal, do same operation
6. In first one: export TURTLEBOT3_MODEL=waffle
7. In first one: ros2 launch turtlebot3_project3 competition_world.launch.py
8. In second one: ros2 run turtlebot3_project3 proj3p2_tu_Li_ajayi_part2.py

For Part02 falconsim:
type in your terminal:
1. cd ~/Desktop/proj3p2_tu_Li_ajayi_falcon/ROS2/falcon_turtlebot3_project_ws
2. colcon build
3. source /opt/ros/humble/setup.bash
4. source install/setup.bash
5. ros2 launch astar_falcon_planner ros_falcon_astar.launch.py     start_position:="[0.5,1.5,0.0]"     end_position:="[5.0,1.5,0.0]"     robot_radius:=0.105     clearance:=0.10     delta_time:=0.1     wheel_radius:=0.033     wheel_distance:=0.16     rpms:="[60.0,30.0]"

