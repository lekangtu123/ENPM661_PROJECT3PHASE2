# ENPM661 PROJECT3 PHASE2
### 
    Author 1: Lekang Tu UID: 121227287 Directory ID: ltu
    Author 2: Xinze Li UID: 120278136 Directory ID: starli98
    Author 3: Opeyemi Ajayi UID: 117001969 Directory ID: dajayi1




source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch turtlebot3_project3 competition_world.launch.py
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_project3 proj3p2_tu_Li_ajayi_part2.py

ros2 launch astar_falcon_planner ros_falcon_astar.launch.py     start_position:="[0.5,1.5,0.0]"     end_position:="[5.0,1.5,0.0]"     robot_radius:=0.105     clearance:=0.10     delta_time:=0.1     wheel_radius:=0.033     wheel_distance:=0.16     rpms:="[60.0,30.0]"
