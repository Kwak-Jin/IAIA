#!/bin/bash

# About: Gomoku Program .sh file
# Created: 2024.12.10
# Author: Jin Kwak, Gyeonheal An
#!/bin/bash

# ROS environment setup
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2 &
sleep 5 

roslaunch ur5e_rg2_moveit_config move_group.launch &
sleep 10
echo "Success: Connect with the Robot."

gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun ur_python gomoku_image_capture.py; exec bash"

gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun ur_python gomoku_image_processing.py; exec bash"

gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun ur_python gomoku_move.py; exec bash"

echo "Gomoku Program is launched."