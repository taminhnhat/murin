#!/bin/bash

date
export ROS_DOMAIN_ID=3
source /home/nhattm/dev-ws/install/setup.bash
printenv | grep ROS
ros2 launch ldlidar_stl_ros2 ld19.launch.py