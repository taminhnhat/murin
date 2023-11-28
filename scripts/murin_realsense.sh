#!/bin/bash

date
export ROS_DOMAIN_ID=3
source /home/nhattm/dev-ws/install/setup.bash
printenv | grep ROS
ros2 launch murin_bringup realsense.launch.py #> ~/.murin/murin_realsense.log