#!/bin/bash

date
export ROS_DOMAIN_ID=3
source /home/nhattm/dev-ws/install/setup.bash
printenv | grep ROS
ros2 run murin_websocket publisher #> ~/.murin/murin_bringup.log