#!/bin/bash

date
export ROS_DOMAIN_ID=3
source /home/nhattm/dev-ws/install/setup.bash
printenv | grep ROS
ros2 run web_video_server web_video_server > ~/.murin/murin_stream.log