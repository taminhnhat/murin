# murin

# slam
```sh
# run slam
ros2 launch slam_toolbox online_async_launch.py
# create transform tree
ros2 run tf2_tools view_frames.py
# launch navigation
ros2 launch nav2_bringup navigation_launch.py
# show voxel in rviz
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
```
