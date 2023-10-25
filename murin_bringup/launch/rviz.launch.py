import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory("murin_bringup")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/main.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='log_level', default_value='info',description=''),
        rviz_node
    ])
