import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

from launch_ros.actions import Node

import os

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = os.path.join(pkg_share, "config", 'async_slam_settings.yaml')
    
    return LaunchDescription([
        Node(
            parameters=[
            {"slam_params_file": slam_params},
            {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),
    ])