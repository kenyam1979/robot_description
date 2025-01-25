import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = os.path.join(pkg_share, "config", 'async_slam_settings.yaml')

    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_slam_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    print(slam_params)
 
    return launch.LaunchDescription([
        Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),
    ])