##################################################
#####  Launch file for Ignition simulation   #####
##################################################

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_description')
    default_model_path = os.path.join(
        pkg_share, 'src/description/robot_description_ign.urdf')
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/nav2_default_view.rviz')
    world_path = os.path.join(pkg_share, 'world/map_ign.sdf')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set config
    model_config = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file',
    )
    rviz_config = launch.actions.DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file',
    )
    use_sim_time_config = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    )

    gz_set_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value='/home/kenyam/.gazebo/models/'
    )

    # Include launch files and set nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(
                ['xacro ', LaunchConfiguration('model')])}
        ],
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', PathJoinSubstitution([world_path])],
            'on_exit_shutdown': 'true'}.items(),
    )

    gz_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', Command(['xacro ', default_model_path]),
            '-name' 'robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.10'
        ],
        output='screen',
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }
        ],
        output='screen',
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    # Initiate config, launch files and nodes
    return launch.LaunchDescription(
        [
            model_config,
            rviz_config,
            use_sim_time_config,
            gz_set_env,
            gz_sim_launch,
            gz_spawn_node,
            gz_bridge_node,
            robot_state_publisher_node,
            robot_localization_node,
            rviz_node,
        ]
    )
