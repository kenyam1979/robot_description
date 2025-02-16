##################################################
#####  Launch file for Ignition simulation   #####
##################################################

import os

from ament_index_python.packages import get_package_share_directory

# import xacro

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

    robot_desc = Command(['xacro ', default_model_path])
    # doc = xacro.process_file(default_model_path)
    # robot_desc = doc.toprettyxml(indent='  ')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(
                ['xacro ', LaunchConfiguration('model')])}
        ],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        # parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([world_path])
            ]}.items(),
        # 'gz_args': PathJoinSubstitution([default_model_path])}.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name' 'robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.10'
        ],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
                # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }
        ],
        output='screen',
    )

    # Doesn't work. Need to check
    gz_set_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value='/home/kenyam/.gazebo/models/'
    )

    # spawn_entity = launch_ros.actions.Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'robot', '-topic', 'robot_description'],
    #     output='screen'
    # )

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

    return launch.LaunchDescription(
        [
            gz_set_env,
            launch.actions.DeclareLaunchArgument(
                name='model',
                default_value=default_model_path,
                description='Absolute path to robot urdf file',
            ),
            launch.actions.DeclareLaunchArgument(
                name='rvizconfig',
                default_value=default_rviz_config_path,
                description='Absolute path to rviz config file',
            ),
            launch.actions.DeclareLaunchArgument(
                name='use_sim_time',
                default_value='True',
                description='Flag to enable use_sim_time',
            ),
            # launch.actions.ExecuteProcess(
            #     cmd=[
            #         'gazebo',
            #         '--verbose',
            #         '-s',
            #         'libgazebo_ros_init.so',
            #         '-s',
            #         'libgazebo_ros_factory.so',
            #         world_path,
            #     ],
            #     output='screen',
            # ),
            gz_sim,
            robot_localization_node,
            gz_spawn_entity,
            gz_bridge,
            robot_state_publisher_node,
            # joint_state_publisher_node,
            rviz_node,
        ]
    )
