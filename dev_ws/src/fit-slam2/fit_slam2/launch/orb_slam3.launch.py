#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # get share directories
    fit_slam2_share = get_package_share_directory('fit_slam2')
    orb_slam3_share = get_package_share_directory('orb_slam3_ros2_wrapper')
    traversability_share = get_package_share_directory('traversability_mapping_ros')
    robot_namespace = ""

    # declare arguments to override RViz and params files (and sim time)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock if true'
    )

    declare_ros_params_file = DeclareLaunchArgument(
        'ros_params_file',
        default_value=os.path.join(
            fit_slam2_share, 'params', 'traversability_ros_params.yaml'),
        description='Path to YAML params for exploration'
    )

    declare_traversability_params_file = DeclareLaunchArgument(
        'traversability_params_file',
        default_value=os.path.join(
            fit_slam2_share, 'params', 'traversabilityParams.yaml'),
        description='Path to YAML params for exploration'
    )

    # include the exploration_server.launch.py from roadmap_explorer
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                orb_slam3_share,
                'launch',
                'rgbd.launch.py'
            )
        ),
        launch_arguments={
            'traversability_params_file': LaunchConfiguration('traversability_params_file'),
            "robot_namespace": robot_namespace
        }.items()
    )

    local_traversability_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                traversability_share,
                'launch',
                'local_traversability_mapping.launch.py'
            )
        ),
        launch_arguments={
            'params_file':  LaunchConfiguration('ros_params_file'),
            'traversability_params_file': LaunchConfiguration('traversability_params_file')
        }.items()
    )

    return LaunchDescription([
        declare_ros_params_file,
        declare_traversability_params_file,
        slam_include,
        local_traversability_include,
    ])
