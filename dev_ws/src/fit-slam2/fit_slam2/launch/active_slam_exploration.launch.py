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
    roadmap_explorer_share = get_package_share_directory('roadmap_explorer')

    # declare arguments to override RViz and params files (and sim time)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock if true'
    )

    declare_rviz_file = DeclareLaunchArgument(
        'rviz_file',
        default_value=os.path.join(
            fit_slam2_share, 'rviz', 'active_slam.rviz'),
        description='Path to RViz config for exploration'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            fit_slam2_share, 'params', 'active_slam_exploration_params.yaml'),
        description='Path to YAML params for exploration'
    )

    # include the exploration_server.launch.py from roadmap_explorer
    exploration_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                roadmap_explorer_share,
                'launch',
                'exploration_server.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_file':    LaunchConfiguration('rviz_file'),
            'params_file':  LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_file,
        declare_params_file,
        exploration_include,
    ])
