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
    nav2_share = get_package_share_directory('robot_navigation2')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            fit_slam2_share, 'params', 'rapid_nav2_params.yaml'),
        description='Path to YAML params for exploration'
    )

    # include the exploration_server.launch.py from roadmap_explorer
    exploration_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_share,
                'launch',
                'navigation_fast.launch.py'
            )
        ),
        launch_arguments={
            'params_file':  LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        declare_params_file,
        exploration_include,
    ])
