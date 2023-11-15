#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_scout_gazebo = get_package_share_directory('scout_gazebo')
    pkg_scout_description = get_package_share_directory('scout_description')
    default_rviz_config_path = os.path.join(pkg_scout_gazebo, 'rviz/urdf_config.rviz')


    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scout_description, 'launch', 'scout_base_description.launch.py'),
        )
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scout',
                                   '-y', '0.0',
                                   '-z', '0.5',
                                   '-x', '0.5'],
                        output='screen')
    
    joint_state_broadcaster_node = Node(
                name="joint_state_broadcaster_spawner",
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            )
    
    drive_control_node = Node(
                name="diff_drive_controller_spawner",
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
            )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        spawn_robot_world,
        spawn_entity,
        joint_state_broadcaster_node,
        drive_control_node,
        rviz_node
    ])
