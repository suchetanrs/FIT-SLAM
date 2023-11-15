#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import launch
import launch_ros
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, SetLaunchConfiguration

def generate_launch_description():

    pkg_scout_gazebo = get_package_share_directory('scout_gazebo')
    pkg_scout_description = get_package_share_directory('scout_description')
    default_rviz_config_path = os.path.join(pkg_scout_gazebo, 'rviz/urdf_config.rviz')

    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value=TextSubstitution(text='scout_ns'),
        description='The namespace of the robot')

    rviz_config_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file')


    def launch_nodes(context):
        spawn_robot_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_scout_description, 'launch', 'scout_base_description.launch.py'),
            ),
            launch_arguments={'robot_namespace': context.launch_configurations['robot_namespace']}.items()
        )

        spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-topic', context.launch_configurations['robot_namespace'] + '/robot_description',
                                    '-entity', context.launch_configurations['robot_namespace'],
                                    '-y', '0.0',
                                    '-z', '0.5',
                                    '-x', '0.0'],
                            output='screen')
        
        joint_state_broadcaster_node = Node(
                    name="joint_state_broadcaster_spawner",
                    package="controller_manager",
                    executable="spawner",
                    namespace=context.launch_configurations['robot_namespace'],
                    arguments=["joint_state_broadcaster"],
                )
        
        drive_control_node = Node(
                    name="diff_drive_controller_spawner",
                    package="controller_manager",
                    executable="spawner",
                    namespace=context.launch_configurations['robot_namespace'],
                    arguments=["diff_drive_controller"],
                )
                
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            namespace=context.launch_configurations['robot_namespace'],
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        )

        return [spawn_robot_world, spawn_entity, joint_state_broadcaster_node, drive_control_node, rviz_node]
    
    opaque_function = OpaqueFunction(function=launch_nodes)

    return LaunchDescription([
        rviz_config_arg,
        robot_namespace_arg,
        opaque_function
    ])
