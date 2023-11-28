#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    explore_pkg = get_package_share_directory('frontier_exploration')
#---------------------------------------------

    # LAUNCH ARGS
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace =  LaunchConfiguration('robot_namespace')
    SCOUT_NAMESPACE = os.environ['SCOUT_NAMESPACE']
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value=TextSubstitution(text=SCOUT_NAMESPACE),
        description='The namespace of the robot')
    
#---------------------------------------------

    def all_nodes_launch(context):
        params_file = LaunchConfiguration('params_file')
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(explore_pkg, 'params', 'exploration_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        param_substitutions = {
            'robot_base_frame': context.launch_configurations['robot_namespace'] + '/base_link',
            'velodyne_frame': context.launch_configurations['robot_namespace'] + '/velodyne'
            }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=context.launch_configurations['robot_namespace'],
            param_rewrites=param_substitutions,
            convert_types=True)
        
        explore_client = Node(
            package='frontier_exploration',
            executable='explore_client',
            output='screen',
            namespace=context.launch_configurations['robot_namespace'],
            parameters=[configured_params])
        
        explore_server = Node(
            package='frontier_exploration',
            executable='explore_server',
            output='screen',
            namespace=context.launch_configurations['robot_namespace'],
            parameters=[configured_params])
        

        return [declare_params_file_cmd, explore_server, explore_client]

    opaque_function = OpaqueFunction(function=all_nodes_launch)
#---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        opaque_function
    ])