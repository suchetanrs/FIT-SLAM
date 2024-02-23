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

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    pkg_scout_gazebo = get_package_share_directory('scout_gazebo')
    pkg_scout_description = get_package_share_directory('scout_description')
    pkg_scout_nav = get_package_share_directory('scout_navigation2')
    pkg_scout_teleop = get_package_share_directory('scout_teleop')
    
    twist_mux_param_file = os.path.join(pkg_scout_nav, 'params', 'twist_mux.yaml')
    robot_localization_file_path = os.path.join(pkg_scout_nav, 'config/ekf_with_gps.yaml')
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

    robot_x = LaunchConfiguration('robot_x')
    SCOUT_X = os.environ['SCOUT_X']
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value=TextSubstitution(text=SCOUT_X),
        description='The namespace of the robot')

    robot_y = LaunchConfiguration('robot_y')
    SCOUT_Y = os.environ['SCOUT_Y']
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value=TextSubstitution(text=SCOUT_Y),
        description='The namespace of the robot')
#---------------------------------------------

    # Start World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scout_gazebo, 'launch', 'start_world.launch.py'),
        )
    )

    # Spawn robot
    spawn_scout = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scout_gazebo, 'launch', 'spawn_scout.launch.py'),
        ),
        launch_arguments={'robot_namespace': robot_namespace, 'robot_y': robot_y, 'robot_x': robot_x}.items()
    )
#---------------------------------------------

    # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        namespace=robot_namespace,
        parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom')]
    )

    def set_gps_localization(context):
    
        # Start robot localization using an Extended Kalman filter...map->odom transform
        start_robot_localization_global_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            namespace=context.launch_configurations['robot_namespace'],
            parameters=[robot_localization_file_path, 
            {'use_sim_time': use_sim_time,
            'map_frame': context.launch_configurations['robot_namespace'] + '/odom',
            'odom_frame': context.launch_configurations['robot_namespace'] + '/odom_scout',
            'base_link_frame': context.launch_configurations['robot_namespace'] + '/base_footprint',
            'world_frame': context.launch_configurations['robot_namespace'] + '/odom'}],
            remappings=[('odometry/filtered', 'odom'),
                        ('set_pose', 'initialpose')])

        # # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
        # start_robot_localization_local_cmd = Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node_odom',
        #     output='screen',
        #     namespace=context.launch_configurations['robot_namespace'],
        #     parameters=[robot_localization_file_path, 
        #     {'use_sim_time': use_sim_time,
        #     'map_frame': context.launch_configurations['robot_namespace'] + '/odom',
        #     'odom_frame': context.launch_configurations['robot_namespace'] + '/odom_scout',
        #     'base_link_frame': context.launch_configurations['robot_namespace'] + '/base_footprint',
        #     'world_frame': context.launch_configurations['robot_namespace'] + '/odom_scout'}],
        #     remappings=[('odometry/filtered', 'odometry/local'),
        #                 ('set_pose', 'initialpose')])
        start_robot_localization_local_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            namespace=context.launch_configurations['robot_namespace'],
            parameters=[robot_localization_file_path, 
            {'use_sim_time': use_sim_time,
            'map_frame': context.launch_configurations['robot_namespace'] + '/odom',
            'odom_frame': context.launch_configurations['robot_namespace'] + '/odom',
            'base_link_frame': context.launch_configurations['robot_namespace'] + '/base_footprint',
            'world_frame': context.launch_configurations['robot_namespace'] + '/odom'}],
            remappings=[('odometry/filtered', 'odom'),
                        ('set_pose', 'initialpose')])

        # set gps point
        set_gps_datum_cmd = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                context.launch_configurations['robot_namespace'] + "/datum ",
                "robot_localization/srv/SetDatum ",
                # '"{geo_pose: {position: {latitude: 43.5655, longitude: 1.4740, altitude: 150}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"',
                '"{geo_pose: {position: {latitude: 43.5655, longitude: 1.4740, altitude: 149.34}, orientation: {x: 0.079, y: -0.043, z: -0.041, w: 0.959}}}"',
            ]],
            shell=True
        )
        # start_robot_localization_global_cmd,
        return [start_robot_localization_local_cmd, set_gps_datum_cmd]

    opaque_function = OpaqueFunction(function=set_gps_localization)

#---------------------------------------------
    # Twist mux and keyboard teleop

    twist_mux_cmd = Node(
            package="twist_mux",
            executable="twist_mux",
            namespace=robot_namespace,
            parameters=[twist_mux_param_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out','diff_drive_controller/cmd_vel_unstamped')]
        )
    
    key_teleop_cmd = Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            namespace=robot_namespace,
            parameters=[{'speed': '0.4'}],
            prefix=["xterm -e"],
            remappings=[('cmd_vel', 'cmd_vel_teleop')],
    )

#---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        robot_x_arg,
        robot_y_arg,
        # start_world,
        spawn_scout,

        start_navsat_transform_cmd,
        opaque_function,
        twist_mux_cmd,
        key_teleop_cmd
    ])
