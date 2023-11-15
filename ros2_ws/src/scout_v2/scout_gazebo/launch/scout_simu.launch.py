#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

#---------------------------------------------
    #Essential_paths
    pkg_scout_gazebo = get_package_share_directory('scout_gazebo')
    pkg_scout_description = get_package_share_directory('scout_description')
    pkg_scout_nav = get_package_share_directory('scout_navigation2')
    default_model_path = os.path.join(pkg_scout_description, 'urdf/scout_v2.urdf')
    default_rviz_config_path = os.path.join(pkg_scout_nav, 'rviz/rviz_config.rviz')
    scout_nav2_pkg = get_package_share_directory('scout_navigation2')
    twist_mux_param_file = os.path.join(scout_nav2_pkg, 'params', 'twist_mux.yaml')
    pkg_scout_teleop = get_package_share_directory('scout_teleop')
#---------------------------------------------

    #for odometry using gps and imu
    pkg_share_scout_nav2 = FindPackageShare(package='scout_navigation2').find('scout_navigation2')
    robot_localization_file_path = os.path.join(pkg_share_scout_nav2, 'config/ekf_with_gps.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

#---------------------------------------------

    # Start World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scout_gazebo, 'launch', 'start_world.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scout_description, 'launch', 'scout_base_description.launch.py'),
        )
    )     

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scout'],
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
                arguments=["diff_drive_controller"]
            )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

#---------------------------------------------


    # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('imu', 'imu'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odom')])

    # Start robot localization using an Extended Kalman filter...map->odom transform
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom'),
                    ('/set_pose', '/initialpose')])

    # # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local'),
                    ('/set_pose', '/initialpose')])

    # set gps point
    set_gps_datum_cmd = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/datum ",
                "robot_localization/srv/SetDatum ",
                # '"{geo_pose: {position: {latitude: 43.5655, longitude: 1.4740, altitude: 150}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"',
                '"{geo_pose: {position: {latitude: 43.5655, longitude: 1.4740, altitude: 149.34}, orientation: {x: 0.079, y: -0.043, z: -0.041, w: 0.959}}}"',
            ]],
            shell=True
        )
                # '"{geo_pose: {position: {latitude: 43.5655, longitude: 1.4740, altitude: 149.34}, orientation: {x: 0.0362, y: -0.151, z: -0.23, w: 0.959}}}"',

    twist_mux_cmd = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_param_file, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')]
        )
#---------------------------------------------  
    pointcloud_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in','/velodyne_points'),
                    ('scan','/scan')],
        parameters=[{
            'target_frame': 'velodyne_base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 3.483,
            'angle_min': -3.14,  # -M_PI/2
            'angle_max': 3.14,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 13.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )
#---------------------------------------------

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        start_world,
        spawn_robot_world,
        spawn_entity,
        joint_state_broadcaster_node,
        drive_control_node,
        rviz_node,

        declare_use_sim_time_cmd,
        start_navsat_transform_cmd,
        start_robot_localization_global_cmd,
        start_robot_localization_local_cmd,
        set_gps_datum_cmd,
        pointcloud_to_laserscan_cmd,
        twist_mux_cmd
    ])
