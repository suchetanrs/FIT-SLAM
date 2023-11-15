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
    pkg_leo_gazebo = get_package_share_directory('leo_gazebo')
    pkg_leo_description = get_package_share_directory('leo_description')
    pkg_leo_nav = get_package_share_directory('leo_navigation2')
    default_rviz_config_path = os.path.join(pkg_leo_nav, 'rviz/rviz_config.rviz')
    leo_nav2_pkg = get_package_share_directory('leo_navigation2')
    twist_mux_param_file = os.path.join(leo_nav2_pkg, 'params', 'twist_mux.yaml')
#---------------------------------------------

    #for odometry using gps and imu
    pkg_share_leo_nav2 = FindPackageShare(package='leo_navigation2').find('leo_navigation2')
    robot_localization_file_path = os.path.join(pkg_share_leo_nav2, 'config/ekf_with_gps.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

#---------------------------------------------

    # Start World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leo_gazebo, 'launch', 'start_world.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leo_description, 'launch', 'leo_base_description.launch.py'),
        )
    )     

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'leo'],
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
        twist_mux_cmd
    ])
