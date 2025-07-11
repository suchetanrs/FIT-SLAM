# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription, OpaqueFunction


def generate_launch_description():
    # Get the launch directory
    # bringup_dir = get_package_share_directory('nav2_bringup')
    robot_nav_dir = get_package_share_directory('robot_navigation2')

    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['planner_server',
                       'controller_server',
                       'bt_navigator',
                    #    'behavior_server',
                    #    'smoother_server'
                       ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('tf', 'tf'),
                  ('tf_static', 'tf_static')]


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value=TextSubstitution(text=""),
        description='Top-level namespace')

    container_name = LaunchConfiguration('container_name')
    container_name_full = (robot_namespace, '/', container_name)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    def launch_opaque_nodes(context):
        params_file = LaunchConfiguration('params_file')
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(robot_nav_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        base_frame = ""
        if(context.launch_configurations['robot_namespace'] == ""):
            base_frame = ""
        else:
            base_frame = context.launch_configurations['robot_namespace'] + "/"

        # Create our own temporary YAML files that include substitutions
        param_substitutions = {
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'robot_base_frame': base_frame + 'base_footprint',
            'desired_linear_vel': "0.5",
            'max_angular_accel': "11.0"
        }


        configured_params = RewrittenYaml(
                source_file=params_file,
                root_key=context.launch_configurations['robot_namespace'],
                param_rewrites=param_substitutions,
                convert_types=True)


        load_nodes = GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    namespace=context.launch_configurations['robot_namespace'],
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    namespace=context.launch_configurations['robot_namespace'],
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                # Node(
                #     package='nav2_behaviors',
                #     executable='behavior_server',
                #     name='behavior_server',
                #     output='screen',
                #     namespace=context.launch_configurations['robot_namespace'],
                #     respawn=use_respawn,
                #     respawn_delay=2.0,
                #     parameters=[configured_params],
                #     arguments=['--ros-args', '--log-level', log_level],
                #     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    namespace=context.launch_configurations['robot_namespace'],
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                # Node(
                #     package='nav2_smoother',
                #     executable='smoother_server',
                #     name='smoother_server',
                #     output='screen',
                #     namespace=context.launch_configurations['robot_namespace'],
                #     respawn=use_respawn,
                #     respawn_delay=2.0,
                #     parameters=[configured_params],
                #     arguments=['--ros-args', '--log-level', log_level],
                #     remappings=remappings),
                # Node(
                #     package='nav2_velocity_smoother',
                #     executable='velocity_smoother',
                #     name='velocity_smoother',
                #     output='screen',
                #     respawn=use_respawn,
                #     respawn_delay=2.0,
                #     parameters=[configured_params],
                #     arguments=['--ros-args', '--log-level', log_level],
                #     remappings=remappings +
                #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_nav')]),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    namespace=context.launch_configurations['robot_namespace'],
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': lifecycle_nodes}]),
            ]
        )

        composable_container = Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            emulate_tty=True,
            remappings=remappings,
            output='screen')

        load_composable_nodes = LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name_full,
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                # ComposableNode(
                #     package='nav2_smoother',
                #     plugin='nav2_smoother::SmootherServer',
                #     name='smoother_server',
                #     parameters=[configured_params],
                #     remappings=remappings),
                ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    parameters=[configured_params],
                    remappings=remappings),
                # ComposableNode(
                #     package='nav2_behaviors',
                #     plugin='behavior_server::BehaviorServer',
                #     name='behavior_server',
                #     parameters=[configured_params],
                #     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    parameters=[configured_params],
                    remappings=remappings),
                # ComposableNode(
                #     package='nav2_velocity_smoother',
                #     plugin='nav2_velocity_smoother::VelocitySmoother',
                #     name='velocity_smoother',
                #     parameters=[configured_params],
                #     remappings=remappings +
                #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_nav')]),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{'use_sim_time': use_sim_time,
                                'autostart': autostart,
                                'node_names': lifecycle_nodes}]),
            ],
        )
        
        return [declare_params_file_cmd, load_nodes, load_composable_nodes, composable_container]

    opaque_function = OpaqueFunction(function=launch_opaque_nodes)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    # ld.add_action(load_nodes)
    ld.add_action(opaque_function)
    return ld