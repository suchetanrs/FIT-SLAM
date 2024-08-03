import os
import launch
import launch_ros
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, SetLaunchConfiguration

def generate_launch_description():
    model_arg = DeclareLaunchArgument('model_name', default_value=TextSubstitution(text='scout_v2.xacro'),
        description='The xacro file name')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True',
        description='Use simulation clock if true')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value=TextSubstitution(text='scout_ns'),
        description='The namespace of the robot')


    def create_robot_description(context):
        xacro_file = os.path.join(get_package_share_directory("scout_description"), "urdf", context.launch_configurations['model_name'])
        assert os.path.exists(xacro_file), "The xacro doesnt exist in "+str(xacro_file)
        robot_description_config = xacro.process_file(xacro_file,
            mappings={"robot_namespace" : context.launch_configurations['robot_namespace']})
        robot_desc = robot_description_config.toxml()
        return [SetLaunchConfiguration('robot_desc', robot_desc)]

    create_robot_description_arg = OpaqueFunction(function=create_robot_description)


    return launch.LaunchDescription([
        model_arg,
        use_sim_time_arg,
        robot_namespace_arg,
        create_robot_description_arg,

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'robot_description': launch.substitutions.LaunchConfiguration('robot_desc')
            }]
        )
    ])