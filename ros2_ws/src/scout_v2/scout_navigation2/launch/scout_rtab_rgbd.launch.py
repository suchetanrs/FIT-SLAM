# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
# Example:
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_demos turtlebot3_rgbd.launch.py
#   OR
#   $ ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint odom_topic:=/odom args:="-d" use_sim_time:=true rgb_topic:=/camera/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info approx_sync:=true qos:=2
#   $ ros2 run topic_tools relay /rtabmap/map /map
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')

    qos = LaunchConfiguration('qos')
    qos_cmd = DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics')

    localization = LaunchConfiguration('localization')     
    localization_cmd = DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.')

    SCOUT_NAMESPACE = os.environ['SCOUT_NAMESPACE']
    robot_namespace =  LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument(
            'robot_namespace', default_value=TextSubstitution(text=SCOUT_NAMESPACE),
        description='The namespace of the robot')

    parameters={
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Grid/MaxObstacleHeight':'0.4',
          'Rtabmap/DetectionRate' : '1.5',
        #   'Optimizer/PriorsIgnored': 'false',
        #   'Rtabmap/LoopGPS': 'true',
    }

    remappings=[
           ('rgb/image', 'camera/image_raw'),
           ('rgb/camera_info', 'camera/camera_info'),
           ('depth/image', 'camera/depth/image_raw')]


    def run_all_commands_opaque(context):
        # SLAM mode:
        slam_cmd = Node(
                condition=UnlessCondition(localization),
                name='rtabmap_slam_node', package='rtabmap_slam', executable='rtabmap', output='screen',
                namespace=context.launch_configurations['robot_namespace'],
                parameters=[parameters,
                {'frame_id': context.launch_configurations['robot_namespace'] + '/base_link',
                'database_path': "/home/" + context.launch_configurations['robot_namespace'] + "/rtabmap.db"}],
                remappings=remappings,
                arguments=['-d']) # This will delete the previous database (~/.ros/rtabmap.db)
                
        # Localization mode:
        localization_cmd = Node(
                condition=IfCondition(localization),
                name='rtabmap_local_node', package='rtabmap_slam', executable='rtabmap', output='screen',
                namespace=context.launch_configurations['robot_namespace'],
                parameters=[parameters,
                {'Mem/IncrementalMemory':'False',
                'Mem/InitWMWithAllNodes':'True',
                'frame_id': context.launch_configurations['robot_namespace'] + '/base_link',
                'database_path': "/home/" + context.launch_configurations['robot_namespace'] + "/rtabmap.db"}],
                remappings=remappings)

            # Node(
            #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            #     parameters=[parameters],
            #     remappings=remappings),

        return[slam_cmd]
    
    opaque_function = OpaqueFunction(function=run_all_commands_opaque)
    
    return LaunchDescription([
        use_sim_time_cmd,
        qos_cmd,
        localization_cmd,
        robot_namespace_arg,
        opaque_function
    ])
