from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='traversability_gridmap',
            executable='traversability_node',
            name='traversability_node',
            output ='screen',
            parameters=[
            	{"resolution": 0.25},
                {"half_size": 10.},
                {"security_distance": 0.7},
                {"max_slope": 0.6},
                {"ground_clearance": 0.25},
                {"robot_height":0.5}, #above lidar
                {"robot_width":0.8},
                {"robot_length":1.1},
                {"draw_isodistance_each": 1.},
                # {"frame_id":"map"},
                # {"frame_id":"map"},
            ],
            remappings=[
                #('/ouster/points', '/agilex_isae/ouster/points'),
                #('/input/pose', '/turtlesim1/turtle1/pose'),
                #('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ],
        ),

        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="static_transform_publisher",
        #     output="screen",
        #     arguments=["0", "0", "0", "0", "0", "0", "velodyne_base_link", "velodyne_sensor"]
        # )
    ])