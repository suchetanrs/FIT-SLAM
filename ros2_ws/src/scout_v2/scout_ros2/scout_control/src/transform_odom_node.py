#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy

from tf2_ros import TransformBroadcaster

from leo_msgs.msg import WheelOdom
from nav_msgs.msg import Odometry


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)

        qos_best = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                                durability = QoSDurabilityPolicy.VOLATILE,
                                history = QoSHistoryPolicy.KEEP_LAST,
                                depth = 1,
                                liveliness=QoSLivelinessPolicy.AUTOMATIC)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            WheelOdom,
            '/firmware/wheel_odom',
            self.handle_odom,
            qos_best)
        self.subscription  # prevent unused variable warning

    def handle_odom(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.stamp #self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose_x
        t.transform.translation.y = msg.pose_y
        t.transform.translation.z = 0.0

        # only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.pose_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()

        odom_msg.header = t.header
        odom_msg.child_frame_id = t.child_frame_id
        odom_msg.pose.pose.position.x = t.transform.translation.x
        odom_msg.pose.pose.position.y = t.transform.translation.y
        odom_msg.pose.pose.position.z = t.transform.translation.z
        odom_msg.pose.pose.orientation = t.transform.rotation
        # odom_msg.pose.covariance = 
        odom_msg.twist.twist.linear.x = msg.velocity_lin
        odom_msg.twist.twist.angular.z = msg.velocity_ang
        # odom_msg.twist.covariance = 
        self.odom_pub.publish(odom_msg)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    #rclpy.shutdown()

if __name__ == '__main__':
    main()
