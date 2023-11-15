#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R


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

def compute_angular_vel( q, q_old, dt):
    m = R.from_quat(q).as_matrix()
    m_old = R.from_quat(q_old).as_matrix()

    diff_m = (m -m_old)/dt
    return diff_m * m.transpose()

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
        self.first_msg = True
        self.pose_old = Pose

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/Leo_AS/pose',
            self.handle_pose,
            qos_best)
        self.subscription  # prevent unused variable warning

    def handle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.header.stamp #self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()

        odom_msg.header = t.header
        odom_msg.child_frame_id = t.child_frame_id
        odom_msg.pose.pose.position = msg.pose.position
        odom_msg.pose.pose.orientation = t.transform.rotation
        # odom_msg.pose.covariance = 
        dt= 1/50
#        if first_msg:
#            linear_speed = Vector3()
#            angular_speed = Vector3()
#            self.pose_old = msg.pose
#        else:
#            linear_speed = (msg.pose - self.pose_old)/dt
#
#            angular_speed = 
#
#
#        odom_msg.twist.twist.linear.x = msg.
#        odom_msg.twist.twist.angular.z = msg.
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
