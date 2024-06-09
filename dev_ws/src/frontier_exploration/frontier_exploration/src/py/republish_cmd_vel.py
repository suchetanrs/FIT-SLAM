import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')
        
        # Create a subscriber to the 'cmd_vel_nav' topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        # Create a publisher to the 'cmd_vel' topic
        self.publisher_ = self.create_publisher(Twist, '/scout_2/cmd_vel_nav', 10)

    def listener_callback(self, msg):
        # Republish the received message to the 'cmd_vel' topic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Republished cmd_vel_nav to cmd_vel: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()