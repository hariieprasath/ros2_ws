#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryModifier(Node):
    def __init__(self):
        super().__init__('odometry_modifier')
        
        # Subscribe to '/odometry/filtered' topic
        self.odometry_filtered_subscription = self.create_subscription(
            Odometry,
            '/odometry/unfiltered',
            self.odometry_filtered_callback,
            10)

        # Create a publisher to publish the modified odometry data to '/odom'
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
    
    def odometry_filtered_callback(self, msg):
        # Create a new Odometry message based on the received message
        modified_odom = Odometry()

        # Copy the header, pose, and twist from the '/odometry/filtered' topic
        modified_odom.header = msg.header
        modified_odom.child_frame_id = msg.child_frame_id
        modified_odom.pose = msg.pose
        modified_odom.twist = msg.twist

        # Publish the modified message to the '/odom' topic
        self.publisher.publish(modified_odom)

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the node
    odometry_modifier = OdometryModifier()

    # Spin the node so the callback functions are called
    rclpy.spin(odometry_modifier)

    # Destroy the node explicitly
    odometry_modifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
