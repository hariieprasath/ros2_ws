import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class RPScanToScan(Node):
    def __init__(self):
        super().__init__('rpscan_to_scan')

        # Subscriber to the `rpscan` topic
        self.rpscan_subscriber = self.create_subscription(
            LaserScan,
            'rpscan',
            self.rpscan_callback,
            10
        )

        # Publisher to the `scan` topic
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)

    def rpscan_callback(self, msg):
        # Update the timestamp to the current ROS time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the modified message to the `scan` topic
        self.scan_publisher.publish(msg)
        
        self.get_logger().info(str(self.get_clock().now().to_msg()))

def main(args=None):
    rclpy.init(args=args)

    node = RPScanToScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
