#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanModifier(Node):
    def __init__(self):
        super().__init__('scan_modifier')
        
        # Create subscribers for 'scan' and 'rpscan' topics
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'rpscan',
            self.scan_callback,
            10)
        
        self.rpscan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.rpscan_callback,
            10)

        # Create a publisher to publish the modified scan data
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        # Store the latest messages
        self.latest_scan = None
        self.latest_rpscan = None
    
    def scan_callback(self, msg):
        # Store the latest scan message
        self.latest_scan = msg

        # Process and publish if both messages are available
        if self.latest_rpscan is not None:
            self.process_and_publish()
    
    def rpscan_callback(self, msg):
        # Store the latest rpscan message
        self.latest_rpscan = msg

        # Process and publish if both messages are available
        if self.latest_scan is not None:
            self.process_and_publish()
    
    def process_and_publish(self):
        # Create a new LaserScan message based on the scan data
        modified_scan = LaserScan()

        # Copy the header from the scan topic and modify if needed
        modified_scan.header.stamp.sec = self.latest_rpscan.header.stamp.sec
        modified_scan.header.stamp.nanosec = self.latest_rpscan.header.stamp.nanosec
        modified_scan.header.frame_id = self.latest_rpscan.header.frame_id

        # Copy the rest of the LaserScan data from the scan topic
        modified_scan.angle_min = self.latest_scan.angle_min
        modified_scan.angle_max = self.latest_scan.angle_max
        modified_scan.angle_increment = self.latest_scan.angle_increment
        modified_scan.time_increment = self.latest_scan.time_increment
        modified_scan.scan_time = self.latest_scan.scan_time
        modified_scan.range_min = self.latest_scan.range_min
        modified_scan.range_max = self.latest_scan.range_max

        # Replace ranges and intensities with data from rpscan topic
        modified_scan.ranges = self.latest_scan.ranges
        modified_scan.intensities = self.latest_scan.intensities

        # Publish the modified scan data
        self.publisher.publish(modified_scan)

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the nod
    scan_modifier = ScanModifier()

    # Spin the node so the callback functions are called.
    rclpy.spin(scan_modifier)

    # Destroy the node explicitly
    scan_modifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
