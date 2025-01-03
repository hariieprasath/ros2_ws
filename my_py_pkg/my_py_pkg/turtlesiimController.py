#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('turtle_spiral')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.linear_velocity = 0.1
        self.angular_velocity = 1.0

    def move_spiral(self):
        twist = Twist()
        twist.angular.z = self.angular_velocity
        while rclpy.ok():
            twist.linear.x = self.linear_velocity
            self.publisher.publish(twist)
            self.linear_velocity += 0.01 
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    turtle_spiral.move_spiral()
    rclpy.spin(turtle_spiral)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
