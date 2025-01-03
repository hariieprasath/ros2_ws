#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import HardWareStatus

class HardWareStatePubNode(Node): 
    def __init__(self):
        super().__init__("hw_state_publer") 
        self.hw_status_publisher=self.create_publisher(HardWareStatus,"hardware_status",10)
        self.timer=self.create_timer(1.0,self.publish_hw_status)
        self.get_logger().warn("hw_is executing")
    def publish_hw_status(self):
        msg=HardWareStatus()
        msg.tmperature=34
        msg.are_motor_ready=True
        msg.debug_message="Here Harii"
        self.hw_status_publisher.publish(msg)

    
 
def main(args=None):
    rclpy.init(args=args)
    node = HardWareStatePubNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()