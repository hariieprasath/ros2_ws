#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import LedStateArray
from my_interfaces.srv import SetLed

class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("ledPanelNode") 
        self.led_states=[0,0,0]
        self.led_state_pub=self.create_publisher(LedStateArray,"led_state",10)
        self.led_state_timer=self.create_timer(4,self.pub_led_state)
        self.set_led_service=self.create_service(SetLed,"set_led",self.callback_led_set)
        self.get_logger().warn("Led node panel has been started")
    
    def pub_led_state(self):
        msg=LedStateArray()
        msg.led_states=self.led_states
        self.led_state_pub.publish(msg)

    def callback_led_set(self,request,response):
        led_num=request.led_number
        state=request.state

        if led_num > len(self.led_states) or led_num<=0:
            response.success=False
            return response
        
        if state not in [0,1]:
            response.success=False
            return response
        
        self.led_states[led_num-1]=state
        self.pub_led_state()
        response.success=True
        return response
    
 
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()