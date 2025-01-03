#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import HardWareStatus
from functools import partial
from my_interfaces.srv import SetLed

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery") 
        self.battery_state="full"
        self.battery_timer=self.create_timer(0.1,self.check_battery_state)
        self.last_time_battery_state= self.get_current_time()

        self.get_logger().warn("battery is executing")
    
    def get_current_time(self):
       sec,nsecs= self.get_clock().now().seconds_nanoseconds()
       return sec+nsecs/1000000000.0   
    def check_battery_state(self):
        time_now=self.get_current_time()
        if self.battery_state=="full":
            if time_now-self.last_time_battery_state >4.0:
                self.battery_state="empty"
                self.get_logger().warn("battery is empty so charging")
                self.call_set_led_server(3,1)
                self.last_time_battery_state=time_now
        else:
            if time_now-self.last_time_battery_state>6.0:
                self.battery_state="full"
                self.get_logger().warn("battery is full ")
                self.call_set_led_server(3,0)
                self.last_time_battery_state=time_now
    
    def call_set_led_server(self,led_number,state):
        client=self.create_client(SetLed,"set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server")
        request =  SetLed.Request()
        request.led_number=led_number
        request.state=state

        future=client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_set_led,led_number=led_number,state=state)
        )
    def callback_call_set_led(self,future,led_number,state):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()