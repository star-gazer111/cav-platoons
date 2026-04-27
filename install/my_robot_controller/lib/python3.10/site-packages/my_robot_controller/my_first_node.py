#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):#constructor to initialise node 
        super().__init__("first_node")
        self.counter_=0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("hello"+ str(self.counter_))
        self.counter_+=1
def main(args=None):
    rclpy.init(args=args) #start ros2 comm
    node = MyNode()
    rclpy.spin(node)#node will be kept alive till we kill it usingctrl c
    rclpy.shutdown() #shut down ros2 comm

if __name__== '__main__':
    main()
