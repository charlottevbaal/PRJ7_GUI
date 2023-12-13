#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class UINode(Node):

    def __init__(self):
        super().__init__("ui_messenger_node")
        self.create_timer(1.0, self.timer_callback)
        self.newlocation:str = ""

    def location_callback(self, location:str):
        self.newlocation = location

    def timer_callback(self):
        self.get_logger().info(self.newlocation)

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()