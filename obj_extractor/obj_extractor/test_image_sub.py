#!/usr/bin/env python3
"""
# @file test_image_sub.py
#
# @brief Script to test the object detection server in image_subsciber.py
#
# @author Adrian Sochaniwsky
#
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cas726_interfaces.srv import DetectObjects
from cas726_interfaces.msg import BoundingBox

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/front/color_image',
            self.listener_callback,
            5)
        self.subscription  # prevent unused variable warning

        self.sub_node = rclpy.create_node('client_node')
        
        self.cli = self.sub_node.create_client(DetectObjects, 'object_detector/detect')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DetectObjects.Request()

    def send_request(self, msg):
        self.req.color = msg
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.sub_node, self.future)

        return self.future.result()

    def listener_callback(self, msg):
        self.get_logger().info('Sending Request')
        response = self.send_request(msg)

        print('Got resp')
        print(response)
        return

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()