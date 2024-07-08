"""
This file performs a single detection with the closed loop controller
and esp32 cam.
It enables streaming from the camera and calls the "detect_target" service.
The intended use is for simulation, where the camera is actually replaying
a bag of data.
"""

import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from rclpy.duration import Duration

from example_interfaces.srv import Trigger
from time import sleep

class SingleDetection(Node):

    def __init__(self):
        super().__init__("single_detection")
        # Client to trigger target detection for on-demand activity
        self.detect_target_client = self.create_client(Trigger,
                                              '/target_practice/detect_target',
                                              callback_group=MutuallyExclusiveCallbackGroup())
        while not self.detect_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detection service')
        # Client to start camera streaming
        self.start_streaming_client = self.create_client(Trigger,
                                                         '/camera/start_streaming',
                                                         callback_group=MutuallyExclusiveCallbackGroup())
        while not self.start_streaming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for streaming service')
        # Service for running the single detection
        self.single_detect_srv = self.create_service(Trigger,
                                                     'single_detection',
                                                     self.detect_single,
                                                     callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Ready")

    def detect_single(self, req, resp):
        self.get_logger().info("Detect single")
        self.start_streaming_client.call(Trigger.Request())
        sleep(1)
        self.detect_target_client.call(Trigger.Request())
        resp.success = True
        return resp

def main():
    rclpy.init()

    ctrl = SingleDetection()
    executor = MultiThreadedExecutor()
    executor.add_node(ctrl)
    executor.spin()
    ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()