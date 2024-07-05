import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.srv import GetPlan
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from example_interfaces.srv import Trigger

import time

def calibration_position():
    range_x = range(8,15)
    range_y = range(-5,6,1)
    range_z = range(0,1)
    num_moves = len(range_x)*len(range_y)*len(range_z)
    mov_idx = 0
    for x in range_x:
        for y in range_y:
            for z in range_z:
                mov_idx += 1
                yield ((x+15)/100, y/100, (25+z)/100, mov_idx, num_moves)

class CameraCalibrator(Node):

    def __init__(self, args=None):
        super().__init__('camera_calibrator')
        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Client for camera streaming service
        self.cam_streaming = self.create_client(Trigger,
                                                '/camera/start_streaming',
                                                callback_group=MutuallyExclusiveCallbackGroup())
        self.cam_stop_streaming = self.create_client(Trigger,
                                                '/camera/stop_streaming',
                                                callback_group=MutuallyExclusiveCallbackGroup())
        while not self.cam_streaming.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for cam streaming service')
        self.cam_is_streaming = False
        # Client for moving arm service
        self.arm_move = self.create_client(GetPlan,
                                             '/px150_1/move_arm',
                                             callback_group=MutuallyExclusiveCallbackGroup())
        while not self.arm_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm move service')
        # Client for putting the arm to sleep
        self.arm_sleep = self.create_client(Trigger,
                                            '/px150_1/sleep',
                                            callback_group=MutuallyExclusiveCallbackGroup())
        # Sub for april tag detection
        self.tag_detect_sub = None
        # Timer to manage calibration process
        self.calibration_timer = self.create_timer(5.0, self.timer_cb)
        self.start_detection_timer = None
        # Generator for calibration positions
        self.pos_gen = calibration_position()
        self.detections = []

    def timer_cb(self):
        self.destroy_timer(self.calibration_timer)
        if not self.cam_is_streaming:
            _ = self.cam_streaming.call(Trigger.Request())
            self.cam_is_streaming = True
        mov_req = GetPlan.Request()
        try:
            (x,y,z,mov_idx,num_moves) = next(self.pos_gen)
        except StopIteration:
            self.cam_stop_streaming.call(Trigger.Request())
            self.arm_sleep.call(Trigger.Request())
            return
        self.get_logger().info(f"[{mov_idx}/{num_moves}]: {x,y,z}")
        mov_req.start.pose.position.x = x
        mov_req.start.pose.position.y = y
        mov_req.start.pose.position.z = z
        # Calculate moving time
        mov_req.start.header.stamp.sec = 1
        mov_req.start.header.stamp.nanosec = 0
        # Call service
        mov_res = self.arm_move.call(mov_req)
        self.start_detection_timer = self.create_timer(2.0, self.start_detection)

    def start_detection(self):
        self.destroy_timer(self.start_detection_timer)
        # Start listening for tag detections
        self.detections = []
        self.get_logger().info('Enable detection')
        self.tag_detect_sub = self.create_subscription(
                                        AprilTagDetectionArray,
                                        '/apriltag_ros_continuous_detector_node/tag_detections',
                                        self.tag_detection_cb,
                                        10,
                                        callback_group=ReentrantCallbackGroup()
                                        )
 
    def tag_detection_cb(self, msg):
        tags = msg.detections
        if len(tags) == 0:
            return
        tag_pose = None
        for d in tags:
            if d.id[0] == 1: # TODO make this a node parameter
                tag_pose = d.pose.pose.pose
        if tag_pose is None:
            return
        self.get_logger().info('Adding pose')
        self.detections.append(tag_pose)
        if len(self.detections) == 5:
            # Stop subscription
            self.destroy_subscription(self.tag_detect_sub)
            # Calculate avg detected position
            x = [t.position.x for t in self.detections]
            y = [t.position.y for t in self.detections]
            z = [t.position.z for t in self.detections]
            x_avg = sum(x)/5
            y_avg = sum(y)/5
            z_avg = sum(z)/5
            # Determine transform from camera_link to target
            t = None
            try:
                now = self.get_clock().now()
                t = self.tf_buffer.lookup_transform(
                        "camera_link",
                        "target_tag",
                        now-rclpy.duration.Duration(seconds=2.0))
            except TransformException as ex:
                self.get_logger().error(f'Lookup transform failed: {ex}')
                return
            x_meas = t.transform.translation.x
            y_meas = t.transform.translation.y
            z_meas = t.transform.translation.z
            # Write to file
            with open('camera_calibration.csv', 'a') as f:
                f.write(f"{x_meas},{y_meas},{z_meas},{x_avg},{y_avg},{z_avg}\n")
            print(f"{x_meas},{y_meas},{z_meas},{x_avg},{y_avg},{z_avg}")
            # Schedule timer for next position
            self.calibration_timer = self.create_timer(1.0, self.timer_cb)

def main(args=None):
    rclpy.init(args=args)
    
    calibrator = CameraCalibrator()
    executor = MultiThreadedExecutor()
    executor.add_node(calibrator)
    executor.spin()
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
