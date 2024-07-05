import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from rclpy.duration import Duration

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from nav_msgs.srv import GetPlan
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from example_interfaces.srv import Trigger

import random

class TargetPracticeController(Node):

    def __init__(self):
        super().__init__("target_practice_ctrl")
        # Transform listeners
        self.to_frame = 'px150_1/base_link' # TF of robot base
        self.from_frame = 'camera_link' # TF of target object
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_abs_pose = None
        # AprilTag position estimation model
        self.ar_x_fit = [0.83791034, 0.01271889]
        self.ar_y_fit = [-1.02293028, 0.01474518]
        self.ar_z_fit = [-0.27271593, 0.0091368]
        # Timer waiting for the transform to be ready
        self.move_timer = None
        self.end_of_move_time = None
        self.move_time = 4.0
        # Callback groups
        self.detect_arm_cbg = MutuallyExclusiveCallbackGroup()
        self.detect_target_cbg = MutuallyExclusiveCallbackGroup()
        self.tf_timer_cbg = MutuallyExclusiveCallbackGroup()
        self.move_timer_cbg = MutuallyExclusiveCallbackGroup()
        self.move_arm_cbg = MutuallyExclusiveCallbackGroup()
        # Detect the arm and target
        self.robot_arm_tf = None
        self.target_tf = None
        self.tf_publish_timer = None
        #self.tf_bcast = TransformBroadcaster(self)
        self.tf_static_bcast = StaticTransformBroadcaster(self)
        # Subscriber to AprilTag detection publisher
        self.tag_detect_sub = None
        self.detections = []
        # Service to trigger target detection for on-demand activity
        self.detect_target_srv = self.create_service(Trigger,
                                              'detect_target',
                                              self.detect_target,
                                              callback_group=self.detect_target_cbg)
        # Connect to arm controller
        # Define queue depth 1 for the server
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.move_arm_srv = self.create_client(GetPlan,
                                                   f'/px150_1/move_arm',
                                                   qos_profile=qos_profile,
                                                   callback_group=self.move_arm_cbg)
        while not self.move_arm_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_arm service')
        # Connect to arm scan service
        self.scanning = False
        self.arm_scan_start_srv = self.create_client(Trigger,
                                                     f'/px150_1/start_scan',
                                                     callback_group=self.move_arm_cbg)
        self.arm_scan_stop_srv = self.create_client(Trigger,
                                                     f'/px150_1/stop_scan',
                                                     callback_group=self.move_arm_cbg)
        self.arm_sleep = self.create_client(Trigger,
                                                     f'/px150_1/sleep',
                                                     callback_group=self.move_arm_cbg)


    def better_estimate(self, x, y, z):
        if self.target_abs_pose is None: return True
        crt_x, crt_y, crt_z = self.target_abs_pose
        # If the delta is greater than 1cm in any direction
        if abs(crt_x - x) > 0.01 or abs(crt_y - y) > 0.01 or abs(crt_z - z) > 0.01:
            return True
        return False

    def detect_target(self, request, response):
        # Clear the previous pose
        self.target_abs_pose = None
        self.end_of_move_time = None
        # Subscribe to tag_detection topic
        self.tag_detect_sub = self.create_subscription(
                                        AprilTagDetectionArray,
                                        '/apriltag_ros_continuous_detector_node/tag_detections',
                                        self.tag_detection_cb,
                                        10,
                                        callback_group=ReentrantCallbackGroup()
                                        )
        self.get_logger().info('Subscribed to tag detections')
        # Delay the scanning for 1s in case the target is within sight
        self.create_timer(1.0, self.start_scanning, MutuallyExclusiveCallbackGroup())
        response.success = True
        return response
    
    def start_scanning(self):
        # If a move has already begun don't scan
        if self.end_of_move_time is not None: return
        # Start the scanning process
        self.scanning = True     # TODO set as the result
        res = self.arm_scan_start_srv.call(Trigger.Request())
        self.get_logger().info('Started scanning')
        
    def tag_detection_cb(self, msg):
        """
        Process a tag detection
        """
        # If it is the right tag, stop the scanning and print out
        tags = msg.detections
        if len(tags) == 0:
            return
        tag_pose = None
        for d in tags:
            if d.id[0] == 1: # TODO make this a node parameter
                tag_pose = d.pose.pose.pose
        if tag_pose is None:
            return
        self.detections.append(tag_pose)
        if len(self.detections) < 5:
            return
        # Set up the move time the first time
        if self.end_of_move_time is None:
            self.end_of_move_time = self.get_clock().now() +\
                                    Duration(seconds=self.move_time) 
            # Schedule end of move timer
            self.move_timer = self.create_timer(self.move_time,
                                                self.end_move,
                                                self.move_timer_cbg)
            self.get_logger().info(f"Move must finish at {self.end_of_move_time}")
        # Average detections
        x_avg = sum([d.position.x for d in self.detections])/5
        y_avg = sum([d.position.y for d in self.detections])/5
        z_avg = sum([d.position.z for d in self.detections])/5
        self.detections = []
        # Continue updating the pose until we get within 15cms of it
        dist_to_target = x_avg**2 + y_avg**2 + z_avg**2
        self.get_logger().debug(f"Detected target at {dist_to_target}")
        rem_time = self.end_of_move_time - self.get_clock().now()
        if dist_to_target < 0.0225 or rem_time < rclpy.duration.Duration(seconds=0.7):
            # Destroy the subscription
            self.get_logger().debug('Cancel subs')
            self.destroy_subscription(self.tag_detect_sub)
        if self.scanning == True:
            self.get_logger().debug('Cancel scanning')
            self.arm_scan_stop_srv.call(Trigger.Request())
            self.scanning = False
        # Compute the position of the target rel to robot base
        # Step 1: get tf between robot base and camera link
        t = None
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                    self.to_frame,
                    self.from_frame,
                    now-rclpy.duration.Duration(seconds=2.0))
        except TransformException as ex:
            self.get_logger().error(f'Lookup transform failed: {ex}')
            return
        # Step 2: add pose coordinates as offset; correct from optical to ROS2
        x = t.transform.translation.x + z_avg*self.ar_x_fit[0] + self.ar_x_fit[1]
        y = t.transform.translation.y + x_avg*self.ar_y_fit[0] + self.ar_y_fit[1]
        z = t.transform.translation.z + y_avg*self.ar_z_fit[0] + self.ar_z_fit[1]
        #self.get_logger().info(f'Transform: {(x,y,z)}')
        # if the difference between new estimate and old is <1cm, ignore
        if self.better_estimate(x, y, z):
            self.get_logger().info('Better estimate')
            self.target_abs_pose = (x,y,z)
            self.move_arm()
        self.get_logger().debug(f"Finished processing detection")

    def end_move(self):
        self.get_logger().info("End move")
        self.destroy_timer(self.move_timer)
        self.destroy_subscription(self.tag_detect_sub)
        self.arm_sleep.call(Trigger.Request())
        
    def move_arm(self):
        # Create request for move_arm and call
        mov_req = GetPlan.Request()
        (x,y,z) = self.target_abs_pose
        mov_req.start.pose.position.x = x
        mov_req.start.pose.position.y = y
        mov_req.start.pose.position.z = z
        # Calculate moving time
        mov_time = self.end_of_move_time - self.get_clock().now()
        mov_sec = mov_req.start.header.stamp.sec = int(mov_time.nanoseconds/1e9)
        mov_nanosec = mov_req.start.header.stamp.nanosec = int(mov_time.nanoseconds % 1e9)
        self.get_logger().info(f'Moving arm to {self.target_abs_pose} in {mov_sec, mov_nanosec}')
        # Call service
        mov_res = self.move_arm_srv.call(mov_req)
        self.get_logger().debug('Move succeeded')
        # Done

def main():
    rclpy.init()

    ctrl = TargetPracticeController()
    executor = MultiThreadedExecutor()
    executor.add_node(ctrl)
    executor.spin()
    ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()