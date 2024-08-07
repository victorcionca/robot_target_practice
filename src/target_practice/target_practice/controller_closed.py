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
from example_interface.msg import Bool

import random

class TargetPoseEstimate():
    
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

    def update(self, transform):
        """
        Update the target pose estimate based on the 
        received transform.
        
        ##### Params

        transform -- TF2 TransformStamped
        """
        raise NotImplementedError

    def get_crt_estimate(self):
        """Return the current coordinates of the estimator"""
        return (self.x, self.y, self.z)

    def reset(self):
        """Resets estimate to None"""
        self.x = None
        self.y = None
        self.z = None

    def is_null(self):
        if self.x is None or self.y is None or self.z is None:
            return True
        return False

class EWMATargetPoseEstimate(TargetPoseEstimate):

    def __init__(self, alpha):
        """
        EWMA estimator for target pose, with _alpha_ as parameter.
        """
        super().__init__()
        self.alpha = alpha

    def update(self, transform):
        new_x = transform.transform.translation.x
        new_y = transform.transform.translation.y
        new_z = transform.transform.translation.z
        if self.x is None or self.y is None or self.z is None:
            self.x = new_x
            self.y = new_y
            self.z = new_z
        else:
            # Update the existing estimate using EWMA
            self.x = self.alpha*new_x + (1-self.alpha)*self.x
            self.y = self.alpha*new_y + (1-self.alpha)*self.y
            self.z = self.alpha*new_z + (1-self.alpha)*self.z
        

class TargetPracticeController(Node):

    def __init__(self):
        super().__init__("target_practice_ctrl")
        # Transform listeners
        self.parent_frame = 'px150_1/base_link' # TF of robot base
        self.target_frame = 'robot_target' # TF of target object (TODO: param)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_abs_pose = None
        # AprilTag position estimation model
        self.ar_x_fit = [0.83791034, 0.01271889]
        self.ar_y_fit = [-1.02293028, 0.01474518]
        self.ar_z_fit = [-0.27271593, 0.0091368]
        # Estimate of target pose
        self.target_estimate = EWMATargetPoseEstimate(0.25)
        # Timer to periodically look up the target transform
        self.target_lookup_timer = None
        self.target_lookup_period = 0.5 # How often to lookup the target (TODO: param)
        # The move must complete before deadline
        self.move_time = 2.0 # TODO: param
        self.end_of_move_time = None
        # Service to trigger target detection for on-demand activity
        self.detect_target_srv = self.create_service(Trigger,
                                              'detect_target',
                                              self.detect_target,
                                              callback_group=MutuallyExclusiveCallbackGroup())
        self.detecting = False
        # Publisher to the detect_target topic
        self.detection_pub = self.create_publisher(Bool,
                                                   'detect_target', 1)
        # Connect to arm controller
        # Define queue depth 1 for the server
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.move_arm_srv = self.create_client(GetPlan,
                                                   f'/px150_1/move_arm',
                                                   qos_profile=qos_profile,
                                                   callback_group=ReentrantCallbackGroup())
        while not self.move_arm_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_arm service')
        # Puttin the arm to sleep when the move is finished
        self.arm_sleep = self.create_client(Trigger,
                                                     f'/px150_1/sleep',
                                                     callback_group=MutuallyExclusiveCallbackGroup())

    def detect_target(self, request, response):
        if self.detecting:
            self.get_logger().info("Already detecting")
            response.success = False
            return response
        self.detecting = True
        # Reset the target estimate
        self.target_estimate.reset()
        # Publish the detection message
        self.detection_pub.publish(Bool(True))
        # Set the deadline timer
        self.end_of_move_time = self.get_clock().now() + self.move_time
        self.create_timer(self.move_time,
                            self.end_move,
                            MutuallyExclusiveCallbackGroup())
        # Perform the first lookup asap
        self.target_lookup_timer = self.create_timer(0.01,
                                                     self.lookup_target,
                                                     MutuallyExclusiveCallbackGroup())
        response.success = True
        return response

    def lookup_target(self):
        """Look for the target transform update target pose estimate and execute move"""
        if self.target_estimate.is_null():
            # This is the first call, set the timer properly
            self.destroy_timer(self.target_lookup_timer)
            self.target_lookup_timer = self.create_timer(self.target_lookup_period,
                                                     self.lookup_target,
                                                     MutuallyExclusiveCallbackGroup())
        t = None
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    self.target_frame,
                    now-rclpy.duration.Duration(seconds=2.0))
        except TransformException as ex:
            self.get_logger().error(f'Lookup transform failed: {ex}')
            return
        if t is None:
            return
        self.target_estimate.update(t)
        self.move_arm()
        # TODO we should stop updating if too close to the target or too little time
        # If the move time is too short we get very fast arm movement potentially dangerous
        if self.end_of_move_time - self.get_clock().now() < rclpy.duration.Duration(seconds=0.5):
            self.destroy_timer(self.target_lookup_timer)
            self.target_lookup_timer = None


    def end_move(self):
        """Called when the deadline has been reached"""
        self.get_logger().info("End move")
        if self.target_lookup_timer is not None:
            self.destroy_timer(self.target_lookup_timer)
        # Stop the target detection
        self.detection_pub.publish(Bool(False))
        # Bring the arm back to sleep position
        self.arm_sleep.call(Trigger.Request())
        self.detecting = False
        self.target_estimate.reset()
        
    def move_arm(self):
        # Create request for move_arm and call
        mov_req = GetPlan.Request()
        (x,y,z) = self.target_estimate.get_crt_estimate()
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