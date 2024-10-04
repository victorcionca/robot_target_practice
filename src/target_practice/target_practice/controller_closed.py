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
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from example_interfaces.srv import Trigger
from example_interfaces.msg import Bool

import random
from threading import Lock

class TargetPoseEstimate():
    
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

    def update(self, source, position):
        """
        Update the target pose estimate based on the 
        position received from the source
        
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

    def update(self, source, pos):
        new_x, new_y, new_z = pos
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
        self.target_lookup_period = 1.0 # How often to lookup the target (TODO: param)
        # The move must complete before deadline
        self.move_time = 4.0 # TODO: param
        self.end_of_move_time = None
        self.end_of_move_timer = None
        # Service to trigger target detection for on-demand activity
        self.detect_target_srv = self.create_service(Trigger,
                                              'detect_target',
                                              self.detect_target,
                                              callback_group=MutuallyExclusiveCallbackGroup())
        self.detecting = False
        # Publisher to the detect_target topic
        self.detection_pub = self.create_publisher(Bool,
                                                   'detect_target', 1,
                                                   callback_group=MutuallyExclusiveCallbackGroup())
        # Subscribe to target pose messages
        self.target_pose_sub = None
        # Locked dictionary for storing target pose updates
        self.target_pose_lock = Lock()
        self.target_pose_dict = dict()
        # Schedule 1s period for target pose estimation and move
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
        # Reset the arm at the start of the move
        self.arm_reset_srv = self.create_client(Trigger,
                                                     f'/px150_1/reset_arm',
                                                     callback_group=MutuallyExclusiveCallbackGroup())
        # Dedicated service for ending a move
        self.end_move_srv = self.create_service(Trigger,
                                                'end_move',
                                                self.end_move_cb,
                                                callback_group=MutuallyExclusiveCallbackGroup())
        # Try to enable the realsense locator, by calling the locate_robot service
        realsense_locate_srv = self.create_client(Trigger,
                                                  '/target_practice/locate_robot')
        self.get_logger().info("Trying to connect to RealSense")
        if realsense_locate_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("RealSense acquired")
            realsense_locate_srv.call(Trigger.Request())
        else:
            self.get_logger().info("Realsense camera not available")
            self.destroy_client(realsense_locate_srv)

        self.get_logger().info("Controller started")

    def detect_target(self, request, response):
        self.get_logger().info("Detect target")
        if self.detecting:
            self.get_logger().info("Already detecting")
            response.success = False
            return response
        self.detecting = True
        # Reset the arm
        self.arm_reset_srv.call(Trigger.Request())
        # Reset the target estimate
        self.target_estimate.reset()
        # Publish the detection message
        self.detection_pub.publish(Bool(data= True))
        # Set the deadline timer
        self.get_logger().info("Move starting")
        self.end_of_move_time = self.get_clock().now()\
                                + rclpy.duration.Duration(seconds=self.move_time)
        self.end_move_timer = self.create_timer(self.move_time,
                                            self.end_move,
                                            MutuallyExclusiveCallbackGroup())
        # Subscribe to the target pose messages
        self.target_pose_sub = self.create_subscription(TransformStamped,
                                                        '/target_practice/target_pose',
                                                        self.target_pose_cb,
                                                        2,
                                                        callback_group=ReentrantCallbackGroup())
        self.target_lookup_timer = self.create_timer(self.target_lookup_period,
                                                     self.lookup_target,
                                                     callback_group=MutuallyExclusiveCallbackGroup())
        response.success = True
        return response

    def target_pose_cb(self, msg):
        """subcriber to target pose updates"""
        t = msg
        new_x = t.transform.translation.x
        new_y = t.transform.translation.y
        new_z = t.transform.translation.z
        source = t.header.frame_id
        # Store the update in the dictionary
        self.target_pose_lock.acquire()
        self.target_pose_dict[source] = (new_x, new_y, new_z)
        self.target_pose_lock.release()
        self.get_logger().info(f"Target pose update from {t.header.frame_id}: {new_x, new_y, new_z}")


    def lookup_target(self):
        """Look for the target transform update target pose estimate and execute move"""
        # We may need to restart the timer with a different period
        self.destroy_timer(self.target_lookup_timer)
        self.get_logger().info("Looking for target TF")
        tfs = []
        # Check what TFs we have available
        self.target_pose_lock.acquire()
        for src, pos in self.target_pose_dict.items():
            tfs.append((src, pos))
        self.target_pose_lock.release()
        if len(tfs) == 0:
            # Nothing yet, reschedule
            self.target_lookup_timer = self.create_timer(
                                            0.1,
                                            self.lookup_target,
                                            MutuallyExclusiveCallbackGroup())
            return
        self.get_logger().info("Found transform")
        for src, pos in tfs:
            self.target_estimate.update(src, pos)
        self.move_arm()
        # TODO we should stop updating if too close to the target or too little time
        # If the move time is too short we get very fast arm movement potentially dangerous
        if self.end_of_move_time - self.get_clock().now()\
                    < rclpy.duration.Duration(seconds=self.target_lookup_period):
            self.target_lookup_timer = None
            self.get_logger().info("This was the last lookup")
        else:
            self.get_logger().info("Rescheduling")
            self.target_lookup_timer = self.create_timer(
                                            self.target_lookup_period,
                                            self.lookup_target,
                                            MutuallyExclusiveCallbackGroup())

    def end_move_cb(self, req, resp):
        if self.detecting == False:
            resp.success = False
            return
        self.end_move()
        resp.success = True
        return resp
        
    def end_move(self):
        """Called when the deadline has been reached"""
        self.detecting = False
        self.target_pose_dict = dict()
        self.get_logger().info("End move")
        if self.target_lookup_timer is not None:
            self.destroy_timer(self.target_lookup_timer)
        # Stop the target detection
        self.detection_pub.publish(Bool(data= False))
        self.destroy_timer(self.end_move_timer)
        # Bring the arm back to sleep position
        self.arm_sleep.call(Trigger.Request())
        self.target_estimate.reset()
        
    def move_arm(self):
        (x,y,z) = self.target_estimate.get_crt_estimate()
        # Create request for move_arm and call
        mov_req = GetPlan.Request()
        mov_req.start.pose.position.x = x-0.05  # Account for pointer
        mov_req.start.pose.position.y = y
        mov_req.start.pose.position.z = z
        # Calculate moving time
        mov_time = self.end_of_move_time - self.get_clock().now()
        #mov_time = rclpy.duration.Duration(seconds=1.0)
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