import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Point, TransformStamped, Quaternion
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from example_interfaces.srv import Trigger
from example_interfaces.msg import Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_common_modules import angle_manipulation as ang
import numpy as np

import time
from sensor_msgs.msg import CameraInfo



class RealSenseLocator(Node):
    """
    This node determines the position of a scene realsense camera
    relative to the base_link of the robot
    """

    def __init__(self, args=None):
        super().__init__('realsense_locator')
        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Static broadcaster for the transform
        self.realsense_frame = 'realsense/camera_link'
        self.robot_base_frame = 'px150_1/base_link'
        self.arm_tag_frame = 'px150_1/ar_tag_link'
        self.target_frame = 'robot_target' # TODO should be parameter
        self.tf_static_bcast = StaticTransformBroadcaster(self)
        self.tf_bcast = TransformBroadcaster(self)
        # Timer for obtaining transform between robot base and arm tag
        self.robot_to_arm_tf = None
        self.timer = self.create_timer(0.1,
                                       self.check_transform,
                                       callback_group=MutuallyExclusiveCallbackGroup())
        # Sub for april tag detection
        self.arm_tag_id = 87 # TODO make this a node parameter
        self.target_tag_id = 1  # TODO make this a node parameter
        # Check to see if the camera is available, if not stop here
        # Use the "/device_info" service on the camera
        self.cam_check_sub = self.create_subscription(CameraInfo,
                                                 '/camera/color/camera_info',
                                                 self.cam_check_cb,
                                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.loc_srv = None
        self.detect_sub = None
        self.target_pub = None
        self.locating = False
        self.detecting = False
        # To hold the subscriber to AprilTag detections
        self.tag_detect_sub = None
        # Average over 5 detections
        self.detections = []
        # Transform between camlink and robot baselink, in matrix format
        self.base_to_camlink_tf_mat = None


    def cam_check_cb(self, msg):
        # If this is called it means the camera is active and we can 
        # start the rest of the services
        self.destroy_subscription(self.cam_check_sub)
        self.loc_srv = self.create_service(Trigger,
                                           'locate_robot',
                                           self.locate_robot,
                                           callback_group=MutuallyExclusiveCallbackGroup())
        #  Sub for detection messages
        self.detect_sub = self.create_subscription(Bool,
                                                 'detect_target',
                                                 self.detect_target,
                                                 1,
                                                 callback_group=MutuallyExclusiveCallbackGroup())
        # Publisher for target pose TF
        self.target_pub = self.create_publisher(TransformStamped,
                                                '/target_practice/target_pose',
                                                1,
                                                callback_group=MutuallyExclusiveCallbackGroup())


    def find_tag(self, ref_tf, invert=False):
        """
        Find the AR tag and create a transformation matrix
        from the reference frame (ref_tf) to the tag
        Determines M_{ct} the matrix bw camera and tag based on the
        AprilTag pose estimation.
        If invert is True it returns ref_tf x M_{ct}^{-1}.
        Otherwise it returns ref_tf x M_{ct}.

        Params:
        =======

        ref_tf   -- Reference TF
        invert   -- Whether to invert the relation bw camera and tag.
        """
        # take the average pose (w.r.t. the camera frame) of the AprilTag over number of samples
        num_samples = len(self.detections)
        point = Point()
        rpy = [0, 0, 0]
        for ps in self.detections:
            point.x += ps.position.x / float(num_samples)
            point.y += ps.position.y / float(num_samples)
            point.z += ps.position.z / float(num_samples)
            quat_sample = ps.orientation
            quat_list = [quat_sample.x, quat_sample.y, quat_sample.z, quat_sample.w]
            rpy_sample = euler_from_quaternion(quat_list)
            rpy[0] += rpy_sample[0] / float(num_samples)
            rpy[1] += rpy_sample[1] / float(num_samples)
            rpy[2] += rpy_sample[2] / float(num_samples)
        T_CamTag = ang.pose_to_transformation_matrix(
            [point.x, point.y, point.z, rpy[0], rpy[1], rpy[2]]
        )

        self.get_logger().info(f"Cam to tag (x,y,z): {point.x, point.y, point.z}")
        # Now, get a snapshot of the pose of arm's base_link frame w.r.t. the AR tag link (as
        # defined in the URDF - not the one found by the algorithm)
        # We can't publish the AR tag pose found using the AprilTag algorithm to the /tf tree since
        # ROS forbids a link to have multiple parents
        T_Ref = ref_tf

        if invert:
             T_CamTag = np.linalg.inv(T_CamTag)
        # Now, lets find the transform between the ref frame and the tag
        tf_mat = np.dot(T_Ref, T_CamTag)

        return tf_mat

    def matrix_to_tf(self, tf_mat, parent_frame, child_frame):
        """Convert transformation matrix to TF"""
        rpy = ang.rotation_matrix_to_euler_angles(tf_mat[:3, :3])
        quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        trans = TransformStamped()
        trans.transform.translation.x = tf_mat[0, 3]
        trans.transform.translation.y = tf_mat[1, 3]
        trans.transform.translation.z = tf_mat[2, 3]
        trans.transform.rotation =\
            Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        trans.header.frame_id = parent_frame
        trans.child_frame_id = child_frame
        trans.header.stamp = self.get_clock().now().to_msg()
        return trans

    def tf_to_matrix(self, tf):
        """Converts the TF transform into matrix format"""
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        quat = tf.transform.rotation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(quat_list)
        tf_mat =\
            ang.pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        return tf_mat

    def check_transform(self):
        t = None
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    self.arm_tag_frame,
                    now-rclpy.duration.Duration(seconds=2.0))
        except TransformException as ex:
            self.get_logger().error(f'Lookup transform failed: {ex}')
            return
        if t is not None:
            self.destroy_timer(self.timer)
        self.robot_to_arm_tf = self.tf_to_matrix(t)
        
    def locate_robot(self, request, response):
        if self.locating or self.detecting:
            # If we've started this don't start again
            self.get_logger().error("Tried to start locating but already doing it or detecting")
            response.success = False
            return response
        if self.robot_to_arm_tf is None:
            # Also bail if we don't have the TF bw robot base and arm tag
            self.get_logger().error("Tried to start locating but don't know where robot base is")
            response.success = False
            return response
        self.locating = True
        self.detections = []
        self.tag_detect_sub =  self.create_subscription(
                                        AprilTagDetectionArray,
                                        '/apriltag_realsense/apriltag_ros_continuous_detector_node/tag_detections',
                                        self.tag_detection_cb,
                                        10,
                                        callback_group=ReentrantCallbackGroup()
                                        )
        response.success = True
        return response

    def detect_target(self, msg):
        """
        msg.data is True then start detection.
        msg.data is False then stop detection.
        """
        if msg.data == False and self.detecting:
            if self.tag_detect_sub is not None:
                self.destroy_subscription(self.tag_detect_sub)
                self.detecting = False
                self.get_logger().info("Target detection stopped")
                return
        if msg.data != True: return
        if self.locating or self.detecting:
            self.get_logger().error("Cannot start detection while locating arm or if already detecting")
            return
        # We need to have the TF from base to cam_link
        if self.base_to_camlink_tf_mat is None:
            self.get_logger().error("Started detection but don't know where the robot base is")
            return
        # Start the AprilTag service again (unless already started)
        self.detecting = True
        self.detections = []
        self.tag_detect_sub = self.create_subscription(
                                        AprilTagDetectionArray,
                                        '/apriltag_realsense/apriltag_ros_continuous_detector_node/tag_detections',
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
            if self.locating and d.id[0] == self.arm_tag_id: 
                tag_pose = d.pose.pose.pose
                break
            if self.detecting and d.id[0] == self.target_tag_id:
                tag_pose = d.pose.pose.pose
                break
        if tag_pose is None:
            return
        self.get_logger().info('Detected tag')
        self.detections.append(tag_pose)
        if len(self.detections) == 5:
            if self.locating:
                # Stop subscription only while locating the arm
                self.destroy_subscription(self.tag_detect_sub)
            # Generate a TF
            if self.locating:
                self.base_to_camlink_tf_mat = self.find_tag(self.robot_to_arm_tf, True)
                base_to_camlink_tf = self.matrix_to_tf(self.base_to_camlink_tf_mat,
                                                      self.robot_base_frame,
                                                      self.realsense_frame)
                # Publish with static broadcaster
                self.tf_static_bcast.sendTransform(base_to_camlink_tf)
                self.locating = False
            elif self.detecting:
                base_to_target_mat = self.find_tag(self.base_to_camlink_tf_mat, False)
                base_to_target_tf = self.matrix_to_tf(base_to_target_mat,
                                                      'realsense',
                                                      self.target_frame)
                # Publish pose of the target using TF
                self.target_pub.publish(base_to_target_tf)
            self.get_logger().info('TF published')
            self.detections = []

def main(args=None):
    rclpy.init(args=args)
    
    calibrator = RealSenseLocator()
    executor = MultiThreadedExecutor()
    executor.add_node(calibrator)
    executor.spin()
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
