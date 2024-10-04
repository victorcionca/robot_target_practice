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
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, TransformStamped, Quaternion
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from example_interfaces.srv import Trigger
from example_interfaces.msg import Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_common_modules import angle_manipulation as ang
import numpy as np

from cv_bridge import CvBridge
import cv2
import time



class EspCamLocator(Node):
    """
    This node determines the position of an arm camera
    relative to the base_link of the robot
    """

    def __init__(self, args=None):
        super().__init__('espcam_locator')
        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Static broadcaster for the transform
        self.espcam_frame = 'espcam/camera_link' # TODO should be parameter
        self.robot_base_frame = 'px150_1/base_link'
        self.target_frame = 'robot_target' # TODO should be parameter
        self.tf_bcast = TransformBroadcaster(self)
        # AprilTag IDs
        self.target_tag_id = 1  # TODO make this a node parameter
        #  Sub for detection messages
        self.detect_sub = self.create_subscription(Bool,
                                                 'detect_target',
                                                 self.detect_target,
                                                 1,
                                                 callback_group=MutuallyExclusiveCallbackGroup())
        # Sub for camera images, to detect touch event
        self.espcam_sub = None
        self.cv_bridge = CvBridge()
        # Publisher for target pose TF
        self.target_pub = self.create_publisher(TransformStamped,
                                                '/target_practice/target_pose',
                                                1,
                                                callback_group=MutuallyExclusiveCallbackGroup())
        # Client to the end_move service of the controller for immediate stop in case of touch det
        self.end_move_client = self.create_client(Trigger,
                                                  '/target_practice/end_move',
                                                  callback_group=MutuallyExclusiveCallbackGroup())
        self.detecting = False
        # To hold the subscriber to AprilTag detections
        self.tag_detect_sub = None
        # Average over 5 detections
        self.detections = []
        # Subscribe to the espcamera feed for detecting touch event
        self.espcam_sub = self.create_subscription(
                                        Image,
                                        '/esp_camera/image',
                                        self.esp_camera_cb,
                                        1,
                                        callback_group=ReentrantCallbackGroup())
        # This will start and stop streaming as needed
        self.espcam_stream_srv = self.create_client(Trigger,
                                               '/esp_camera/start_streaming',
                                               callback_group=MutuallyExclusiveCallbackGroup())
        self.espcam_stopstream_srv = self.create_client(Trigger,
                                               '/esp_camera/stop_streaming',
                                               callback_group=MutuallyExclusiveCallbackGroup())

    def find_tag(self):
        """
        Find the AR tag and create a transform from the camera link
        to the tag.
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

        # We cannot publish this as is because in ROS we cannot have multiple
        # parents for a frame. We have to obtain the transform from base to camera

        # Find transform from robot base to wrist camera
        t = None
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    self.espcam_frame,
                    now-rclpy.duration.Duration(seconds=2.0))
        except TransformException as ex:
            self.get_logger().error(f'Lookup transform failed: {ex}')
            return
        if t is None:
            self.get_logger().error(f'Cannot find transform')
            return
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        quat = t.transform.rotation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(quat_list)
        T_BaseArm =\
            ang.pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])
 
        # Transform from base to target is T_BaseArm * T_CamTag
        T_BaseTarget = np.dot(T_BaseArm, T_CamTag)
        
        return T_BaseTarget

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


    def detect_target(self, msg):
        """
        msg.data is True then start detection.
        msg.data is False then stop detection.
        """
        self.get_logger().info("Start detecting")
        if msg.data == False and self.detecting:
            self.detecting = False
            # Stop streaming
            self.espcam_stopstream_srv.call(Trigger.Request())
            if self.tag_detect_sub is not None:
                self.destroy_subscription(self.tag_detect_sub)
                self.get_logger().info("Target detection stopped")
                return
        if msg.data != True: return
        if self.detecting:
            self.get_logger().error("Cannot start detection if already detecting")
            return
        # Enable streaming on the camera
        stat = self.espcam_stream_srv.call(Trigger.Request())
        if not stat:
            self.get_logger().error("Couldn't enable espcam streaming!")
            return
        self.get_logger().info("Enabled streaming from espcam")
        # Start the AprilTag service again (unless already started)
        self.detecting = True
        self.detections = []
        self.tag_detect_sub = self.create_subscription(
                                        AprilTagDetectionArray,
                                        '/apriltag_espcam/apriltag_ros_continuous_detector_node/tag_detections',
                                        self.tag_detection_cb,
                                        10,
                                        callback_group=ReentrantCallbackGroup()
                                        )
        # Subscribe to the espcamera feed for detecting touch event
        self.espcam_sub = self.create_subscription(
                                        Image,
                                        '/esp_camera/image',
                                        self.esp_camera_cb,
                                        1,
                                        callback_group=ReentrantCallbackGroup())

    def is_color_present(self,img):
        """Uses opencv to check if the colour is present in the image"""
        # Based on ChatGPT code
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Define the color range in HSV
        colour_variation = 10
        lower_bound = np.array([50,120,70]) # Lower bound for green
        upper_bound = np.array([70,255,255]) # Upper bound for green
        
        # Create a mask that captures areas within the color range
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Calculate the percentage of the image that is within the color range
        color_percentage = (np.sum(mask) / 255) / mask.size * 100
        
        return color_percentage

    def esp_camera_cb(self, msg):
        """Callback for camera images from the esp camera"""
        # Convert message data to CV2 image
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg)
        # Use CV2 to process the image looking for the target colour that indicates touch event
        perc = self.is_color_present(cv_img)
        if perc > 0.5:
            self.get_logger().info(f"Touch event detected {perc}. Emergency stop")
            self.destroy_subscription(self.espcam_sub)
            self.destroy_subscription(self.tag_detect_sub)
            self.detecting = False
            self.end_move_client.call(Trigger.Request())
            self.get_logger().info("Target detection stopped")

    def tag_detection_cb(self, msg):
        tags = msg.detections
        if len(tags) == 0:
            return
        tag_pose = None
        for d in tags:
            if self.detecting and d.id[0] == self.target_tag_id:
                tag_pose = d.pose.pose.pose
                break
        if tag_pose is None:
            return
        self.get_logger().info('Detected tag')
        self.detections.append(tag_pose)
        if len(self.detections) == 5:
            # Generate a TF
            if self.detecting:
                base_to_target_mat = self.find_tag()
                # Convert matrix to transform
                transform = self.matrix_to_tf(base_to_target_mat,
                                              'espcam',
                                              self.target_frame)
                # Publish pose of the target using TF
                self.target_pub.publish(transform)
            self.get_logger().info('TF published')
            self.detections = []

def main(args=None):
    rclpy.init(args=args)
    
    calibrator = EspCamLocator()
    executor = MultiThreadedExecutor()
    executor.add_node(calibrator)
    executor.spin()
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
