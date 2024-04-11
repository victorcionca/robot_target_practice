import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from target_practice.tag_detector import TagDetector
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.callback_groups import ReentrantCallbackGroup
from interbotix_perception_msgs.srv import SnapPicture
from interbotix_moveit_interface_msgs.srv import MoveItPlan 

from example_interfaces.srv import Trigger

class TargetPracticeController(Node):

    def __init__(self):
        super().__init__("target_practice_ctrl")
        self.tag_det = TagDetector(1, 87, target_obj_frame='target_obj', node_inf=self)
        # Transform listener
        self.to_frame = 'px150/base_link' # TF of robot base
        self.from_frame = 'target_obj' # TF of target object
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform = None
        # Timer waiting for the transform to be ready
        self.move_timer = self.create_timer(1.0, self.move_timer_cb)
        # Callback groups
        self.detect_cbg = MutuallyExclusiveCallbackGroup()
        self.tf_timer_cbg = MutuallyExclusiveCallbackGroup()
        self.move_timer_cbg = MutuallyExclusiveCallbackGroup()
        self.move_arm_cbg = MutuallyExclusiveCallbackGroup()
        # Detect the arm and target
        self.robot_arm_tf = None
        self.target_tf = None
        self.tf_publish_timer = self.create_timer(1.0,
                                                  self.publish_transforms,
                                                  callback_group=self.tf_timer_cbg)
        self.tf_bcast = TransformBroadcaster(self)
        # Declare service for detection
        self.detect_srv = self.create_service(Trigger,
                                              'start_detection',
                                              self.detect_arm,
                                              callback_group=self.detect_cbg)
        # Temp
        self.move_arm_srv = self.create_client(MoveItPlan,
                                                   f'/target_practice/move_arm',
                                                   callback_group=self.move_arm_cbg)
        while not self.move_arm_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_arm service')

    def detect_arm(self, request, response):
        self.target_tf = None
        self.robot_arm_tf = None
        while self.robot_arm_tf is None:
            self.robot_arm_tf = self.tag_det.find_ref_to_arm_base_transform()
            # Introduce a delay
        self.get_logger().info('Robot arm tf ready')
        # Detect the target
        self.detect_target()
        response.success = True
        return response

    def detect_target(self):
        while self.target_tf is None:
            self.target_tf = self.tag_det.find_target_transform()
            # Introduce a delay
        self.get_logger().info('Target tf ready')

    def publish_transforms(self):
        self.get_logger().info("Publishing TFs")
        if self.robot_arm_tf is not None:
            self.robot_arm_tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_bcast.sendTransform(self.robot_arm_tf)
            self.get_logger().info('Publish robot TF')
        if self.target_tf is not None:
            self.target_tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_bcast.sendTransform(self.target_tf)
            self.get_logger().info('Publish target TF')

    def move_timer_cb(self):
        self.get_logger().info("Move timer")
        # Exit early if no TFs available
        if self.target_tf is None or self.robot_arm_tf is None:
            return
        t = None
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                    self.to_frame,
                    self.from_frame,
                    now-rclpy.duration.Duration(seconds=2.0))
        except TransformException as ex:
            self.get_logger().info(f'Lookup transform failed: {ex}')
            return
        # Apply offsets for gripper and base
        x = t.transform.translation.x - 0.07 # -0.07 for pointer, -0.04 w/o
        y = t.transform.translation.y
        z = t.transform.translation.z + 0.07
        self.get_logger().info(f'Transform: {(x,y,z)}')
        self.transform = (x,y,z)
        self.destroy_timer(self.move_timer)
        self.move_arm()

    def move_arm(self):
        self.get_logger().info(f'Moving arm to {self.transform}')
        # Create request for move_arm and call
        mov_req = MoveItPlan.Request()
        (x,y,z) = self.transform
        mov_req.ee_pose.position.x = x
        mov_req.ee_pose.position.y = y
        mov_req.ee_pose.position.z = z
        # Call service
        mov_res = self.move_arm_srv.call(mov_req)
        if mov_res.success:
            self.get_logger().info('Move succeeded')
        # Move to the block
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