# Load the interbotix packages in the system path
import sys
sys.path.extend([
    '/root/interbotix_ws/install/interbotix_xs_modules/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_xs_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_moveit_interface_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_common_modules/lib/python3.8/site-packages'])

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_moveit_interface_msgs.srv import MoveItPlan 

class ArmController(InterbotixManipulatorXS):

    def __init__(self):
        super().__init__(robot_model='px150',
                         group_name='arm',
                         gripper_name='gripper')
        # Create move arm service, use node in super
        self.move_arm_srv = self.core.create_service(MoveItPlan,
                                                     'move_arm',
                                                     self.move_arm)
        self.timer = None

    def move_arm(self, request, response):
        target_point = request.ee_pose.position
        self.core.get_logger().info(f'Moving arm to {target_point.x, target_point.y, target_point.z}')
        _, success = self.arm.set_ee_pose_components(x=target_point.x,
                                        y=target_point.y,
                                        z=target_point.z)
        self.timer = self.core.create_timer(1.0,self.finish)
        response.success = success
        return response

    def finish(self):
        self.core.get_logger().info('Back to sleep')
        self.arm.go_to_sleep_pose()
        self.core.destroy_timer(self.timer)
        # self.shutdown() This should be done on Interrupt, before exit

def main():
    try:
        arm_controller = ArmController()
    except KeyboardInterrupt:
        arm_controller.shutdown()