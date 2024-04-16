# Load the interbotix packages in the system path
import sys
sys.path.extend([
    '/root/interbotix_ws/install/interbotix_xs_modules/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_xs_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_moveit_interface_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_common_modules/lib/python3.8/site-packages'])

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_moveit_interface_msgs.srv import MoveItPlan 
from example_interfaces.srv import Trigger

class ArmController(InterbotixManipulatorXS):

    def __init__(self, robot_name=None, auto_finish=True):
        super().__init__(robot_model='px150',
                         robot_name=robot_name,
                         group_name='arm',
                         gripper_name='gripper')
        self.auto_finish = auto_finish
        # Create move arm service, use node in super
        self.move_arm_srv = self.core.create_service(MoveItPlan,
                                                     f'/{robot_name}/move_arm',
                                                     self.move_arm)
        self.sleep_arm_srv = self.core.create_service(Trigger,
                                                      f'/{robot_name}/sleep',
                                                      self.go_to_sleep)
        self.arm.set_ee_pose_components(x=0.15, y=0, z=0.24)
        self.timer = None

    def move_arm(self, request, response):
        target_point = request.ee_pose.position
        self.core.get_logger().info(f'Moving arm to {target_point.x, target_point.y, target_point.z}')
        _, success = self.arm.set_ee_pose_components(x=target_point.x,
                                        y=target_point.y,
                                        z=target_point.z)
        if self.auto_finish:
            self.timer = self.core.create_timer(1.0,self.finish)
        response.success = success
        return response

    def finish(self):
        self.core.get_logger().info('Back to sleep')
        #self.arm.go_to_sleep_pose()
        self.arm.set_ee_pose_components(x=0.15, y=0, z=0.24)
        self.core.destroy_timer(self.timer)
        # self.shutdown() This should be done on Interrupt, before exit

    def go_to_sleep(self, request, response):
        self.arm.set_ee_pose_components(x=0.15, y=0, z=0.24)
        self.arm.go_to_sleep_pose()
        response.success = True
        return response

def main(*args):
    robot_name = None
    auto_finish = True
    for i in range(len(sys.argv)):
        if sys.argv[i] == '--robot_name':
            robot_name = sys.argv[i+1]
        if sys.argv[i] == '--no-autofinish':
            auto_finish = False
    try:
        arm_controller = ArmController(robot_name,
                                       auto_finish=auto_finish)
    except KeyboardInterrupt:
        arm_controller.arm.go_to_sleep_pose()
        arm_controller.shutdown()