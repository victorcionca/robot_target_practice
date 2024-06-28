# Load the interbotix packages in the system path
import sys
sys.path.extend([
    '/root/interbotix_ws/install/interbotix_xs_modules/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_xs_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_moveit_interface_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_common_modules/lib/python3.8/site-packages'])

from threading import Thread
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from nav_msgs.srv import GetPlan
from example_interfaces.srv import Trigger
from rclpy.qos import QoSProfile, QoSHistoryPolicy

class ArmController(InterbotixManipulatorXS):

    def __init__(self, robot_name=None, auto_finish=True):
        super().__init__(robot_model='px150',
                         robot_name=robot_name,
                         group_name='arm',
                         gripper_name='gripper')
        self.auto_finish = auto_finish
        self.robot_name = robot_name
        # Create move arm service, use node in super
        # Define queue depth 1 for the server
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.move_arm_srv = self.core.create_service(GetPlan,
                                                     f'/{robot_name}/move_arm',
                                                     qos_profile=qos_profile,
                                                     callback=self.move_arm)
        self.sleep_arm_srv = self.core.create_service(Trigger,
                                                      f'/{robot_name}/sleep',
                                                      self.go_to_sleep)
        self.scan_timer = None
        self.target_scan_position = 0.785375
        self.scan_time = 4.0 # seconds
        self.start_scan_srv = self.core.create_service(Trigger,
                                                 f'/{robot_name}/start_scan',
                                                 self.start_scan)
        self.stop_scan_srv = self.core.create_service(Trigger,
                                                 f'/{robot_name}/stop_scan',
                                                 self.stop_scan)
        self.is_scanning = False
        self.arm.set_ee_pose_components(x=0.15, y=0, z=0.24)
        self.timer = None

    def start_scan(self, request, response):
        self.is_scanning = True
        # Start immediately
        self.scan_timer = self.core.create_timer(0.1, self.arm_scan)
        response.success = True
        return response

    def stop_scan(self, request, response):
        self.is_scanning = False
        self.scan_timer.destroy()
        # Uncomment the next line to bring back to start position
        #self.arm.set_ee_pose_components(x=0.15, y=0, z=0.24)
        response.success = True
        return response

    def arm_scan(self):
        if self.is_scanning == False: return
        self.core.get_logger().info("Scanning")
        self.arm.set_single_joint_position(f"waist",
                                           self.target_scan_position,
                                           moving_time=self.scan_time)
        self.target_scan_position *= -1
        self.scan_timer.destroy()
        self.scan_timer = self.core.create_timer(self.scan_time, self.arm_scan)
        
    def move_arm(self, request, response):
        target_point = request.start.point
        duration_timestamp = request.start.header.stamp
        duration = duration_timestamp.sec + duration_timestamp.nanosec/1e9
        self.core.get_logger().info(f'Moving arm to {target_point.x, target_point.y, target_point.z}')
        _, success = self.arm.set_ee_pose_components(x=target_point.x,
                                        y=target_point.y,
                                        z=target_point.z,
                                        moving_time=duration,
                                        blocking=False)
        #if self.auto_finish:
        #    self.timer = self.core.create_timer(1.0,self.finish)
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