# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import time
from threading import Thread, Event
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from random import choice

# Load the interbotix packages in the system path
import sys
sys.path.extend([
    '/root/interbotix_ws/install/interbotix_xs_modules/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_xs_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_moveit_interface_msgs/lib/python3.8/site-packages',
    '/root/interbotix_ws/install/interbotix_common_modules/lib/python3.8/site-packages'])

from interbotix_moveit_interface_msgs.srv import MoveItPlan 


hostName = "0.0.0.0"
serverPort = 80

class TargetPracticeResult():
    def __init__(self, state=False):
        self.iteration = 0
        self.state = state

class TouchListener(BaseHTTPRequestHandler):
    def __init__(self, ev, tp_result, *args):
        self.ev = ev # To synchronise main thread on touch event
        self.tp_result = tp_result
        BaseHTTPRequestHandler.__init__(self, *args)

    def do_GET(self):
        self.ev.set()  # Release main thread
        self.tp_result.state = True
        self.send_response(204)
        self.end_headers()

class RobotMover(Node):
    """
    Class for interfacing with the ROS2 components: the target arm move
    controller and the target practice controller.
    Implements functionality for moving the target arm and activating the
    target practice detection.
    """
    def __init__(self, args=None):
        rclpy.init(args=args)
        super().__init__('robot_mover')
        # Create client for the target robot arm
        self.target_arm = self.create_client(MoveItPlan,
                                             'move_arm') # TODO: proper service name
        # Create client for the target practice TODO
        self.detection_srv = None

    def move_target(self, x, y, z):
        """Calls service for moving the target arm. Returns status (true/false)"""
        mov_req = MoveItPlan.Request()
        mov_req.ee_pose.position.x = x
        mov_req.ee_pose.position.y = y
        mov_req.ee_pose.position.z = z
        # Call service
        mov_res = self.target_arm.call(mov_req)
        if mov_res.success: return True
        else: return False

    def start_target_practice(self):
        # TODO
        pass

def start_webserver(webserver):
    try:
        webserver.serve_forever()
    except KeyboardInterrupt:
        pass
    webserver.server_close()
    print("Server stopped.")

def run_robot_if(robot_if):
    executor = MultiThreadedExecutor()
    executor.add_node(robot_if)
    executor.spin()

def main():
    # Create event for synchronisation bw webserver and main threads
    touch_ev = Event()
    current_state = TargetPracticeResult()

    def ws_handler(*args):
        TouchListener(touch_ev, current_state, *args)
    webServer = HTTPServer((hostName, serverPort), ws_handler)
    print("Server started http://%s:%s" % (hostName, serverPort))

    # Run web server into separate thread
    Thread(target=start_webserver, args=[webServer]).start()

    # Interface to robots
    robot_if = RobotMover()
    robot_if_thread = Thread(target=run_robot_if, args=[robot_if])
    robot_if_thread.start()

    # List of coordinates
    coords = [(0.2, 0.08, 0.3),
              (0.2, -0.08, 0.3),
              (0.205, -0.03, 0.22),
              (0.205, 0.03, 0.22)]

    # Results

    # Run experiment for number of iterations
    num_iters = 10
    touch_timeout = 5 # seconds
    for iter in range(num_iters):
        print(f"Starting iteration {iter}")
        # Move the target
        x,y,z = choice(coords)
        print(f"Moving target to {x,y,z}")
        robot_if.move_target(x,y,z)
        # Start the target practice
        # Wait for the touch
        current_state.state = False
        touch_ev.clear()
        touch_ev.wait(touch_timeout)
        # Set the result
        print(f"Iteration {iter} result {current_state.state}")

    # Clean up
    webServer.server_close()
    robot_if.destroy_node()
    rclpy.shutdown()
    robot_if_thread.join()