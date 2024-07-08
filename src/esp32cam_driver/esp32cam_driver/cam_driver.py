import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from example_interfaces.srv import Trigger

# To conver the image to sensor_msg::Image message
from cv_bridge import CvBridge

from threading import Thread
import requests
import cv2
from io import BytesIO
import numpy as np
from time import time_ns, sleep

import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message

# cam_ip is a node parameter
cam_ip = "10.0.0.1"
stream_url = "http://{cam_ip}/stream"
stream_ctrl_url = "http://{cam_ip}/control?var=stream&val={status}"

def create_caminfo():
    """Create a caminfo for the espcam without a timestamp or sequence"""
    caminfo = CameraInfo()
    caminfo.width = 640
    caminfo.height = 480
    caminfo.distortion_model = "plumb_bob"
    caminfo.d = [-0.025734, -0.025026, 0.000300, 0.000563, 0.000000]
    caminfo.k = [268.605981, 0.000000, 320.940380, 0.000000, 267.527477, 252.490292, 0.000000, 0.000000, 1.000000]

    caminfo.p = [268.186279, 0.000000, 320.438914, 0.000000, 0.000000, 266.970123, 251.964263, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
    caminfo.binning_x = 1
    caminfo.binning_y = 1
    caminfo.header.frame_id = "camera_link"
    return caminfo

class Esp32CamDriver(Node):
    def __init__(self, args=None):
        super().__init__('esp32cam_driver')
        # Declare IP address parameter
        self.declare_parameter('cam_ip', cam_ip)
        self.cam_ip = self.get_parameter('cam_ip').get_parameter_value().string_value
        self.get_logger().info(f"Camera IP: {self.cam_ip}")
        #
        self.streaming = False
        self.img_publisher = self.create_publisher(Image, 'image', 1)
        self.caminfo_publisher = self.create_publisher(CameraInfo, 'camera_info', 1)
        # Service to start streaming
        self.streaming_cbg = MutuallyExclusiveCallbackGroup()
        self.start_streaming_srv = self.create_service(Trigger,
                                              'start_streaming',
                                              self.start_streaming)
        # Service to stop streaming
        self.stop_streaming_srv = self.create_service(Trigger,
                                                      'stop_streaming',
                                                      self.stop_streaming,
                                                      callback_group=self.streaming_cbg)
        self.streaming_thread = None
        # Parameter for recording/replaying in simulation
        self.record = False
        self.declare_parameter('record', False)
        self.record = self.get_parameter('record').get_parameter_value().bool_value
        self.replay = False
        self.declare_parameter('replay', False)
        self.replay = self.get_parameter('replay').get_parameter_value().bool_value
        if self.record and self.replay:
            self.record = False
            self.replay = False
            self.get_logger().error("Cannot record and replay at the same time. Set both to false.")
        # For ros2 bag record/replay
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='espcam_target_practice_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.bag_writer = None
        self.get_logger().info(f"Record: {self.record}. Replay: {self.replay}")
        if self.record:
            self.get_logger().info("Configuring to record to ros2 bag")
            self.bag_writer = rosbag2_py.SequentialWriter()
            self.bag_writer.open(storage_options, converter_options)

            topic_info = rosbag2_py._storage.TopicMetadata(
                name='camera_image',
                type='sensor_msgs/msg/Image',
                serialization_format='cdr')
            self.bag_writer.create_topic(topic_info)
            self.get_logger().info("Configured to record to ros2 bag")
        self.bag_reader = None
        if self.replay:
            self.bag_reader = rosbag2_py.SequentialReader()
            self.bag_reader.open(storage_options, converter_options)
            self.get_logger().info("Configured to replay from ros2 bag")

    def start_streaming(self, req, resp):
        # Access streaming start URL
        if not self.replay:
            requests.get(stream_ctrl_url.format(cam_ip=self.cam_ip, status=1))
        # TODO error check on the above
        self.streaming = True
        # Start streaming 
        streaming_method = None
        if not self.replay:
            streaming_method = self.fetch_mjpeg_stream
        else:
            streaming_method = self.replay_mjpeg_stream
        self.streaming_thread = Thread(target=streaming_method)
        self.streaming_thread.start()
        resp.success = True
        return resp

    def stop_streaming(self, req, resp):
        self.streaming = False
        # Access streaming stop URL
        if not self.replay:
            requests.get(stream_ctrl_url.format(cam_ip=self.cam_ip, status=0))
        # TODO error check on the above
        # Wait on the streaming thread to complete
        self.streaming_thread.join()
        # Complete the service
        resp.success = True
        return resp

    def replay_mjpeg_stream(self):
        """
        Replays the image topics from a recorded ros bag
        """
        previous_ts = None
        while self.bag_reader.has_next():
            topic, img_msg, ts = self.bag_reader.read_next()
            # img_msg is bytes, must be deserialized
            img_msg = deserialize_message(img_msg, Image)
            # Adjust timestamp and introduce delays
            if previous_ts is not None:
                delay_ns = ts - previous_ts
                sleep(delay_ns/1e9)
            previous_ts = ts
            frame_timestamp_ns = time_ns()
            img_msg.header.stamp.sec = int(frame_timestamp_ns/1e9)
            img_msg.header.stamp.nanosec = int(frame_timestamp_ns%1e9)
            # Publish the image
            self.img_publisher.publish(img_msg)    # Publish the camera info
            # Publish the camera info
            caminfo = create_caminfo()
            caminfo.header.stamp.sec = img_msg.header.stamp.sec
            caminfo.header.stamp.nanosec = img_msg.header.stamp.nanosec
            self.caminfo_publisher.publish(caminfo)
            self.get_logger().info(f"Recovered and published {topic} at {frame_timestamp_ns}, {ts}")
        self.get_logger().info("Done replaying from bag")

    def fetch_mjpeg_stream(self):
        response = requests.get(stream_url.format(cam_ip=self.cam_ip), stream=True)
        if response.status_code != 200:
            self.get_logger().info(f"Failed to connect to {stream_url.format(cam_ip=self.cam_ip)}, status code: {response.status_code}")
            return

        boundary = None
        content_type = response.headers.get('Content-Type')
        if content_type and 'boundary=' in content_type:
            boundary = content_type.split('boundary=')[-1].strip()

        if not boundary:
            self.get_logger().info("No boundary found in Content-Type header.")
            return

        boundary = boundary.encode()
        buffer = b""
        cv_br = CvBridge()
        for chunk in response.iter_content(chunk_size=1024):
            if not self.streaming:
                break
            buffer += chunk
            while self.streaming:
                # Find the boundary
                start = buffer.find(boundary)
                if start == -1:
                    break
                # Find the end of the boundary
                end = buffer.find(boundary, start + len(boundary))
                if end == -1:
                    break
                # Extract the image part
                image_data = buffer[start + len(boundary):end]
                # Remove the processed part from buffer
                buffer = buffer[end + len(boundary):]

                # Find the header end
                header_end = image_data.find(b'\r\n\r\n')
                if header_end == -1:
                    continue

                # Extract image data
                image_data = image_data[header_end + 4:]

                try:
                    img_dat = BytesIO(image_data)
                    cv2_img = cv2.imdecode(np.frombuffer(img_dat.read(), np.uint8), 1)
                    # Convert the image to ros2 Image message
                    img_msg = cv_br.cv2_to_imgmsg(cv2_img, encoding='rgb8')
                    # Set a timestamp for the frame
                    frame_timestamp_ns = time_ns()
                    # Publish the image
                    img_msg.header.frame_id = "camera_link"
                    img_msg.header.stamp.sec = int(frame_timestamp_ns/1e9)
                    img_msg.header.stamp.nanosec = int(frame_timestamp_ns%1e9)
                    self.img_publisher.publish(img_msg)
                    if self.record:
                        self.bag_writer.write(
                            'camera_image',
                            serialize_message(img_msg),
                            frame_timestamp_ns
                        )
                    # Publish the camera info
                    caminfo = create_caminfo()
                    caminfo.header.stamp.sec = int(frame_timestamp_ns/1e9)
                    caminfo.header.stamp.nanosec = int(frame_timestamp_ns%1e9)
                    self.caminfo_publisher.publish(caminfo)
                    #self.get_logger().info(f"[{frame_timestamp_ns}] Generated frame")
                except Exception as e:
                    self.get_logger().info(f"Failed to decode image: {e}")
        self.get_logger().info("Stopped streaming")


def main():
    rclpy.init()

    espcam_driver = Esp32CamDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(espcam_driver)
    executor.spin()
    espcam_driver.destroy_node()
    rclpy.shutdown()
