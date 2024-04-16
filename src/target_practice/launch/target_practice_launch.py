from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # interbotix_xsarm_controller xs_launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xsarm_control'),
                    'launch',
                    'xsarm_control.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_model': 'px150',
                'robot_name': 'px150_1',
                'motor_configs': '/root/work/px150_1.yaml',
                'show_ar_tag': 'true',
                'use_rviz': 'false',
                'use_sim': 'false',
                'use_world_frame': 'false'
            }.items()
        ),
        # RealSense camera
        Node(
            package='realsense2_camera',
            namespace='camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[
                {'rgb_camera.profile': '1920x1080x15'}
            ]
            ),
        # perception apriltag
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('interbotix_perception_modules'),
                    'launch',
                    'apriltag.launch.py'
                ])
            ]),
            launch_arguments={
                'camera_color_topic': '/camera/color/image_raw',
                'camera_info_topic': '/camera/color/camera_info'
            }.items()
        ),
        # Arm controller to interface with robot arm
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='arm_controller',
            name='arm_controller',
            arguments=['--robot_name', 'px150_1']
        ),
        # Main node
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='controller',
            name='controller'
        )
    ])