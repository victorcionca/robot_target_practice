from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
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
        # Arm controller to interface with robot target arm
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='arm_controller',
            name='arm_controller',
            arguments=['--robot_name', 'px150_2', '--no-autofinish']
        ),        # Main node
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='controller',
            name='controller'
        )
    ])