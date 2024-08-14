from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # perception apriltag
    apriltag_realsense = \
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('apriltag_ros'),
                    'launch',
                    'continuous_detection.launch.xml'
                ])
            ]),
            launch_arguments={
                'camera_name': '/realsense/color',
                'image_topic': 'image_raw'
            }.items()
        )
    apriltag_rs_ns = GroupAction(
        actions=[
            PushRosNamespace('apriltag_realsense'),
            apriltag_realsense,
        ]
    )
    apriltag_espcam = \
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('apriltag_ros'),
                    'launch',
                    'continuous_detection.launch.xml'
                ])
            ]),
            launch_arguments={
                'camera_name': '/esp_camera',
                'image_topic': 'image'
            }.items()
        )
    apriltag_esp_ns = GroupAction(
        actions=[
            PushRosNamespace('apriltag_espcam'),
            apriltag_espcam,
        ]
    )
    return LaunchDescription([
        apriltag_rs_ns,
        apriltag_esp_ns,
        # Realsense locator node
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='realsense_locator',
            name='realsense_locator'
        ),
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='espcam_locator',
            name='espcam_locator'
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
        #Node(
        #    package='target_practice',
        #    namespace='target_practice',
        #    executable='arm_controller',
        #    name='arm_controller',
        #    arguments=['--robot_name', 'px150_2', '--no-autofinish']
        #),        # Main node
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='controller_closed',
            name='controller'
        )
    ])