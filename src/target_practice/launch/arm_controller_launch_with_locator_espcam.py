from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_config = LaunchConfiguration('robot_config')
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='/root/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/px150.yaml'
    )

    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='px150'
    )

    camera_ip = LaunchConfiguration('camera_ip')
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.0.106'
    )

    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false'
    )

    replay = LaunchConfiguration('replay')
    replay_arg = DeclareLaunchArgument(
        'replay',
        default_value='false'
    )

    
    return LaunchDescription([
        robot_config_arg,
        robot_name_arg,
        camera_ip_arg,
        record_arg,
        replay_arg,
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
                'robot_name': robot_name,
                'motor_configs': robot_config,
                'show_ar_tag': 'true',
                'use_rviz': 'false',
                'use_sim': 'false',
                'use_world_frame': 'false'
            }.items()
        ),
        # perception apriltag
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
        ),
        # espcam32 camera
        Node(
            package='esp32cam_driver',
            namespace='esp_camera',
            executable='cam_driver',
            name='espcam',
            parameters=[
                {'cam_ip': camera_ip,
                 'record': record,
                 'replay': replay}
            ]
        ),
        # Locator node
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='espcam_locator',
            name='espcam_locator'
        ),
        # Static TF from camera_link to robot wrist link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.05', '0', '0.05', '-1.57075', '0', '-1.57075', 'px150_1/wrist_link', 'espcam/camera_link']
        ),
        # Arm controller to interface with robot arm
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='arm_controller',
            name='arm_controller',
            arguments=['--robot_name', robot_name, '--no-autofinish']
        ),
    ])