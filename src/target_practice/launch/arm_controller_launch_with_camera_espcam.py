from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    
    return LaunchDescription([
        robot_config_arg,
        robot_name_arg,
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
        # RealSense camera
        Node(
            package='esp32cam_driver',
            namespace='camera',
            executable='cam_driver',
            name='camera',
            parameters=[
                {'cam_ip': camera_ip}
            ]
            ),
        # Arm controller to interface with robot arm
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='arm_controller',
            name='arm_controller',
            arguments=['--robot_name', robot_name, '--no-autofinish']
        ),
        # Static TF from camera_link to robot wrist link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.05', '0', '0.05', '0', '0', '0', 'px150_1/wrist_link', 'camera_link']
        ),
    ])