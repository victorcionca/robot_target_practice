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
                'camera_name': '/realsense/color',
                'image_topic': 'image_raw'
            }.items()
        ),
        # Realsense camera
        Node(
            package='realsense2_camera',
            namespace='realsense',
            executable='realsense2_camera_node',
            name='realsense'
        ),
        # AprilTag continuous detection
        # Locator node
        Node(
            package='target_practice',
            namespace='target_practice',
            executable='realsense_locator',
            name='realsense_locator'
        )
    ])