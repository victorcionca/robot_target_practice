from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    camera_ip = LaunchConfiguration('camera_ip')
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.0.106'
    )

    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='False'
    )

    replay = LaunchConfiguration('replay')
    replay_arg = DeclareLaunchArgument(
        'replay',
        default_value='False'
    )

    
    return LaunchDescription([
        camera_ip_arg, record_arg, replay_arg,
        # RealSense camera
        Node(
            package='esp32cam_driver',
            namespace='camera',
            executable='cam_driver',
            name='camera',
            parameters=[
                {'cam_ip': camera_ip,
                 'record': record,
                 'replay': replay}
            ]
            ),
    ])