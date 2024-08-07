from setuptools import setup

package_name = 'target_practice'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/target_practice_launch.py',
                                   'launch/arm_controller_launch.py',
                                   'launch/arm_controller_launch_with_locator_realsense.py',
                                   'launch/arm_controller_launch_with_locator_espcam.py',
                                   'launch/arm_controller_launch_with_camera.py',
                                   'launch/arm_controller_launch_with_camera_espcam.py',
                                   'launch/robot_local_launch.py',
                                   'launch/robot_local_launch_espcam.py',
                                   'launch/robot_remote_launch.py',
                                   'launch/robot_remote_launch_closed.py',
                                   'launch/espcam_calibrator_launch.py',
                                   ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector = target_practice.tag_detector:main',
            'controller = target_practice.controller:main',
            'controller_closed = target_practice.controller_closed:main',
            'arm_controller = target_practice.arm_controller:main',
            'exp_manager = target_practice.exp_manager:main',
            'timer_test = target_practice.timer_test:main',
            'camera_calibrator = target_practice.camera_calibrator:main',
            'single_detection = target_practice.single_detection:main',
            'realsense_locator = target_practice.realsense_locator:main',
            'espcam_locator = target_practice.espcam_locator:main'
        ],
    },
)
