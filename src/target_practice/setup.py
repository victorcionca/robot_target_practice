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
        ('share/' + package_name, ['launch/target_practice_launch.py']),
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
            'arm_controller = target_practice.arm_controller:main',
            'exp_manager = target_practice.exp_manager:main'
        ],
    },
)
