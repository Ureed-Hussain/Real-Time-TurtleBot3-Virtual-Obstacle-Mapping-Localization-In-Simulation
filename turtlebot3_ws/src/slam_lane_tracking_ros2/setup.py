from setuptools import find_packages, setup

package_name = 'slam_lane_tracking_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masters',
    maintainer_email='masters@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_pose_logger = slam_lane_tracking_ros2.robot_pose_logger:main',
            'lane_offsets_logger = slam_lane_tracking_ros2.lane_offsets_logger:main',
            'virtual_obstacles_node = slam_lane_tracking_ros2.virtual_obstacles_node_ros2:main',
            'virtual_map_builder = slam_lane_tracking_ros2.virtual_map_builder:main',
            'aruco_detector = slam_lane_tracking_ros2.aruco_detector:main',

        ],
    },
)
