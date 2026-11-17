"""Step 1: arise_slam, subscribing directly to the lidar topics the lower computer publishes.

arise_slam: lidar_topic (livox_ros_driver2/CustomMsg, default /livox/lidar) + imu_topic (default /livox/imu)
            -> /state_estimation (frame map -> sensor) and TF map -> sensor.

The map origin is wherever the lidar is when SLAM starts, so start it with the robot at the
start pose of the paths you want to run.

Also opens RViz (h1_bringup/rviz/h1.rviz) unless rviz:=false, e.g. on a headless machine.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('arise_slam_mid360'), 'launch', 'arize_slam.launch.py')),
            launch_arguments={
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }.items(),
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('h1_bringup'), 'rviz', 'h1.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
