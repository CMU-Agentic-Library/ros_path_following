"""Step 1: arise_slam, fed from the lidar topics the lower computer publishes.

livoxInputAdapter: lidar_topic (PointCloud2 or Livox CustomMsg, see lidar_format) + imu_topic
                   -> /lidar/scan (CustomMsg) + /imu/data, which arise_slam subscribes to.
arise_slam:        -> /state_estimation (frame map -> sensor) and TF map -> sensor.

Check the type with `ros2 topic info /livox/lidar`: sensor_msgs/msg/PointCloud2 -> lidar_format:=pointcloud2,
livox_ros_driver2/msg/CustomMsg -> lidar_format:=custom.

The map origin is wherever the lidar is when SLAM starts, so start it with the robot at the
start pose of the paths you want to run.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('lidar_format', default_value='pointcloud2', description='pointcloud2 | custom'),

        Node(
            package='h1_bringup', executable='livoxInputAdapter', name='livoxInputAdapter', output='screen',
            parameters=[{
                'inputCloudTopic': LaunchConfiguration('lidar_topic'),
                'inputImuTopic': LaunchConfiguration('imu_topic'),
                'inputFormat': LaunchConfiguration('lidar_format'),
            }],
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('arise_slam_mid360'), 'launch', 'arize_slam.launch.py'))),
    ])
