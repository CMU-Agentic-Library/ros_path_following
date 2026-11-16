"""Step 4 (optional): send the paths in path_sender/data once, one after another.

Alternatively send a single path with scripts/h1_uturn.py --odom /state_estimation --send.
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # pathSender maps <ws>/install/... to <ws>/src/..., so pass the prefix path like path_sender.launch does
    data = os.path.join(get_package_prefix('path_sender'), 'data')

    return LaunchDescription([
        DeclareLaunchArgument('path_topic', default_value='/follow_path'),
        DeclareLaunchArgument('looping', default_value='false'),

        Node(
            package='path_sender', executable='pathSender', name='pathSender', output='screen',
            parameters=[{
                'pathFile1': os.path.join(data, 'path_1.txt'),
                'pathFile2': os.path.join(data, 'path_2.txt'),
                'pathFile3': os.path.join(data, 'path_3.txt'),
                'goalYaw1': 1.57, 'goalYaw2': 3.14, 'goalYaw3': 0.0,
                'goalDelay': 2.0, 'initDelay': 2.0,
                'looping': LaunchConfiguration('looping'),
            }],
            remappings=[('/path', LaunchConfiguration('path_topic'))],
        ),
    ])
