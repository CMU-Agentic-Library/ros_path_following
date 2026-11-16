"""Step 2: pathFollower. /state_estimation + path on `path_topic` -> /cmd_vel (vehicle frame).

sensorOffsetX/Y: lidar position relative to the chassis center (forward / left, m).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')

    return LaunchDescription([
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),
        DeclareLaunchArgument('speed', default_value='0.2', description='cruise speed (m/s)'),
        DeclareLaunchArgument('yawRate', default_value='0.75', description='max yaw rate (rad/s)'),
        DeclareLaunchArgument('path_topic', default_value='/follow_path'),

        Node(
            package='path_follower', executable='pathFollower', name='pathFollower', output='screen',
            parameters=[{
                'sensorOffsetX': sensorOffsetX,
                'sensorOffsetY': sensorOffsetY,
                'lookAheadDis': 0.3,
                'speed': LaunchConfiguration('speed'),
                'accel': 0.3,
                'slowDwnDis': 0.5,
                'yawRate': LaunchConfiguration('yawRate'),
                'yawRateGain': 2.0,
                'goalYawDis': 1.0,
                'goalStopDis': 0.05,
            }],
            remappings=[('/path', LaunchConfiguration('path_topic'))],
        ),

        # sensor -> vehicle, same convention as path_follower.launch
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='vehicleTransPublisher',
            arguments=['--x', PythonExpression(['-', sensorOffsetX]), '--y', PythonExpression(['-', sensorOffsetY]),
                       '--frame-id', 'sensor', '--child-frame-id', 'vehicle'],
        ),
    ])
