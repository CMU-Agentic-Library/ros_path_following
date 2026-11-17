"""Step 3: /cmd_vel -> zeno_h1_motion_control AUTO input (/zeno/h1/auto/wholebody/cmd).

Needs zeno_h1_driver and zeno_h1_motion_control running. AUTO is motion_control's lowest-priority
mode, so the joystick can take over at any time.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='h1_bringup', executable='cmdVelToMotionControl', name='cmdVelToMotionControl',
            output='screen',
        ),
    ])
