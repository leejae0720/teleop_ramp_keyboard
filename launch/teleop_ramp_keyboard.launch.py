import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():

    teleop_ramp_keyboard_node = GroupAction(
        actions=[
            Node(
                package='teleop_ramp_keyboard',
                executable='teleop_ramp_keyboard',
                output='screen',
                emulate_tty=True,

            )
        ]
    )

    return LaunchDescription([
        teleop_ramp_keyboard_node,

    ])
