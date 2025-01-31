from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
           Node(
                package="lab3",
                executable="turn_to_angle",
                remappings=[
                    ("/odometry", "/diff_drive/odometry"),
                    ("/cmd_vel", "/diff_drive/cmd_vel"),
                ],
                parameters=[
                    {"kp": 0.8},
                    {"ki": 0.0},
                    {"kd": 0.0},
                ]
            ),
        ]
    )
