from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
           Node(
                package="lab3",
                executable="move_to_pose",
                remappings=[
                    ("/odometry", "/diff_drive/odometry"),
                    ("/cmd_vel", "/diff_drive/cmd_vel"),
                ],
                parameters=[
                    {"kp_linear": 1.5},
                    {"ki_linear": 0.1},
                    {"kd_linear": 0.1},
                    {"kp_angular": 0.2},
                    {"ki_angular": 0.1},
                    {"kd_angular": 0.1},
                ]
            ),
        ]
    )
