from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lab3",
                namespace="linear",
                executable="pid_controller",
            ),
            Node(
                package="lab3",
                namespace="angular",
                executable="pid_controller",
            ),
            Node(
                package="lab3",
                executable="move_to_pose.py",
                remappings=[
                    ("/odometry", "/diff_drive/odometry"),
                    ("/cmd_vel", "/dif_drive/cmd_vel"),
                ],
            ),
        ]
    )
