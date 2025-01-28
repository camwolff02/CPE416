#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from tf_transformations import (
    quaternion_from_euler,
    quaternion_conjugate,
    quaternion_multiply,
    euler_from_quaternion,
)

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from cpe416_interfaces.action import MoveToPose


class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__("move_to_pose_server")
        self._action_server = ActionServer(
            self, MoveToPose, "move_to_pose", self.set_goal
        )

        self.init_members()

        # get the current pose
        self.create_subscription(Odometry, "/odom", self.update_cmd_vel, 10)
        # calculate the control from error in pose
        self.linear_error_publisher_ = self.create_publisher(
            Float64, "/linear/error", 10
        )
        self.angular_error_publisher_ = self.create_publisher(
            Float64, "/angular/error", 10
        )
        self.create_subscription(Float64, "/linear/control", self.update_linear_vel, 10)
        self.create_subscription(
            Float64, "/angular/control", self.update_angular_vel, 10
        )
        # control the robot
        self.cmd_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

    def init_members(self):
        self.cmd_vel_ = Twist()
        self.curr_pose_ = Pose()
        self.threshold_ = 0.2
        self.goal_handle_ = None
        self.linear_set_ = False
        self.angular_set_ = False
        self.goal_succeeded_ = False

    def update_cmd_vel(self, odometry: Odometry):
        # only update if we have a goal set
        if self.goal_handle_ == None:
            return

        # calculate the error from the current pose
        self.curr_pose_ = odometry.pose.pose
        linear_error = math.sqrt(
            (self.target_pose.position.x - self.curr_pose_.position.x) ** 2
            + (self.target_pose.position.y - self.curr_pose_.position.y) ** 2
        )
        angular_error = euler_from_quaternion(
            quaternion_multiply(
                self.target_pose.orientation,
                quaternion_conjugate(self.curr_pose_.orientation),
            )
        )[2]

        # publish the error to the PID controllers
        self.linear_set_, self.angular_set_ = False, False
        self.linear_error_publisher_.publish(linear_error)
        self.angular_error_publisher_.publish(angular_error)

        while not self.linear_set_ and not self.angular_set_:
            ...  # wait for PID controllers to calculate next step

        if linear_error < self.threshold_ and angular_error < self.threshold_:
            # if the error is small enough, we have succeeded
            self.goal_succeeded_ = True
        else:
            # otherwise, update the caller with feedback on current state
            feedback_msg = MoveToPose.Feedback()
            feedback_msg.current = self.curr_pose_
            self.goal_handle_.publish_feedback(feedback_msg)

    # should trigger once after every publish of linear error
    def update_linear_vel(self, linear_vel: Float64):
        self.cmd_vel_.linear = linear_vel
        self.linear_set_ = True

    # should trigger once after every publish of angular error
    def update_angular_vel(self, angular_vel: Float64):
        self.cmd_vel_.angular = angular_vel
        self.angular_set_ = True

    def set_goal(self, goal_handle):
        # set the goal
        self.get_logger().info("Executing goal...")
        goal = goal_handle.request.goal
        self.target_pose = Pose()
        self.target_pose.position.x = goal.x
        self.target_pose.position.y = goal.y

        quat_raw = quaternion_from_euler(0, 0, goal.theta)
        self.target_pose.orientation = Quaternion(
            x=quat_raw[0], y=quat_raw[1], z=quat_raw[2], w=quat_raw[3]
        )

        self.goal_handle_ = goal_handle  # hand off handler to send feedback

        while not self.goal_succeeded_:
            ...  # wait for the goal to succeed

        goal_handle.succeed()

        result = MoveToPose.Result()
        result.destination = self.curr_pose_
        self.init_members()  # wipe previous goal's variables
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
