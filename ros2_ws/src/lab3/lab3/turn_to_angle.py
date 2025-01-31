#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
import rclpy.action
from rclpy.node import Node
import rclpy.executors
from tf_transformations import (
    quaternion_from_euler,
    quaternion_conjugate,
    quaternion_multiply,
    euler_from_quaternion,
)

from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from cpe416_interfaces.action import TurnToAngle
from lab3.pid import PidController


class TurnToAngleServer(Node):
    def __init__(self):
        super().__init__("turn_to_angle_server")
        self._action_server = ActionServer(
            self, 
            TurnToAngle, 
            "turn_to_angle", 
            self.execute_callback
        )

        # initialize variables for PID control
        self.declare_parameter("stop_threshold", 0.005)
        self.init_members()

        # initialize PID constants
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)

        # initialize pid controller
        self.u = PidController(
            Kp = self.get_parameter("kp").get_parameter_value().double_value,
            Ki = self.get_parameter("ki").get_parameter_value().double_value,
            Kd = self.get_parameter("kd").get_parameter_value().double_value,
        )

        # get the current pose and create robot controller publisher
        self.create_subscription(Odometry, "/odometry", self.handle_odom, 10)
        self.cmd_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

    def init_members(self):
        self.cmd_vel_ = Twist()
        self.curr_pose_ = Pose()
        self.threshold_ = self.get_parameter("stop_threshold").get_parameter_value().double_value
        self.goal_handle_ = None
        self.goal_succeeded_ = False

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        self.init_members()  # reset variables for next request
        return rclpy.action.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # set the goal
        goal = goal_handle.request.goal
        self.get_logger().info(f"Executing goal: {goal}deg")

        quat_raw = quaternion_from_euler(0, 0, goal)
        self.target_pose = Pose()
        self.target_pose.orientation = Quaternion(
            x=quat_raw[0], y=quat_raw[1], z=quat_raw[2], w=quat_raw[3]
        )

        self.goal_handle_ = goal_handle  # hand off handler to send feedback
        self.goal_handle_.publish_feedback(TurnToAngle.Feedback())

        while not self.goal_succeeded_:
            ...  # wait for the goal to succeed

        self.get_logger().info("Goal success!")
        goal_handle.succeed()

        result = TurnToAngle.Result()
        result.destination = self.curr_pose_
        self.init_members()  # wipe previous goal's variables
        return result


    def handle_odom(self, odometry: Odometry):
        # only update if we have a goal set
        if self.goal_handle_ is None:
            return

        self.curr_pose_ = odometry.pose.pose
        self.get_logger().info(f"handling odometry: ({self.curr_pose_.position.x}, {self.curr_pose_.position.y})")

        # calculate the error from the current pose
        q_final, q_initial = self.target_pose.orientation, self.curr_pose_.orientation
        error = euler_from_quaternion(
            quaternion_multiply(
                [q_final.x, q_final.y, q_final.z, q_final.w],
                quaternion_conjugate([q_initial.x, q_initial.y, q_initial.z, q_initial.w]),
            )
        )[2]

        if abs(error) < self.threshold_:
            # if the error is small enough, we have succeeded
            self.cmd_publisher_.publish(Twist())  # make the robot stop
            time.sleep(1)  # wait to release flag AFTER stop is published
            self.goal_succeeded_ = True  
        elif self.goal_handle_ is not None:
            # feed error into PID controller, then move the robot
            self.cmd_vel_.angular.z = self.u.next(error)
            self.cmd_publisher_.publish(self.cmd_vel_)

            # update the caller with feedback on current state
            feedback_msg = TurnToAngle.Feedback()
            feedback_msg.current = self.curr_pose_
            self.goal_handle_.publish_feedback(feedback_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = TurnToAngleServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

