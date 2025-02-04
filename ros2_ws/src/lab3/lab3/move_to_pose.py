#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.action import ActionServer
import rclpy.action
from rclpy.node import Node
import rclpy.executors
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from cpe416_interfaces.action import MoveToPose
from lab3.pid import PidController


class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__("move_to_pose_server")
        self._action_server = ActionServer(
            self, 
            MoveToPose, 
            "move_to_pose", 
            self.execute_callback
        )

        # initialize variables for PID control
        self.declare_parameter("stop_threshold", 0.05)
        self.init_members()

        # initialize PID constants
        self.declare_parameter("kp_linear", 1.0)
        self.declare_parameter("ki_linear", 0.0)
        self.declare_parameter("kd_linear", 0.0)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("ki_angular", 0.0)
        self.declare_parameter("kd_angular", 0.0)

        # initialize pid controller
        self.u_linear = PidController(
            Kp = self.get_parameter("kp_linear").get_parameter_value().double_value,
            Ki = self.get_parameter("ki_linear").get_parameter_value().double_value,
            Kd = self.get_parameter("kd_linear").get_parameter_value().double_value,
        )

        self.u_angular = PidController(
            Kp = self.get_parameter("kp_angular").get_parameter_value().double_value,
            Ki = self.get_parameter("ki_angular").get_parameter_value().double_value,
            Kd = self.get_parameter("kd_angular").get_parameter_value().double_value,
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
        self.get_logger().info(f"Executing goal: ({goal.x}m, {goal.y}m)")

        self.target_pose = Pose()
        self.target_pose.position.x = goal.x
        self.target_pose.position.y = goal.y

        self.goal_handle_ = goal_handle  # hand off handler to send feedback
        self.goal_handle_.publish_feedback(MoveToPose.Feedback())

        self.get_logger().info("waiting for result...")
        while not self.goal_succeeded_:
            ...  # wait for the goal to succeed

        self.get_logger().info("Goal success!")
        goal_handle.succeed()

        result = MoveToPose.Result()
        result.destination = self.curr_pose_
        self.init_members()  # wipe previous goal's variables
        return result


    def handle_odom(self, odometry: Odometry):
        # only update if we have a goal set
        if self.goal_handle_ is None:
            return

        self.curr_pose_ = odometry.pose.pose


        # calculate the error from the current pose
        x_error = self.target_pose.position.x - self.curr_pose_.position.x
        y_error = self.target_pose.position.y - self.curr_pose_.position.y
        q = self.curr_pose_.orientation
        curr_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        target_angle = math.atan2(y_error, x_error)

        linear_error = math.sqrt((x_error) ** 2 + (y_error) ** 2)
        angular_error = target_angle - curr_angle

        self.get_logger().info(f"angle: {curr_angle} | Error: linear {linear_error} | angular {angular_error}")

        if abs(linear_error) < self.threshold_ and abs(angular_error) < self.threshold_:
            # if the error is small enough, we have succeeded
            self.cmd_publisher_.publish(Twist())  # make the robot stop
            time.sleep(1)  # wait to release flag AFTER stop is published
            self.goal_succeeded_ = True              
        elif self.goal_handle_ is not None:
            # feed error into PID controller, then move the robot
            self.cmd_vel_.linear.x = self.u_linear.next(linear_error)
            self.cmd_vel_.angular.z = self.u_angular.next(angular_error)
            self.cmd_publisher_.publish(self.cmd_vel_)
 
            # update the caller with feedback on current state
            feedback_msg = MoveToPose.Feedback()
            feedback_msg.current = self.curr_pose_
            self.goal_handle_.publish_feedback(feedback_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
