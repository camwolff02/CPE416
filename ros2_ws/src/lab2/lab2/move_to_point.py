import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose

import math


class MoveToPos(Node):
    def __init__(self):
        node_name = "move_to_point"
        super().__init__(node_name)

        # Initialize Proportional (P) control constants
        self.k_orientation = 0.75
        self.k_velocity = 0.15

        self.threshold = 0.2  # stopping threshold [m, rad]

        # Initialize target pose from parameters
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("theta", math.pi)
        self._desired_pose = Pose2D()
        self._update_desired_pose()

        # Initialize variables to hold state of node
        self._cmd_vel = Twist()
        self._pose = Pose()
        self._has_pose = False

        # Subscribe to the turtle's position, publish to the turtle's velocity
        self.create_subscription(Pose, "/turtle1/pose", self._get_pose, 10)
        self._timer_period = 1  # [seconds]
        self.create_timer(self._timer_period, self._move)

        self.declare_parameter("cmd_vel_topic", "/turtle1/cmd_vel")
        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._publish_cmd_vel = self.create_publisher(Twist, topic, 10)

    def angular_controller(self, desired_angle: float) -> float:
        return self.k_orientation * (desired_angle - self._pose.theta)

    def linear_controller(self) -> float:
        return self.k_velocity * math.sqrt(
            (self._desired_pose.x - self._pose.x) ** 2
            + (self._desired_pose.y - self._pose.y) ** 2
        )

    def _update_desired_pose(self) -> None:
        self._desired_pose.x = (
            self.get_parameter("x").get_parameter_value().double_value
        )
        self._desired_pose.y = (
            self.get_parameter("y").get_parameter_value().double_value
        )
        self._desired_pose.theta = (
            self.get_parameter("theta").get_parameter_value().double_value
        )

    def _get_pose(self, pose_msg: Pose) -> None:
        self._pose = pose_msg
        self._has_pose = True
        self._update_desired_pose()

    def _move(self) -> None:
        if not self._has_pose:
            return  # Only run if we have gotten the turtle's position

        self._cmd_vel.angular.z = 0.0
        self._cmd_vel.linear.x = 0.0

        x_error = self._desired_pose.x - self._pose.x
        y_error = self._desired_pose.y - self._pose.y

        if abs(x_error) > self.threshold or abs(y_error) > self.threshold:
            desired_angle = math.atan2(y_error, x_error)
            self._cmd_vel.angular.z = self.angular_controller(desired_angle)
            self._cmd_vel.linear.x = self.linear_controller()
        elif abs(self._desired_pose.theta - self._pose.theta) > self.threshold:
            self._cmd_vel.angular.z = self.angular_controller(self._desired_pose.theta)

        self.get_logger().info(
            f"x: ({self._pose.x:.2f}, y: {self._pose.y:.2f}, theta: {self._pose.theta:.2f})"
        )
        self._publish_cmd_vel.publish(self._cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
