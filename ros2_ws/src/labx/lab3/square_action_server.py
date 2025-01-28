import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Pose2D

class PoseActionServer(Node):
    def __init__(self):
        super().__init__('pose_action_server')

        # PID constants
        self.P_ANGULAR = 0.75
        self.I_ANGULAR = 0.15
        self.D_ANGULAR = 0.2

        self.P_LINEAR = 1
        self.I_LINEAR = 1
        self.D_LINEAR = 1

        self._action_server = ActionServer(
            self,
            Pose2D,
            'desired_pose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Pose2D.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Pose.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = PoseActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()