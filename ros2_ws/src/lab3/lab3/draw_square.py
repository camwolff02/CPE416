import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D
from cpe416_interfaces.action import MoveToPose


class DrawSquare(Node):
    def __init__(self):
        super().__init__("draw_square")
        self._action_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.goal_msg_ = MoveToPose.Goal()

    def send_goal(self, goal: Pose2D):
        self.goal_msg_.goal = goal

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(self.goal_msg_)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :) driving to point')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    draw_square = DrawSquare()
    current_goal = Pose2D()

    # first line
    current_goal.x += 10
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # turn left
    current_goal.theta += math.pi / 2
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # second line
    current_goal.y += 10
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # turn left
    current_goal.theta += math.pi / 2
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # third line
    current_goal.x -= 10
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # turn left
    current_goal.theta += math.pi / 2
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # fourth line
    current_goal.y -= 10
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)
    # turn left
    current_goal.theta -= math.pi / 2
    draw_square.send_goal(current_goal)
    rclpy.spin(draw_square)

    draw_square.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
