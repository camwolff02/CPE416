#!/usr/bin/env python3
import math
from enum import Enum
from queue import Queue
import rclpy
from rclpy.node import Node
import rclpy.executors
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D
from cpe416_interfaces.action import MoveToPose, TurnToAngle


class Move(Enum):
    STRAIGHT = 1
    TURN = 2


class DrawSquare(Node):
    def __init__(self):
        super().__init__("draw_square")
        self._straight_action_client = ActionClient(self, MoveToPose, "move_to_pose")
        self._turn_action_client = ActionClient(self, TurnToAngle, "turn_to_angle")

        self._request_queue: Queue[MoveToPose.Goal|TurnToAngle.Goal] = Queue()

    def send_straight_goal(self, goal: Pose2D) -> None:
        goal_msg = MoveToPose.Goal()
        goal_msg.goal.x = goal.x
        goal_msg.goal.y = goal.y
        self.get_logger().info(f"Adding goal: ({goal.x}m, {goal.y}m, {goal.theta}deg)")
        self._request_queue.put(goal_msg)

    def send_turn_goal(self, goal: float) -> None:
        goal_msg = TurnToAngle.Goal()
        goal_msg.goal = goal
        self.get_logger().info(f"Adding goal: {goal}deg")
        self._request_queue.put(goal_msg)

    def start(self) -> None:
        if self._request_queue.empty(): 
            self.get_logger().info(f"All tasks complete!")
            rclpy.shutdown()
            return

        goal_msg = self._request_queue.get()
        goal = goal_msg.goal

        if type(goal_msg) is MoveToPose.Goal:
            self.get_logger().info(f"Sending goal: ({goal.x}m, {goal.y}m, {goal.theta}deg)")
            self._straight_action_client.wait_for_server()
            self._send_goal_future = self._straight_action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info(f"Sending goal: {goal}deg")
            self._turn_action_client.wait_for_server()
            self._send_goal_future = self._turn_action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :) driving to point")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.destination))
        self.start()  # process next request in queue


def main(args=None):
    rclpy.init(args=args)

    # Create node and queue server requests
    draw_square = DrawSquare()
    current_goal = Pose2D()
    turn_goal = 0
    side_len = 5

    # first line
    current_goal.x += side_len
    draw_square.send_straight_goal(current_goal)
    # turn left
    turn_goal += math.pi / 2
    draw_square.send_turn_goal(turn_goal)
    # second line
    current_goal.y += side_len
    draw_square.send_straight_goal(current_goal)
    # turn left
    turn_goal += math.pi / 2
    draw_square.send_turn_goal(turn_goal)
    # third line
    current_goal.x -= side_len
    draw_square.send_straight_goal(current_goal)
    # turn left
    turn_goal += math.pi / 2
    draw_square.send_turn_goal(turn_goal)
    # fourth line
    current_goal.y -= side_len
    draw_square.send_straight_goal(current_goal)
    # turn left
    turn_goal -= math.pi / 2
    draw_square.send_turn_goal(turn_goal)

    # start node to send requests
    draw_square.start()
    rclpy.spin(draw_square)
    draw_square.destroy_node()


if __name__ == "__main__":
    main()
