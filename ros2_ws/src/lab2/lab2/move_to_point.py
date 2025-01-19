import rclpy
from rclpy.node import Node

import math

"""
    We're going to implement an open loop controller/FSM to have the turtlebot
    draw a square on the screen. Below I have commented parts of the code that
    you need to fill in to make the logic complete.
"""

# We have to use the geometry_msgs/msg/Twist to control robots
# Write in here what the correct import should be
from geometry_msgs.msg import Twist, Pose2D
from turtlesim_msgs.msg import Pose


class DrawSpiral(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('draw_spiral')

        self._publish_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._subscribe_pose = self.create_subscription(Pose, '/turtle1/pose')
        self._cmd_vel = Twist()

        self.declare_parameter('target_pos', rclpy.Parameter.Type.POSE2D)

        timer_period = 1.0  # [seconds]
        self._timer = self.create_timer(timer_period, self.timer_callback)



        # Initialize the robot pointing at the target position


    # Callback for the events
    def timer_callback(self):

        # Call publisher here
        self.get_logger().info("Robot is Still Turning!")
        if self.turn_msg.angular.z > 0.5:
            self.turn_msg.angular.z -= .5

        self.publisher_.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    draw_square = DrawSpiral()
    rclpy.spin_once(draw_square)
    draw_square.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
