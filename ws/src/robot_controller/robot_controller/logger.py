import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotOdometryLogger(Node):
    def __init__(self, robot_name: str) -> None:
        super().__init__("asdf")
        self.get_logger().info("Creacted node")

        self.robot_name = robot_name
        self.vel = Twist()

    def sub_odometry(self) -> None:
        sub = self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._odom_callback, 10
        )

    def _odom_callback(self, msg: Odometry) -> None:
        with open(f"{self.robot_name}_odometry.csv", "a") as f:
            f.write(f"{msg.header.stamp.sec},{msg.header.stamp.nanosec},{msg.pose.pose.position.x},{msg.pose.pose.position.y},{rads_to_deg(msg.pose.pose.orientation.z)}\n")
        

def main(args=None):
    rclpy.init(args=args)

    robot_logger = RobotOdometryLogger("box_bot")
    robot_logger.sub_odometry()

    rclpy.spin(robot_logger)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
