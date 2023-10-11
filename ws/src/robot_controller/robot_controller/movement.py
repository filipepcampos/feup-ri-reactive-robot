import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np


def rads_to_deg(rads: float) -> float:
    return rads * 180 / math.pi


class LaserPoint:
    def __init__(self, index: float, distance: float, angle: float) -> None:
        self.angle = angle
        self.index = index
        self.distance = distance

    def __str__(self) -> str:
        return f"index: {self.index}, distance: {self.distance}, angle: {self.angle}"


def get_closest_point(msg: LaserScan) -> LaserPoint:
    ranges = np.array(msg.ranges)
    closest_point_index = np.argmin(ranges)
    point = LaserPoint(
        index=closest_point_index,
        distance=ranges[closest_point_index],
        angle=msg.angle_min + closest_point_index * msg.angle_increment,
    )
    return point

def get_average_point(msg: LaserScan) -> LaserPoint:
    closest_point = get_closest_point(msg)
    forward_point_index = int(len(msg.ranges)/2)
    if msg.ranges[forward_point_index] < 999999 and msg.ranges[forward_point_index] < closest_point.distance + 1.0:
        return LaserPoint(
            index=closest_point.index,
            distance=closest_point.distance,
            angle=(closest_point.angle)/2,
        )
    return closest_point


def get_closest_lateral_point(
    msg: LaserScan, closest_point: LaserPoint
) -> (LaserPoint, LaserPoint):
    closest_lateral_sign = 1 if closest_point.angle > 0 else -1
    lateral_angle = closest_lateral_sign * math.pi / 2
    lateral_index = int((lateral_angle - msg.angle_min) / msg.angle_increment)
    return LaserPoint(
        index=lateral_index,
        distance=msg.ranges[lateral_index],
        angle=lateral_angle,
    )


class RobotMovement(Node):
    def __init__(self, robot_name: str) -> None:
        super().__init__("asdf")
        self.get_logger().info("Creacted node")

        self.robot_name = robot_name
        self.vel = Twist()

        self.pub = self.create_publisher(Twist, f"/{robot_name}/cmd_vel", 1)

        # if CTurtle.doOdometry:
        #     print('"seq","sec","x","y"')
        #     rospy.Subscriber(
        #         "/odometry/ground_truth", Odometry, self._odometryGroundTruth
        #     )

    def change_vel(self, linear: float, angular: float) -> None:
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)

    def sub_scan(self) -> None:
        print(f"Subscribing to /{self.robot_name}/laser_scan")
        sub = self.create_subscription(
            LaserScan, f"/{self.robot_name}/laser_scan", self._scan_callback, 10
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        closest_point = get_average_point(msg)
        print(f"Closest point ({closest_point})")

        lateral_point = get_closest_lateral_point(
            msg, closest_point
        )  # Probably useless?
        print(f"Closest lateral point ({lateral_point})")

        closest_lateral_sign = 1 if closest_point.angle > 0 else -1
        
        if abs(closest_point.angle - math.pi / 2) < 0.1:
            closest_point.angle += 0.1

        alpha = (
            abs(closest_point.angle) - math.pi / 2
        )  # This does not need ABS. Don't know why????

        target_distance = 0.40
        relative_distance = closest_point.distance - target_distance
        path_sign = np.sign(
            relative_distance
        )  # inside/outside target distance boundary
        steering_sign = closest_lateral_sign * path_sign

        new_ang_vel = steering_sign * alpha  # + relative_distance/10

        # Print variables
        print(f"Alpha: {rads_to_deg(alpha)}º")
        print(f"Path sign: {path_sign}")
        print(f"Steering sign: {steering_sign}")
        print(
            f"New angular velocity: {rads_to_deg(new_ang_vel)}º/s\n------------------"
        )

        k = 2.0
        self.change_vel(1.0, new_ang_vel * k)


def main(args=None):
    rclpy.init(args=args)

    robot_movement = RobotMovement("box_bot")
    robot_movement.sub_scan()

    rclpy.spin(robot_movement)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
