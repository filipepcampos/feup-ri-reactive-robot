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
    def __init__(self, robot_name: str, target_distance : float) -> None:
        super().__init__("asdf")
        self.get_logger().info("Creacted node")

        self.robot_name = robot_name
        self.vel = Twist()
        self.target_distance = target_distance

        self.pub = self.create_publisher(Twist, f"/{robot_name}/cmd_vel", 1)
 

    def change_vel(self, linear: float, angular: float) -> None:
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)

    def sub_scan(self) -> None:
        print(f"Subscribing to /{self.robot_name}/laser_scan")
        sub = self.create_subscription(
            LaserScan, f"/{self.robot_name}/laser_scan", self._scan_callback, 10
        )
    
    def _found_wall(self, msg: LaserScan) -> bool: 
        return np.sum(np.isinf(msg.ranges)) != len(msg.ranges)
        
    def _stop_condition(self, msg: LaserScan, closest_point: LaserPoint) -> bool:
        ranges = np.array(msg.ranges)
        max_linear = 1.0
        # print(f"Ranges: {ranges}")
        
        # stopping conditions 
        # at least half of the values are inf
        if np.sum(np.isinf(ranges)) < len(ranges)/2:
           return max_linear

        wall_values = np.where(ranges < 5)[0]
        # print(f"Wall values: {wall_values}")
        if len(wall_values) < 2:
            return max_linear
        
       
        point_1_index = wall_values[0]
        point_2_index = wall_values[-1]

        # points = []
        # for point_index in wall_values:
        #     angle = msg.angle_min + point_index * msg.angle_increment
        #     points.append((math.cos(angle)*ranges[point_index], math.sin(angle)*ranges[point_index]))
        
        # reference_sin = points[0][1]
        # for point in points:
        #     if abs(point[1] - reference_sin) > 0.3:
        #         print("------\n\n\n\nNot colinear")
        #         # print point coordinates
        #         print(f"Point 1: {points[0]}")
        #         print(f"Point 2: {point}")
        #         return max_linear
        
        create_point = lambda idx: LaserPoint(
                index=idx,
                distance=ranges[idx],
                angle=msg.angle_min + idx * msg.angle_increment,
                )
        
        point_1, point_2 = (create_point(idx) for idx in [point_1_index, point_2_index])

        # if they are not in the same side of the closest point
        if np.sign(point_1.angle) != np.sign(point_2.angle) or np.sign(point_1.angle) != np.sign(closest_point.angle):
            return max_linear

        # get the angle between closest point and the other two
        angle_1 = abs(point_1.angle - closest_point.angle)
        angle_2 = abs(point_2.angle - closest_point.angle)

        wall_size = math.sin(angle_1) * point_1.distance + math.sin(angle_2) * point_2.distance

        self.get_logger().info(f"Angle 1: {rads_to_deg(angle_1)}")
        self.get_logger().info(f"Angle 2: {rads_to_deg(angle_2)}")
        self.get_logger().info(f"Wall size: {wall_size}")

        # return abs(wall_size - 4.0) < 1.0
        
        if (abs(wall_size - 4.0) < 1.0):
            diff_angle = abs(angle_1 - angle_2)
            return diff_angle * (max_linear/0.3) if diff_angle > 0.2 else 0.0 #0.22689
        return max_linear
        if (abs(wall_size - 4.0) < 1.0):
            if (angle_1 - angle_2) < 0.1:
                return 2
            return 1
        return False

    def _compute_angular_velocity(self, msg: LaserScan, closest_point: LaserPoint) -> float:
        alpha = (
            abs(closest_point.angle) - math.pi / 2
        )  # This does not need ABS. Don't know why????
 
        closest_lateral_sign = 1 if closest_point.angle > 0 else -1

        relative_distance = closest_point.distance - self.target_distance
        path_sign = np.sign(relative_distance)  # inside/outside boundary
        steering_sign = closest_lateral_sign * path_sign

        new_ang_vel = steering_sign * alpha  # + relative_distance/10

        # Print variables
        print(f"Alpha: {rads_to_deg(alpha)}º")
        print(f"Path sign: {path_sign}")
        print(f"Steering sign: {steering_sign}")
        print(
            f"New angular velocity: {rads_to_deg(new_ang_vel)}º/s\n------------------"
        )

        return new_ang_vel

    def _scan_callback(self, msg: LaserScan) -> None:
        closest_point = get_closest_point(msg)
        print(f"Closest point ({closest_point})")

        lateral_point = get_closest_lateral_point(
            msg, closest_point
        )  # Probably useless?
        print(f"Closest lateral point ({lateral_point})")

        if abs(closest_point.angle - math.pi / 2) < 0.1:
            closest_point.angle += 0.1

        
        # if self._stop_condition(msg, closest_point):
        #     self.get_logger().info("\n\n\n\nStopping\n\n\n\n")
        #     linear = 0.0
        #     new_ang_vel = 0.0
        # stop_action = self._stop_condition(msg, closest_point)
        # if stop_action == 1:
        #     self.get_logger().info("\n\n\n\nStopping\n\n\n\n")
        #     k = 1.0
        #     linear = 0.5
        #     new_ang_vel = self._compute_angular_velocity(msg, closest_point) * k
        # elif stop_action == 2:
        #     self.get_logger().info("\n\n\n\nStopping\n\n\n\n")
        #     linear = 0.0
        #     new_ang_vel = 0.0
        # else:
            # k = 2.0
            # linear = 1.0
            # new_ang_vel = self._compute_angular_velocity(msg, closest_point) * k
        
        k = 2.0
        linear = self._stop_condition(msg, closest_point)
        new_ang_vel = self._compute_angular_velocity(msg, closest_point) * k

        self.change_vel(linear, new_ang_vel)


def main(args=None):
    rclpy.init(args=args)

    robot_movement = RobotMovement("box_bot", 1)
    robot_movement.sub_scan()

    rclpy.spin(robot_movement)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
