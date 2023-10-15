import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

from helper import foo


def rads_to_deg(rads: float) -> float:
    return rads * 180 / math.pi


class LaserPoint:
    def __init__(self, index: float, distance: float, angle: float) -> None:
        self.angle = angle
        self.index = index
        self.distance = distance
        self.x = distance * math.cos(angle)
        self.y = distance * math.sin(angle)

    def __str__(self) -> str:
        return f"Point(idx: {self.index}, distance: {self.distance:.3f}, angle: {self.angle:.3f})"

def create_laser_point(index: float, msg: LaserScan) -> LaserPoint:
    return LaserPoint(
        index=index,
        distance=msg.ranges[index],
        angle=msg.angle_min + index * msg.angle_increment,
    )

def get_closest_point(msg: LaserScan) -> LaserPoint:
    ranges = np.array(msg.ranges)
    closest_point_index = np.argmin(ranges)
    return create_laser_point(closest_point_index, msg)

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
        super().__init__("robot_controller")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_distance', 1.0),
                ('k1', 1.0),
                ('k2', 1.0),
                ('finish_distance', 1.7),
                ('finish_max_error', 0.1),
            ]
        )

        self.get_logger().info("Creacted node")

        self.target_distance = self.get_parameter("target_distance").get_parameter_value().double_value
        self.k1 = self.get_parameter("k1").get_parameter_value().double_value
        self.k2 = self.get_parameter("k2").get_parameter_value().double_value
        self.finish_distance = self.get_parameter("finish_distance").get_parameter_value().double_value
        self.finish_max_error = self.get_parameter("finish_max_error").get_parameter_value().double_value

        self.robot_name = robot_name
        self.vel = Twist()

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

    def _stop(self, msg: LaserScan) -> bool:
        closest_point = get_closest_point(msg)
        ranges = np.array(msg.ranges)

        if(np.sum(ranges < 999999) > len(ranges)/2):
            return False
   
        valid_ranges = np.where(ranges < 999999)[0] # Remove points that are too far
        print(valid_ranges)

        for i in range(1, len(valid_ranges)):
            if valid_ranges[i] != valid_ranges[i-1] + 1:
                return False

        # Remove points from other side of the robot
        if closest_point.angle > 0:
            valid_ranges = valid_ranges[valid_ranges > len(ranges)/2]
        else:
            valid_ranges = valid_ranges[valid_ranges < len(ranges)/2]

        if len(valid_ranges) < 2:
            return False
        
        print("AYYYYY")
        
        p1 = create_laser_point(valid_ranges[0], msg)
        p2 = create_laser_point(valid_ranges[-1], msg)

        p1_relative_angle = abs(p1.angle - closest_point.angle)
        p2_relative_angle = abs(p2.angle - closest_point.angle)

        print(abs(p1.angle-closest_point.angle)*180/math.pi)
        print(abs(p2.angle-closest_point.angle)*180/math.pi)

        if p1_relative_angle > 75 * math.pi/180 or p2_relative_angle > 75 * math.pi/180: # Too wide
            print("A")
            return False

        if abs(abs(p1.angle-closest_point.angle) - (abs(p2.angle-closest_point.angle))) > 0.75: # Should describe a triangle
            print("B")
            return False
        
        
        x_distance = abs(p1.x - p2.x)
        print(f"X-Distance: {x_distance:.4f}, w: {abs(x_distance - self.finish_distance):.4f}")

        return abs(x_distance - self.finish_distance) < self.finish_max_error


    def _scan_callback(self, msg: LaserScan) -> None:
        if self._stop(msg):
            linear_vel = 0.0
            angular_vel = 0.0
            self.get_logger().info("\n---------------\nIssued STOP command\n------------------\n")
        else:
            closest_point = get_closest_point(msg)
            relative_distance = closest_point.distance - self.target_distance

            linear_vel = 1.0
            direction = 1 if closest_point.angle > 0 else -1
            delta_angle = closest_point.angle -  direction * math.pi/2
            self.get_logger().info(f"Closest angle: {closest_point.angle:.4f} ({rads_to_deg(closest_point.angle)}º)")
            self.get_logger().info(f"Delta angle: {delta_angle:.4f} ({rads_to_deg(delta_angle)}º)")
            self.get_logger().info(f"Relative distance: {relative_distance:.4f} ({closest_point.distance:.4f} - {self.target_distance:.4f})")
            
            angular_vel = self.k1 * relative_distance + self.k2 * delta_angle * direction
        self.change_vel(
            linear=linear_vel, 
            angular=angular_vel
        )
        

    # def _old_scan_callback(self, msg: LaserScan) -> None:
    #     closest_point = get_average_point(msg)
    #     print(f"Closest point ({closest_point})")

    #     lateral_point = get_closest_lateral_point(
    #         msg, closest_point
    #     )  # Probably useless?
    #     print(f"Closest lateral point ({lateral_point})")

    #     closest_lateral_sign = 1 if closest_point.angle > 0 else -1
        
    #     if abs(closest_point.angle - math.pi / 2) < 0.1:
    #         closest_point.angle += 0.1

    #     alpha = (
    #         abs(closest_point.angle) - math.pi / 2
    #     )  # This does not need ABS. Don't know why????

    #     target_distance = 0.40
    #     relative_distance = closest_point.distance - target_distance
    #     path_sign = np.sign(
    #         relative_distance
    #     )  # inside/outside target distance boundary
    #     steering_sign = closest_lateral_sign * path_sign

    #     new_ang_vel = steering_sign * alpha  # + relative_distance/10

    #     # Print variables
    #     print(f"Alpha: {rads_to_deg(alpha)}º")
    #     print(f"Path sign: {path_sign}")
    #     print(f"Steering sign: {steering_sign}")
    #     print(
    #         f"New angular velocity: {rads_to_deg(new_ang_vel)}º/s\n------------------"
    #     )

    #     k = 2.0
    #     self.change_vel(1.0, new_ang_vel * k)


def main(args=None):
    rclpy.init(args=args)

    robot_movement = RobotMovement("box_bot")
    robot_movement.sub_scan()

    rclpy.spin(robot_movement)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
