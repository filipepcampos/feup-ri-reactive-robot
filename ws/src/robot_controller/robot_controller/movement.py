import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


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


class RobotMovement(Node):
    def __init__(self, robot_name: str) -> None:
        super().__init__("robot_controller")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_distance', 1.0),
                ('k1', 1.0),
                ('k2', 1.0),
                ('finish_distance', 3.65),
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
    
    def _wander(self, msg: LaserScan) -> bool:
        return all([math.isinf(x) for x in msg.ranges])

    def _wander_move(self) -> (float, float):
        rand_num = np.random.rand()

        if rand_num < 0.05:
            vel = (4.0, 0.0) 
        elif rand_num < 0.45:
            vel = (0.4, 1.0)
        else:
            vel = (0.4, -1.0)

        return vel
    
    def _stop(self, msg: LaserScan) -> bool:
        closest_point = get_closest_point(msg)
        ranges = np.array(msg.ranges)
   
        valid_ranges = np.where(ranges < 999999)[0] # Remove points that are too far

        # Condition 1: All points should be consecutive
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
        
        p1 = create_laser_point(valid_ranges[0], msg)
        p2 = create_laser_point(valid_ranges[-1], msg)

        # Condition 2: the extreme points should have approximately the same angle
        if abs(abs(p1.angle-closest_point.angle) - (abs(p2.angle-closest_point.angle))) > 0.35:
            return False
        
        
        x_distance = abs(p1.x - p2.x)
        print(f"X-Distance: {x_distance:.4f}, w: {abs(x_distance - self.finish_distance):.4f}")

        points = [create_laser_point(i, msg) for i in valid_ranges]
        x, y = [p.x for p in points], [p.y for p in points]
        line_fit_error = np.sum((np.polyval(np.polyfit(x, y, 1), x) - y)**2)

        # Condition 3: The points should be approximately in a line (to account for possible errors)
        if line_fit_error > 0.1:
            return False
        
        # Condition 4: The end wall distance should be approximately the same as self.finish_distance
        return abs(x_distance - self.finish_distance) < self.finish_max_error


    def _scan_callback(self, msg: LaserScan) -> None:
        if self._wander(msg):
            linear_vel, angular_vel = self._wander_move()
            self.get_logger().info("\n---------------\nIssued WANDER command\n------------------\n")
        elif self._stop(msg):
            linear_vel = 0.0
            angular_vel = 0.0
            self.get_logger().info("\n---------------\nIssued STOP command\n------------------\n")
        else:
            closest_point = get_closest_point(msg)
            relative_distance = closest_point.distance - self.target_distance

            linear_vel = 1.0
            #direction =  if closest_point.angle > 0 else -1

            delta_angle = 0.0
            wall_side = "left" if closest_point.angle > 0 else "right"
            if wall_side == "left":
                delta_angle = closest_point.angle - math.pi/2
            elif wall_side == "right":
                delta_angle = math.pi/2 - closest_point.angle

            if wall_side == "right":
                relative_distance *= -1 

            self.get_logger().info(f"Closest angle: {closest_point.angle:.4f} ({rads_to_deg(closest_point.angle)}º)")
            self.get_logger().info(f"Delta angle: {delta_angle:.4f} ({rads_to_deg(delta_angle)}º)")
            self.get_logger().info(f"Relative distance: {relative_distance:.4f} ({closest_point.distance:.4f} - {self.target_distance:.4f})")
            
            #angular_vel = self.k1 * relative_distance + self.k2 * delta_angle
            angular_vel = self.k2 * delta_angle
        self.change_vel(
            linear=linear_vel, 
            angular=angular_vel
        )
        



def main(args=None):
    rclpy.init(args=args)

    robot_movement = RobotMovement("box_bot")
    robot_movement.sub_scan()

    rclpy.spin(robot_movement)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
