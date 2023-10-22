import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


DEBUG_PRINTS = False
TEST_PRINTS = True


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
                ('max_linear_velocity', 5.0),
                ('max_angular_velocity', 5.0),
            ]
        )

        self.logger_debug("Creacted node")

        self.target_distance = self.get_parameter("target_distance").get_parameter_value().double_value
        self.k1 = self.get_parameter("k1").get_parameter_value().double_value
        self.k2 = self.get_parameter("k2").get_parameter_value().double_value
        self.finish_distance = self.get_parameter("finish_distance").get_parameter_value().double_value
        self.finish_max_error = self.get_parameter("finish_max_error").get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value

        self.logger_test(f"k1: {self.k1}")
        self.logger_test(f"k2: {self.k2}")
        self.logger_test(f"Max_linear: {self.max_linear_velocity}")
        self.logger_test(f"Max_angular: {self.max_angular_velocity}")
        self.logger_test(f"Target_distance: {self.target_distance}")

        self.robot_name = robot_name
        self.vel = Twist()

        self.pub = self.create_publisher(Twist, f"/{robot_name}/cmd_vel", 1)


    def logger_debug(self, msg : str) -> None: 
        if DEBUG_PRINTS:
            self.get_logger().info(msg)

    def logger_test(self, msg : str) -> None:
        if TEST_PRINTS:
            self.get_logger().info(msg)


    def change_vel(self, linear: float, angular: float) -> None:
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)

    def sub_scan(self) -> None:
        self.logger_debug(f"Subscribing to /{self.robot_name}/laser_scan")
        sub = self.create_subscription(
            LaserScan, f"/{self.robot_name}/laser_scan", self._scan_callback, 10
        )
    
    def _wander(self, msg: LaserScan) -> bool:
        return all([math.isinf(x) for x in msg.ranges])

    def _wander_move(self) -> (float, float):
        rand_num = np.random.rand()

        if rand_num < 0.5:
            vel = (4.0, 0.0) 
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
        self.logger_debug(f"X-Distance: {x_distance:.4f}, w: {abs(x_distance - self.finish_distance):.4f}")

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
            self.logger_debug("\n---------------\nIssued WANDER command\n------------------\n")
        elif self._stop(msg):
            linear_vel = 0.0
            angular_vel = 0.0
            self.logger_debug("\n---------------\nIssued STOP command\n------------------\n")
        else:
            closest_point = get_closest_point(msg)
            relative_distance = closest_point.distance - self.target_distance

            linear_vel = 1.0
            #direction =  if closest_point.angle > 0 else -1

            delta_angle = 0.0
            wall_side = "left" if closest_point.angle > 0 else "right"
            if wall_side == "left":
                # print("WALL_LEFT")
                delta_angle = abs(closest_point.angle) - math.pi/2
            elif wall_side == "right":
                # print("WALL_RIGHT")
                delta_angle = math.pi/2 - abs(closest_point.angle)

            if wall_side == "right":
                relative_distance *= -1 

            self.logger_debug(f"Closest angle: {closest_point.angle:.4f} ({rads_to_deg(closest_point.angle)}º)")
            self.logger_debug(f"Delta angle: {delta_angle:.4f} ({rads_to_deg(delta_angle)}º)")
            self.logger_debug(f"Relative distance: {relative_distance:.4f} ({closest_point.distance:.4f} - {self.target_distance:.4f})")
            


            angular_vel = self.k1 * relative_distance + self.k2 * delta_angle
            
            self.logger_debug(f"Angular vel: {angular_vel:.4f}")


            # angular_vel ~ 0.0 then linear_vel ~ max
            # angular_vel ~ max then linear_vel ~ 0.0 
            linear_vel *= max( 
                min(1/abs(angular_vel + 0.0001), 1.0),  # 0.0001 to avoid division by 0
                0.2
            )

            # Direction;Wall_distance;angular_vel;linear_vel
            self.logger_test(f"{wall_side};{closest_point.distance:.4f};{relative_distance:.4f};{angular_vel:.4f};{linear_vel:.4f}")
            # can we log it to a file or a node? n

        self.change_vel(
            linear=min(linear_vel, self.max_linear_velocity),  
            angular=min(angular_vel, self.max_angular_velocity)
        )
        



def main(args=None):
    rclpy.init(args=args)

    robot_movement = RobotMovement("reactive_robot")
    robot_movement.sub_scan()

    rclpy.spin(robot_movement)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
