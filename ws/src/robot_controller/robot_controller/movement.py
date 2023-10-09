import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

def rads_to_deg(rads: float) -> float:
    return rads * 180 / math.pi

class RobotMovement(Node):
    def __init__(self, robot_name: str) -> None:
        super().__init__('asdf')
        self.get_logger().info('Creacted node')
        #node = rclpy.create_node('asdf')
        #node.get_logger().info('Created node')
        
        self.robot_name = robot_name
        self.vel = Twist()

        #self.pub = rospy.Publisher(f"/{robot_name}/cmd_vel", Twist, queue_size=1)
        self.pub = self.create_publisher(Twist, f"/{robot_name}/cmd_vel", 1)


        # if CTurtle.doOdometry:
        #     print('"seq","sec","x","y"')
        #     rospy.Subscriber(
        #         "/odometry/ground_truth", Odometry, self._odometryGroundTruth
        #     )
        self.change_vel(0.0, 0.0)
    
    def change_vel(self, linear: float, angular: float) -> None:
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular 
        self.pub.publish(vel)


    def sub_scan(self) -> None:
        print(f"Subscribing to /{self.robot_name}/laser_scan")
        sub = self.create_subscription(LaserScan, f"/{self.robot_name}/laser_scan", self._scan_callback, 10)


    def _scan_callback(self, msg: LaserScan) -> None:
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        ranges = np.array(msg.ranges)
        closest_point_index = np.argmin(ranges)
        closest_point_angle = angle_min + closest_point_index * angle_increment
        closest_point_distance = ranges[closest_point_index]
        print(f"Min index: {closest_point_index}")
        print(f"Min angle: {rads_to_deg(angle_min + closest_point_index * angle_increment)}º")
        print(f"Min distance: {closest_point_distance}\n------------------")

        closest_lateral_sign = 1 if closest_point_angle > 0 else -1 
        
        lateral_angle = closest_lateral_sign * math.pi/2
        lateral_index = int((lateral_angle - angle_min) / angle_increment)

        #angle_between_point_and_lateral = abs(lateral_angle - closest_point_angle)

        lateral_distance = ranges[lateral_index]
        print(f"Lateral distance: {lateral_distance} (Side={'left' if closest_lateral_sign > 0 else 'right'})")
        #print(f"Angle between point and lateral: {rads_to_deg(angle_between_point_and_lateral)}º\n------------------")

        if abs(closest_point_angle - math.pi/2) < 0.1:
            closest_point_angle += 0.1

        alpha = (abs(closest_point_angle)- math.pi/2)  # This does not need ABS. Don't know why????
        target_distance = 0.40
        relative_distance = closest_point_distance - target_distance
        path_sign = np.sign(relative_distance) #inside/outside target distance boundary
        steering_sign = closest_lateral_sign * path_sign
        

        new_ang_vel = steering_sign * alpha # + relative_distance/10

        # Print variables
        print(f"Alpha: {rads_to_deg(alpha)}º")
        print(f"Path sign: {path_sign}")
        print(f"Steering sign: {steering_sign}")
        print(f"New angular velocity: {rads_to_deg(new_ang_vel)}º/s\n------------------")
        

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

