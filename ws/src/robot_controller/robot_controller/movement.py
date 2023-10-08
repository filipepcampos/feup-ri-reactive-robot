import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import numpy as np


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


    def subScan(self) -> None:
        print(f"Subscribing to /{self.robot_name}/laser_scan")
        sub = self.create_subscription(LaserScan, f"/{self.robot_name}/laser_scan", self._scanCallback, 10)


    def _scanCallback(self, msg: LaserScan) -> None:
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        ranges = np.array(msg.ranges)
        min_index = np.argmin(ranges)
        print(f"Min index: {min_index}\nMin angle: {angle_min + min_index * angle_increment}\nMin value: {ranges[min_index]}")



def main(args=None):
    rclpy.init(args=args)

    robot_movement = RobotMovement("box_bot")
    robot_movement.subScan()


    rclpy.spin(robot_movement)
    rclpy.shutdown()



if __name__ == "__main__":
    main()

    