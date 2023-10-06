import random

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_base_name = "box_bot"

    entity_name = robot_base_name + "_" + str(random.randint(0, 1000000))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=["-entity",
                   entity_name,
                   "-x", str(position[0]),"-y", str(position[1]),"-z", str(position[2]),
                     "-R", str(orientation[0]),"-P", str(orientation[1]),"-Y", str(orientation[2]),
                     "-topic", "/robot_description"]
    )

    return LaunchDescription([spawn_robot])