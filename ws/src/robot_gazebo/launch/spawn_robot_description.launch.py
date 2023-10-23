import random

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    random_pose = LaunchConfiguration("random_pose", default="true").perform(context)

    print("random_pose: {}, bool: {}".format(random_pose, random_pose == "true"))

    # 8x2
    x = random.uniform(-1.0, 1.0) if random_pose == "true" else 0.0 
    y = random.uniform(-3.0, 3.0) if random_pose == "true" else 0.0
    z = 0.2

    position = [x, y, z]
    orientation = [0.0, 0.0, random.uniform(-3.14, 3.14)] if random_pose == "true" else [0.0, 0.0, 0.0]

    robot_base_name = "reactive_robot"

    entity_name = robot_base_name  # + "_" + str(random.randint(0, 1000000))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-entity",
            entity_name,
            "-x",
            str(position[0]),
            "-y",
            str(position[1]),
            "-z",
            str(position[2]),
            "-R",
            str(orientation[0]),
            "-P",
            str(orientation[1]),
            "-Y",
            str(orientation[2]),
            "-topic",
            "/robot_description",
        ],
    )

    return [spawn_robot]


def generate_launch_description():

    random_pose_arg = DeclareLaunchArgument(
            "random_pose", default_value="true",
            description="Randomize robot pose")
    



    return LaunchDescription([
        random_pose_arg,
        OpaqueFunction(function=launch_setup)])
