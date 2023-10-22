import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    urdf_file = "reactive_robot.urdf"
    package_description = "robot_description"

    print("Fetching URDF -->")
    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "urdf", urdf_file
    )

    entity_name = "reactive_robot"

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[
            {
                "frame_prefix": entity_name + "/",
                "use_sim_time": True,
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_desc_path, " robot_name:=", entity_name]),
                    value_type=str,
                ),
            }
        ],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher_node",
        parameters=[
            {
                "frame_prefix": entity_name + "/",
                "use_sim_time": True,
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_desc_path, " robot_name:=", entity_name]),
                    value_type=str,
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
