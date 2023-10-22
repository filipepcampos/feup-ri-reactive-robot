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
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    print("A")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[{"robot_description": ParameterValue(Command(["xacro", " ", robot_desc_path]), value_type=str), "use_sim_time": True}],
        output="screen",
    )

    print("B")
    
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), "rviz", "reactive_robot.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_dir],
        output="screen",
    )
    
    return LaunchDescription([robot_state_publisher_node, rviz_node])