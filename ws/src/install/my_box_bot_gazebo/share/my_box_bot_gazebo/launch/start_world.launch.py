import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_reactive_robot_gazebo = get_package_share_directory('my_reactive_robot_gazebo')

    description_package_name = "robot_description"
    install_dir = get_package_prefix(description_package_name)

    gazebo_models_path = os.path.join(pkg_reactive_robot_gazebo, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ":" + gazebo_models_path
    
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] += ':' + install_dir + "/lib"
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + "/lib"
    
    print("GAZEBO_MODEL_PATH: " + os.environ['GAZEBO_MODEL_PATH'])
    print("GAZEBO_PLUGIN_PATH: " + os.environ['GAZEBO_PLUGIN_PATH'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_reactive_robot_gazebo, 'worlds', 'reactive_robot_empty.world'), ''],
            description='SDF world file'),
        gazebo
    ])