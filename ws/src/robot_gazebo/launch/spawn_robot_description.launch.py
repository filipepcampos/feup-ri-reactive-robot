import random

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

pos = {
    'pos1': ([-0.29818344712671063,-0.3078749588767158,0.2], [0.0,0.0,-0.5773728460641534]),
    'pos2': ([-0.035294648107484505,-1.9874038155023217,0.2], [0.0,0.0,0.5755361008769788]),
    'pos3': ([-0.4499376372869217,0.0018420373614103092,0.2], [0.0,0.0,-1.0923946199675307]),
    'pos4': ([0.07714259167643789,-2.544121073422243,0.2], [0.0,0.0,-1.2529362020158463]),
    'pos5': ([0.14783795424858326,-0.2962641667728252,0.2], [0.0,0.0,2.601097075963406])
}

def launch_setup(context, *args, **kwargs):
    random_pose = LaunchConfiguration("random_pose", default="true").perform(context)
    #pos_number = LaunchConfiguration("pos_number", default="true").perform(context)



    print("random_pose: {}, bool: {}".format(random_pose, random_pose == "true"))

    # 8x2
    x = random.uniform(-1.0, 1.0) if random_pose == "true" else 0.0 # 10.0
    y = random.uniform(-3.0, 3.0) if random_pose == "true" else 0.0 # -15.0
    z = 0.2

    #position = [x, y, z]
    #orientation = [0.0, 0.0, random.uniform(-3.14, 3.14)] if random_pose == "true" else [0.0, 0.0, 0.0]
    position = pos['pos2'][0]
    orientation = pos['pos2'][1]    

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
    
    #pos_number = DeclareLaunchArgument(
    #        "pos_number", default_value="1",
    #        description="Randomize robot pose")

    return LaunchDescription([
        random_pose_arg,
        #pos_number, 
        OpaqueFunction(function=launch_setup)])
