# Reactive Wall Following Robot

## Description
This is a reactive wall-following robot that uses a laser scanner to detect obstacles and a differential drive to move around. The robot is simulated in Gazebo and the control is done using ROS2.

## Instructions

## Docker setup

Install [Docker](https://www.docker.com/) for your preferred operating system.

To start a Docker container simply run the following command in the root of the repository:
```bash
docker compose up -d --build
```

To grant the container access to the X server, the following command is required. Without it, the Gazebo simulation environment will crash. This step needs to be done only once for a new desktop session.
```bash
xhost +local:root
```

To start a terminal inside the container type:
```bash
docker exec -it ros2-it bash
```

## Setup without Docker
If you wish to run this project we are using ROS2 Iron and Gazebo Classic. For more dependencies, please emulate what is installed in the [Dockerfile](https://github.com/filipepcampos/feup-ri-reactive-robot/blob/55585e06fe7109fc3b3b1936e3a66848090f58f4/Dockerfile#L7C1-L7C1).

## Running the Simulation

For this setup, you will require 3 terminals running inside the container (or a single multiplexed terminal using `tmux`, pre-installed in the image).
The following instructions assume that these commands have been run already:
```bash
cd /ws
colcon build # it is only required once
source install/setup.bash
```

**Start the world**:
```bash
ros2 launch robot_gazebo start_world.launch.py
```

**Spawn the URDF robot**:
```bash
ros2 launch robot_gazebo spawn_robot_ros2.launch.xml
```

**Control the robot**:
To use our wall-following algorithm use:
```bash
ros2 run robot_controller movement --ros-args --params-file src/robot_controller/param/params.yaml
```
These two commands are useful for testing purposes. The first allows the user to manually control the robot and the second resets the world to its starting position.

# Acknowledgments

The robot description was adapted from the following [repository](https://bitbucket.org/theconstructcore/box_bot/src/master/box_bot_description/)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/reactive_robot/cmd_vel
ros2 service call /reset_world std_srvs/srv/Empty
```
