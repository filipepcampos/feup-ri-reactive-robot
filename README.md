# Reactive Wall Following Robot

## Description
This is a reactive wall following robot that uses a laser scanner to detect obstacles and a differential drive to move around. The robot is simulated in Gazebo and the control is done using ROS2.

## Instructions

Build the docker image using:
```bash
docker build -t ros2-gazebo .
```

Run the docker image using:
```bash
docker compose up -d
```

Exec using:
```bash
xhost +local:root # allow access to the X server
docker exec -it ros2-it bash
```
