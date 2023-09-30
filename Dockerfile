FROM osrf/ros:iron-desktop-full

# Install tmux
RUN sudo apt-get update; sudo apt-get install tmux -y 

# Install gazebo packages
RUN sudo apt-get install ros-iron-ros-ign-bridge ros-iron-teleop-twist-keyboard -y

# Add the following lines to the end of the file
RUN echo 'source /opt/ros/iron/setup.bash' >> /root/.bashrc

