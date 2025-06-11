FROM osrf/ros:noetic-desktop-full

RUN apt update && export DEBIAN_FRONTEND=noninteractive && \
  apt install -y \
    python3-pip \
    python-is-python3 \
    ros-noetic-four-wheel-steering-msgs \
    ros-noetic-urdf-geometry-parser \
    ros-noetic-moveit \
    ros-noetic-twist-mux-msgs \
    ros-noetic-controller-interface \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    tmux \
    htop \
    git

RUN pip3 install vcstool catkin_tools mujoco

#  Unitree stuff
RUN git clone https://github.com/unitreerobotics/unitree_ros.git src/unitree_ros --recursive
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git src/unitree_sdk2 --recursive
