FROM ros:noetic

# Install required packages and tools
RUN apt-get update && apt-get install -y \
    x11-apps \
    tmux \
    zsh \
    htop \
    nvtop \
    git \
    sudo \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    mesa-utils \
    python3-pip \
    python-is-python3
RUN apt-get update && apt-get install -y \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-tf2-ros \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-controller-manager \
    ros-noetic-urdf-geometry-parser \
    ros-noetic-moveit \
    ros-noetic-twist-mux-msgs \
    ros-noetic-controller-interface \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller
RUN rm -rf /var/lib/apt/lists/*
RUN pip3 install catkin_tools

# oh-my-zsh install
ENV ZSH_THEME="powerlevel10k/powerlevel10k"
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" \
    -x \
    -p git \
    -p ssh-agent \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions

ENV DEBIAN_FRONTEND=noninteractive

# Create user 'robot' and add to sudoers
RUN useradd -m -s /bin/bash robot && echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER robot

# .zshrc
RUN echo '\
export TERM=xterm-256color\n\
export PS1="%n@%m:%~$ "\n\
source /opt/ros/noetic/setup.zsh\n\
' >> /home/robot/.zshrc

# Set DISPLAY and allow X11 forwarding
ENV DISPLAY=:1
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

WORKDIR /home/robot/ws

SHELL ["/bin/zsh", "-c"]
CMD ["zsh"]
