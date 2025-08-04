# Base image: ROS Noetic desktop-full
# This image includes:
# - ROS core packages
# - RViz for visualization
# - Gazebo for simulation
# - Other GUI tools and dependencies
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic \
    DISABLE_ROS1_EOL_WARNINGS=1

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
    
USER $USERNAME

# Install ROS 1 dependencies
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-joint-state-publisher \
        ros-noetic-controller-interface \
        ros-noetic-controller-manager \
        ros-noetic-moveit-core \
        ros-noetic-moveit-ros-move-group \
        ros-noetic-moveit-ros-visualization \
        ros-noetic-moveit-kinematics \
        ros-noetic-moveit-planners-ompl \
        ros-noetic-joint-trajectory-controller \
        ros-noetic-gazebo-ros-control \
        ros-noetic-moveit-simple-controller-manager \
        ros-noetic-realtime-tools \
        ros-noetic-control-msgs \
        ros-noetic-rviz \
        ros-noetic-xacro \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

# Set the default shell to bash
SHELL [ "/bin/bash", "-c" ]

# Create workspace and clone repositories
WORKDIR /catkin_ws
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone https://github.com/frankarobotics/franka_ros.git && \
    git clone https://github.com/moveit/panda_moveit_config.git && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

# Copy the entrypoint script and make it executable
COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]

