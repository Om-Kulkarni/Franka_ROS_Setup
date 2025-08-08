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
        cmake \
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
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-ros-controllers \
        ros-${ROS_DISTRO}-gazebo-ros-control \
        ros-${ROS_DISTRO}-combined-robot-hw \
        ros-${ROS_DISTRO}-libfranka \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-boost-sml \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

# Set the default shell to bash
SHELL [ "/bin/bash", "-c" ]

# Create workspace and copy repositories from project root
WORKDIR /catkin_ws
RUN mkdir -p /catkin_ws/src

# Copy repositories from project root
COPY franka_ros /catkin_ws/src/franka_ros/
COPY panda_moveit_config /catkin_ws/src/panda_moveit_config/

# Initialize rosdep and build workspace
RUN sudo rosdep init || true && \
    rosdep update --include-eol-distros && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

# Copy the entrypoint script and make it executable
COPY Docker/ros1/franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]

