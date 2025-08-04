# Start with an official ROS 2 base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

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
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
    
USER $USERNAME

# Install ROS 2 dependencies
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joint-state-publisher \
        ros-humble-controller-interface \
        ros-humble-hardware-interface \
        ros-humble-controller-manager \
        ros-humble-moveit-core \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-ros2controlcli \
        ros-humble-realtime-tools \
        ros-humble-control-msgs \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-generate-parameter-library \
        ros-humble-backward-ros \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Set the default shell to bash
SHELL [ "/bin/bash", "-c" ]

# Create workspace and clone repositories
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src && \
    cd /ros2_ws/src && \
    git clone https://github.com/frankarobotics/franka_ros.git && \
    git clone https://github.com/moveit/panda_moveit_config.git && \
    cd /ros2_ws && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Copy the entrypoint script and make it executable
COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
