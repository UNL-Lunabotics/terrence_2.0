FROM ros:humble 
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG INPUT_GID=995

# Delete user if it exists in container (e.g Ubuntu Jammy: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Add user to input group to access input devices
RUN getent group input || groupadd -g $INPUT_GID input \
    && usermod -aG input $USERNAME
# Install necessary packages and ROS2 packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3-pip \
    git \
    evtest \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard
# Set the default shell to bash instead of sh
ENV SHELL /bin/bash

# Set the default user when starting the container
USER $USERNAME
# Set the default command to bash
CMD ["/bin/bash"]