ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel "$(id -un $USER_UID)"; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo git python3-pip \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y

# Install ROS packages
RUN apt-get install -y ros-${ROS_DISTRO}-desktop ros-dev-tools \
    ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-twist-mux ros-${ROS_DISTRO}-twist-stamper \
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control

# Source ROS automatically for the user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV PATH="${ROS_ROOT}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${ROS_ROOT}/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="${ROS_ROOT}/lib/python3.10/site-packages:${PYTHONPATH}"

USER $USERNAME
CMD ["bash", "-i"]