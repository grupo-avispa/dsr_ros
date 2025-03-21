ARG OVERLAY_WS=/opt/overlay_ws

FROM grupoavispa/cortex:development-ros-latest AS base
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
RUN mkdir -p src
COPY . ./src/dsr_ros

# Install dependencies
RUN apt update && apt install --no-install-recommends -y \
    python3-pip \
    ros-dev-tools \
    python3-vcstool \
    python3-colcon-clean
RUN rosdep update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -q -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
RUN colcon clean workspace --base-select build log -y

CMD ["/bin/bash"]