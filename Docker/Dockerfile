FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle
ENV CATKIN_WS=/catkin_ws2


# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-navigation \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-gmapping \
    ros-noetic-slam-gmapping \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace and copy project
WORKDIR ${CATKIN_WS}
RUN mkdir -p ${CATKIN_WS}/src

# Copy project files
COPY . ${CATKIN_WS}/src/turtlebot_slam_navigation/

# Build project
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd ${CATKIN_WS} \
    && catkin_make \
    && echo 'source ${CATKIN_WS}/devel/setup.bash' >> ~/.bashrc"

# Create entry point script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source "/opt/ros/noetic/setup.bash"' >> /entrypoint.sh && \
    echo 'source "${CATKIN_WS}/devel/setup.bash"' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
