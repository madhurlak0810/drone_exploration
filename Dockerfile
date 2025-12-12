# Autonomous Drone Exploration Docker Container
# Based on ROS2 Humble with complete simulation environment

FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/drone_ws

# Install system dependencies and additional ROS2 packages
RUN apt-get update && apt-get install -y \
    # Core development tools
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # ROS2 packages for simulation and navigation
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-teleop-twist-keyboard \
    # Python packages for ML and analytics
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-pandas \
    python3-sklearn \
    # Graphics and visualization (for headless with optional GUI)
    xvfb \
    x11vnc \
    novnc \
    supervisor \
    # Utilities
    nano \
    vim \
    htop \
    tree \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages for reinforcement learning
RUN pip3 install --no-cache-dir \
    transforms3d \
    pyyaml \
    rospkg \
    defusedxml

# Create workspace directory
WORKDIR ${WORKSPACE}

# Copy the entire workspace
COPY . ${WORKSPACE}/

# Set up ROS2 environment and build workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --parallel-workers 4 && \
    source install/setup.bash"

# Make launch scripts executable
RUN chmod +x ${WORKSPACE}/launch_final.sh \
    && chmod +x ${WORKSPACE}/launch_rviz_fixed.sh \
    && chmod +x ${WORKSPACE}/launch_single_frame.sh

# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source ROS2 environment' >> /entrypoint.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /entrypoint.sh && \
    echo 'source ${WORKSPACE}/install/setup.bash' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Set Gazebo model path' >> /entrypoint.sh && \
    echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${WORKSPACE}/install/drone_rl/share/drone_rl' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Create directories for logs and metrics' >> /entrypoint.sh && \
    echo 'mkdir -p ${WORKSPACE}/metrics' >> /entrypoint.sh && \
    echo 'mkdir -p ${WORKSPACE}/log' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Function to handle shutdown gracefully' >> /entrypoint.sh && \
    echo 'cleanup() {' >> /entrypoint.sh && \
    echo '    echo "Shutting down drone exploration system..."' >> /entrypoint.sh && \
    echo '    pkill -f "ros2|gazebo|gzserver|gzclient" || true' >> /entrypoint.sh && \
    echo '    echo "System shutdown complete"' >> /entrypoint.sh && \
    echo '    exit 0' >> /entrypoint.sh && \
    echo '}' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Set trap for graceful shutdown' >> /entrypoint.sh && \
    echo 'trap cleanup SIGTERM SIGINT' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Start the exploration system based on mode' >> /entrypoint.sh && \
    echo 'if [ "$1" = "headless" ]; then' >> /entrypoint.sh && \
    echo '    echo "Starting autonomous drone exploration in headless mode..."' >> /entrypoint.sh && \
    echo '    cd ${WORKSPACE}' >> /entrypoint.sh && \
    echo '    exec ./launch_final.sh' >> /entrypoint.sh && \
    echo 'elif [ "$1" = "gui" ]; then' >> /entrypoint.sh && \
    echo '    echo "Starting with GUI support (requires X11 forwarding)..."' >> /entrypoint.sh && \
    echo '    cd ${WORKSPACE}' >> /entrypoint.sh && \
    echo '    # Start VNC server for remote GUI access' >> /entrypoint.sh && \
    echo '    Xvfb :1 -screen 0 1024x768x16 &' >> /entrypoint.sh && \
    echo '    sleep 2' >> /entrypoint.sh && \
    echo '    export DISPLAY=:1' >> /entrypoint.sh && \
    echo '    x11vnc -display :1 -nopw -listen localhost -xkb -bg' >> /entrypoint.sh && \
    echo '    sleep 2' >> /entrypoint.sh && \
    echo '    # Start noVNC web interface' >> /entrypoint.sh && \
    echo '    /usr/share/novnc/utils/launch.sh --vnc localhost:5900 --listen 6080 &' >> /entrypoint.sh && \
    echo '    sleep 3' >> /entrypoint.sh && \
    echo '    echo "VNC Server: localhost:5900"' >> /entrypoint.sh && \
    echo '    echo "Web VNC: http://localhost:6080"' >> /entrypoint.sh && \
    echo '    exec ./launch_final.sh' >> /entrypoint.sh && \
    echo 'elif [ "$1" = "bash" ]; then' >> /entrypoint.sh && \
    echo '    echo "Starting interactive bash session..."' >> /entrypoint.sh && \
    echo '    cd ${WORKSPACE}' >> /entrypoint.sh && \
    echo '    exec /bin/bash' >> /entrypoint.sh && \
    echo 'else' >> /entrypoint.sh && \
    echo '    echo "Usage: docker run [options] drone-exploration [headless|gui|bash]"' >> /entrypoint.sh && \
    echo '    echo "  headless - Run simulation without GUI (recommended)"' >> /entrypoint.sh && \
    echo '    echo "  gui      - Run with VNC server for remote GUI access"' >> /entrypoint.sh && \
    echo '    echo "  bash     - Start interactive shell for development"' >> /entrypoint.sh && \
    echo '    echo ""' >> /entrypoint.sh && \
    echo '    echo "Examples:"' >> /entrypoint.sh && \
    echo '    echo "  docker run --rm drone-exploration headless"' >> /entrypoint.sh && \
    echo '    echo "  docker run --rm -p 5900:5900 drone-exploration gui"' >> /entrypoint.sh && \
    echo '    echo "  docker run --rm -it drone-exploration bash"' >> /entrypoint.sh && \
    echo '    exit 1' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Create supervisor configuration for multi-process management
RUN mkdir -p /etc/supervisor/conf.d && \
    echo '[supervisord]' > /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'nodaemon=true' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'user=root' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo '' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo '[program:drone_exploration]' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'command=/entrypoint.sh headless' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'autostart=false' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'autorestart=false' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'stderr_logfile=/var/log/drone_exploration.err.log' >> /etc/supervisor/conf.d/drone-exploration.conf && \
    echo 'stdout_logfile=/var/log/drone_exploration.out.log' >> /etc/supervisor/conf.d/drone-exploration.conf

# Expose ports for VNC (5900), noVNC (6080), and ROS communication (11311)
EXPOSE 5900 6080 11311

# Set working directory
WORKDIR ${WORKSPACE}

# Health check to monitor system status
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD pgrep -f "ros2|gazebo" > /dev/null || exit 1

# Default entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command (shows usage if no args provided)
CMD [""]

# Labels for metadata
LABEL maintainer="madhurlak0810"
LABEL description="Autonomous Drone Exploration System with Q-Learning and SLAM"
LABEL version="1.0"
LABEL ros.distro="humble"
LABEL gazebo.version="garden"