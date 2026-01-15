# Base image: Official ROS2 Humble with Ubuntu 22.04
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

# Keep things simple: run as root (no extra user)
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    DISPLAY=:0 \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    HOME=/root \
    USER=root

# Setup apt prerequisites (keys, repos)
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release ca-certificates apt-transport-https \
    wget git vim nano sudo locales software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Intel RealSense SDK key & repo
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
    tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/librealsense.list

# Docker repo key (for docker CLI install)
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg && \
    chmod a+r /etc/apt/keyrings/docker.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" \
    > /etc/apt/sources.list.d/docker.list

# Install system packages (including ROS packages used by Doosan)
RUN apt-get update && apt-get install -y \
    # Core utilities
    wget git vim nano sudo \
    locales software-properties-common \
    ca-certificates apt-transport-https \
    # Docker CLI
    docker-ce-cli \
    # Display and GUI support
    x11-apps mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    dbus-x11 libxext6 libx11-6 \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    # Network tools
    net-tools iputils-ping \
    # Intel RealSense SDK dependencies
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
    # Intel RealSense SDK packages (excluding DKMS - install on host instead)
    librealsense2-utils librealsense2-dev librealsense2-dbg \
    # Doosan-specific dependencies
    libpoco-dev libyaml-cpp-dev \
    # ROS2 packages for Doosan robot (installing relevant packages)
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-moveit \
    # Intel RealSense ROS2 wrapper packages
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    # Additional useful packages for vision processing
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-depth-image-proc \
    && rm -rf /var/lib/apt/lists/*

# Configure locale
RUN locale-gen en_US.UTF-8

# Gazebo / Ignition (kept as before)
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y libignition-gazebo6-dev ros-${ROS_DISTRO}-ros-gz && \
    rm -rf /var/lib/apt/lists/*

# Workspace location (root-owned)
ENV ROS2_WS=/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# Copy entrypoint script (created below)
COPY container-entrypoint.sh /usr/local/bin/container-entrypoint.sh
RUN chmod +x /usr/local/bin/container-entrypoint.sh

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc \
 && echo 'export LIBREALSENSE_USB_BACKEND=RSUSB' >> /root/.bashrc


# Ensure rosdep can be initialized (run as root)
RUN rosdep init || echo "rosdep already initialized"

# Set working dir
WORKDIR ${ROS2_WS}

# Expose common ports (optional)
EXPOSE 11311 12345

COPY container-entrypoint.sh /usr/local/bin/container-entrypoint.sh
RUN chmod +x /usr/local/bin/container-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/container-entrypoint.sh"]
CMD ["/bin/bash"]


