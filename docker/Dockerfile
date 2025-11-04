######################################################################
# - Base stage
#   - This stage serves as the base image for the following stages.
#   - Based on Isaac Sim official image
######################################################################

FROM nvcr.io/nvidia/isaac-sim:4.5.0 AS base

LABEL org.opencontainers.image.title="Pegasus Simulator"
LABEL org.opencontainers.image.authors="scx@gapp.nthu.edu.tw"
LABEL org.opencontainers.image.licenses="MIT"

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

######################################################################
# - User setup stage
#   - Create a non-root user with default bash shell.
######################################################################

FROM base AS user-setup

USER root

RUN apt-get update \
    && apt-get install -y sudo gosu \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

######################################################################
# - ROS2 Installation stage
#   - Install ROS2 Humble and related packages
######################################################################

FROM user-setup AS ros2-setup

# Use root for all apt operations
USER root
WORKDIR /home/ubuntu
ENV DEBIAN_FRONTEND=noninteractive

# ARG variables need to be redeclared in each stage
ARG ROS_DISTRO=humble

# Fix base image APT conflicts (deb.sury.org brotli issue)
RUN set -e \
    && rm -f /etc/apt/sources.list.d/*sury* /etc/apt/sources.list.d/php* 2>/dev/null || true \
    && sed -i '/deb\.sury\.org/d' /etc/apt/sources.list || true \
    && apt-get update \
    && printf "Package: *\nPin: release n=jammy\nPin-Priority: 700\n" > /etc/apt/preferences.d/pin-jammy

# Install locale, repo and prerequisites
RUN apt-get install -y --no-install-recommends \
    locales \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository universe \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Add ROS2 official repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros-archive-keyring.gpg \
    && mv /tmp/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg \
    && sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# Install RViz dependencies (fix freetype/fontconfig & brotli conflict)
RUN set -e \
 && apt-mark unhold libbrotli1 || true \
 && apt-get purge -y 'libbrotli*' || true \
 && rm -f /etc/apt/sources.list.d/*sury* /etc/apt/sources.list.d/php* 2>/dev/null || true \
 && sed -i '/deb\.sury\.org/d' /etc/apt/sources.list || true \
 && rm -rf /var/lib/apt/lists/* \
 && apt-get clean && apt-get update \
 && apt-get install -y --no-install-recommends \
      libbrotli1=1.0.9-2build6 \
      libfreetype6-dev \
      libfontconfig1-dev \
      libxft-dev \
      libxrender-dev \
 && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages (official Desktop Install method)
RUN apt-get update && apt-get upgrade -y \
 && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    # ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-ament-cmake \
    python3-colcon-common-extensions \
#
# Additional ROS2 packages (commented out to reduce build time and image size)
# Uncomment as needed for your specific use case:
#
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    # ros-${ROS_DISTRO}-nav2-msgs \
    # ros-${ROS_DISTRO}-nav2-util \
    # ros-${ROS_DISTRO}-navigation2 \
    # ros-${ROS_DISTRO}-nav2-bringup \
    # ros-${ROS_DISTRO}-turtlebot3* \
    # ros-${ROS_DISTRO}-urdf-launch \
    # ros-${ROS_DISTRO}-joint-state-publisher \
    # ros-${ROS_DISTRO}-joint-state-publisher-gui \
    # ros-${ROS_DISTRO}-behaviortree-cpp \
    # ros-${ROS_DISTRO}-generate-parameter-library \
#
# Isaac Sim Documentation recommended packages
# reference: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_ros.html
#
    ros-humble-vision-msgs \
    ros-humble-ackermann-msgs \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

######################################################################
# - Tools Installation stage
#   - Install common tools for development.
######################################################################
FROM ros2-setup AS tools

RUN apt-get update && apt-get install -y \
    tmux \
    htop \
    tree \
    vim \
    git \
    wget \
    curl \
    unzip \
    usbutils \
    python3-pip \
    bash-completion

# Install PX4-Autopilot and QGroundControl dependencies
RUN apt-get install -y \
    make \
    cmake \
    python3-pip \
    # QGroundControl dependencies
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    libfuse2 \
    fuse \
    && rm -rf /var/lib/apt/lists/*

# Add user to dialout group for serial port access (required for QGroundControl)
RUN usermod -a -G dialout ubuntu

# Remove modemmanager to avoid conflicts with QGroundControl
RUN apt-get update && apt-get remove -y modemmanager && rm -rf /var/lib/apt/lists/*

# Switch to ubuntu user for Python package installations
USER ubuntu
WORKDIR /home/ubuntu

# Set proper ownership for Isaac Sim installation
RUN sudo chown -R ubuntu:ubuntu /isaac-sim

# Install PX4 Python dependencies
RUN pip3 install --no-cache-dir \
    kconfiglib \
    jinja2 \
    empy \
    jsonschema \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    future

# Clone and build PX4-Autopilot v1.14.3
RUN git clone --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git /home/ubuntu/PX4-Autopilot && \
    cd /home/ubuntu/PX4-Autopilot && \
    git submodule update --init --recursive && \
    (timeout 900 make px4_sitl_default || true) && \
    if [ ! -f build/px4_sitl_default/bin/px4 ]; then \
        echo "ERROR: PX4 build failed - binary not found"; \
        exit 1; \
    else \
        echo "PX4 build succeeded"; \
    fi

# Install Pegasus Simulator Python dependencies
RUN /isaac-sim/python.sh -m pip install --no-cache-dir \
    numpy \
    scipy \
    pymavlink \
    omegaconf

# Install Pegasus Simulator extension (will use mounted volume at runtime)
# Extension is installed from the mounted /home/ubuntu/PegasusSimulator/extensions
# This happens at build time but can be overridden by the mounted volume
RUN mkdir -p /home/ubuntu/PegasusSimulator/extensions && \
    chown -R ubuntu:ubuntu /home/ubuntu/PegasusSimulator

# Download and install QGroundControl
RUN cd /home/ubuntu && \
    wget -O QGroundControl.AppImage https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage && \
    chmod +x QGroundControl.AppImage

# Final system cleanup (as root)
USER root
RUN apt-get update && apt-get dist-upgrade -y \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

######################################################################
# - Final stage
#   - Set up the environment and entrypoint.
######################################################################

FROM tools AS final

ARG USERNAME=ubuntu

# Set up the user environment
ENV TZ=Asia/Taipei
ENV SHELL=/bin/bash
ENV TERM=xterm-256color

# Isaac Sim environment variables
ENV ISAACSIM_PATH=/isaac-sim
ENV ISAACSIM_PYTHON=/isaac-sim/python.sh
ENV ISAACSIM=/isaac-sim/isaac-sim.sh
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=N
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ROS2 environment variables (fixed to Humble with FastRTPS)
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# PX4 environment variables
ENV PX4_DIR=/home/ubuntu/PX4-Autopilot

USER root
WORKDIR /home/$USERNAME

# Set up bashrc
COPY .bashrc .bashrc.conf
RUN cat .bashrc.conf >> /home/$USERNAME/.bashrc && rm .bashrc.conf && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]