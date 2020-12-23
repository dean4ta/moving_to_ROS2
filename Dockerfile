FROM ros:foxy-ros1-bridge
SHELL ["/bin/bash","-c"] 

# set environments
ENV ROS2_DISTRO foxy

# install building dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Setup Locales
RUN apt-get update && apt-get install -y locales
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"

# nvidia-container-runtime for setting up display environment
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen --purge $LANG && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=$LANG LC_ALL=$LC_ALL LANGUAGE=$LANGUAGE

# Set up timezone
ENV TZ 'America/Los_Angeles'
RUN echo $TZ > /etc/timezone && \
    rm /etc/localtime && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# Install basic dev and utility tools
RUN apt-get update && apt-get install -y \
    apt-utils \
    git \
    lsb-release \
    build-essential \
    stow \
    neovim \
    nano \
    tmux \
    wget \
    htop \
    unzip \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    ros-${ROS2_DISTRO}-turtlesim

# create ros directories
ENV ROS_WS=/root/ros_ws
WORKDIR ${ROS_WS}/src
COPY . ${ROS_WS}/src

WORKDIR $ROS_WS
ENV DEBIAN_FRONTEND noninteractive
RUN source /opt/ros/${ROS2_DISTRO}/setup.bash \
    && apt-get update \
    # Install dependencies
    && rosdep install -y --from-paths . --ignore-src -r --rosdistro ${ROS2_DISTRO}

RUN source /opt/ros/${ROS2_DISTRO}/setup.bash \
    # Build workspace
    && colcon build --symlink-install

COPY ./ros-entrypoint.sh /
ENTRYPOINT ["/ros-entrypoint.sh"]