FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgoogle-glog-dev \
    libomp-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
