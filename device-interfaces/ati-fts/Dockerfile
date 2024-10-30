# Use the official ROS Noetic base image
FROM ros:noetic-ros-core

# Set the working directory
WORKDIR /root

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    vim tmux \
    && rm -rf /var/lib/apt/lists/*

# Install tools to build ros ws
RUN apt-get update && apt-get install -y \
    libjsoncpp-dev \
    python3-wstool \
    python3-pip \
    python3-pybind11 \
    python3-catkin-tools

# Set up the environment
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic

# Source the ROS setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Copy the script and install dependencies
RUN mkdir -p /root/ati-fts
COPY ./ros_fts.py /root/ati-fts
# COPY ./run.sh /root/ati-fts
RUN pip3 install serial
WORKDIR /root/ati-fts
# RUN chmod +x run.sh
USER root

SHELL ["/bin/bash", "-c"]
