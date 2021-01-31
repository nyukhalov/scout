FROM banzzaj/scout-ros-base:latest

RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

RUN apt update

# downloading scout's dependencies and its dependencies.
# the libraries do not exist in the repository for the aarch64 platform,
# so we have to do it manually.

# ros-noetic-joy and its dependencies
RUN cd ./src && \
    git clone --depth 1 --no-tags https://github.com/ros/roslint.git && \
    git clone --depth 1 --no-tags https://github.com/ros/diagnostics.git && \
    git clone --depth 1 --no-tags https://github.com/ros-drivers/joystick_drivers.git && \
    apt install -y --no-install-recommends \
        joystick \
        libusb-dev \
        libbluetooth-dev \
        libcwiid-dev \
        libspnav-dev \
        bluez

# building all ROS packages
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

# cleanup
RUN rm -rf /var/lib/apt/lists/*

