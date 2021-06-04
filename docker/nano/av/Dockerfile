FROM banzzaj/scout-ros-base:latest

RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

RUN apt-get update

# downloading scout's dependencies and its dependencies.
# the libraries do not exist in the repository for the aarch64 platform,
# so we have to do it manually.

# ros-noetic-joy and its dependencies
RUN cd ./src && \
    git clone --depth 1 --no-tags https://github.com/ros/roslint.git && \
    git clone --depth 1 --no-tags https://github.com/ros/diagnostics.git && \
    git clone --depth 1 --no-tags https://github.com/ros-drivers/joystick_drivers.git && \
    apt install -y --no-install-recommends \
        python3-pip \
        joystick \
        libusb-dev \
        libbluetooth-dev \
        libcwiid-dev \
        libspnav-dev \
        bluez \
        vim && \
    pip3 install wheel pyserial

# install YDLidar ROS package
RUN cd ./src && \
    git clone -b X4 --depth 1 https://github.com/YDLIDAR/ydlidar_ros.git && \
    cd ..

# copy scout source code
COPY ./scout /ros_ws/src/scout/

# building all ROS packages
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

# cleanup
RUN rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./docker/scripts/ros_entrypoint_av.sh /ros_entrypoint_av.sh
RUN echo "source /ros_ws/devel/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint_av.sh"]
