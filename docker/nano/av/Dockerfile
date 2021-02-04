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
        python3-pip \
        joystick \
        libusb-dev \
        libbluetooth-dev \
        libcwiid-dev \
        libspnav-dev \
        bluez && \
    pip3 install wheel pyserial && \
    git clone https://github.com/chrippa/ds4drv.git && \
    cd ds4drv && \
    python3 setup.py sdist && \
    pip3 install dist/*.tar.gz --system && \
    cd .. && \
    rm -rf ds4drv

COPY ./scout /ros_ws/src/scout/

# building all ROS packages
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

# other tools
RUN apt install -y vim

# cleanup
RUN rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./docker/scripts/ros_entrypoint_av.sh /ros_entrypoint_av.sh
RUN echo "source /ros_ws/devel/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint_av.sh"]
