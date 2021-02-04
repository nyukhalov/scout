# Setting up Scout for development on a x86 PC

- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Enable `ros-testing` repository. At the moment this is needed for installing
  realsense packages.
  ```bash
  # edit the content of sudo vim /etc/apt/sources.list.d/ros-latest.list
  deb http://packages.ros.org/ros-testing/ubuntu focal main
  ```
- Install the following system libraries
  ```bash
  sudo apt install python-is-python3 python3-rosdep python3-wstool \
       ros-noetic-joy ros-noetic-realsense2-camera ros-noetic-realsense2-description \
       ros-noetic-imu-filter-madgwick ros-noetic-robot-localization \
       ros-noetic-rtabmap-ros jstest-gtk
  ```
- Install ds4drv
  ```bash
  git clone https://github.com/chrippa/ds4drv.git
  cd ds4drv
  python3 setup.py sdist
  sudo pip3 install dist/ds4drv-0.5.1.tar.gz --system --upgrade
  ```
- Follow [the guide](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md) to install realsense libraries: `librealsense2-dkms` and `librealsense2-utils`. The libraries are required for being able to connect and initialize the camera.
- Create and initialize a catkin workspace. Install dependencies. `sudo rosdep init` will print an error if you have already executed it since installing ROS. This error can be ignored.
  ```bash
  mkdir catkin_ws/src
  cd catkin_ws/src
  ln -s /path/to/scout
  git clone -b X4 --depth 1 https://github.com/YDLIDAR/ydlidar_ros.git
  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
  ```
- Build and install
  ```bash
  source /opt/ros/noetic/setup.zsh
  catkin_make
  ```
- Define the following environment variable
  ```bash
  export SCOUT_CATKIN_WS_PATH="/path/to/catkin_ws"
  ```
- Source the development script
  ```bash
  source /path/to/scout/scripts/devel.zsh
  ```
- Configure the lidar alias
  ```bash
  roscd ydlidar/startup
  sudo chmod 777 ./*
  sudo sh initenv.sh
  ```
