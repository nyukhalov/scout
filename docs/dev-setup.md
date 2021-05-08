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
       ros-noetic-rtabmap-ros ros-noetic-map-server ros-noetic-amcl \
       ros-noetic-laser-scan-matcher jstest-gtk
  ```
- Follow [the guide](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md) to install realsense libraries: `librealsense2-dkms` and `librealsense2-utils`. The libraries are required for being able to connect and initialize the camera.
- Create and initialize a catkin workspace. Install dependencies. `sudo rosdep init` will print an error if you have already executed it since installing ROS. This error can be ignored.
  ```bash
  mkdir catkin_ws
  cd catkin_ws
  wstool init src
  wstool set ydlidar -y -t src --git https://github.com/YDLIDAR/ydlidar_ros.git -v X4
  wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
  wstool update -t src

  ln -s <PATH_TO_SCOUT>/scout ./src/scout_ros

  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y

  src/cartographer/scripts/install_abseil.sh
  ```
- Build and install
  ```bash
  source /opt/ros/noetic/setup.zsh
  catkin_make_isolated --install --use-ninja
  ```
- Define the following environment variable
  ```bash
  export SCOUT_CATKIN_WS_PATH="/path/to/catkin_ws"
  ```
- Source the development script
  ```bash
  source <PATH_TO_SCOUT>/scripts/devel.zsh
  ```
- Configure the lidar alias
  ```bash
  roscd ydlidar/startup
  sudo chmod 777 ./*
  sudo sh initenv.sh
  ```

