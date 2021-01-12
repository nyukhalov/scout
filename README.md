# scout

Self-Driving RC Car

## Installation

- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Install the following libraries
  ```bash
  sudo apt install python-is-python3 python3-rosdep python3-wstool ros-noetic-joy jstest-gtk
  ```
- Create and initialize a catkin workspace
  ```bash
  mkdir catkin_ws
  cd catkin_ws
  wstool init src
  wstool merge -t src https://raw.githubusercontent.com/nyukhalov/scout/master/scout_ros.rosinstall
  wstool update -t src
  ```
- Install dependencies. `sudo rosdep init` will print an error if you have already executed it since installing ROS. This error can be ignored.
  ```bash
  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
  ```
- Build and install
  ```bash
  source /opt/ros/noetic/setup.zsh
  catkin_make
  source devel/setup.zsh
  ```
- Modify `ds4drv/backends/hidraw.py` as in https://github.com/chrippa/ds4drv/pull/105/files
  ```
  sudo vim /usr/local/lib/python3.8/dist-packages/ds4drv/backends/hidraw.py
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

## Running

1. Run DualShock4 driver
  ```bash
  sudo ds4drv --hidraw
  ```
2. Run the Scout stack
   ```bash
   roslaunch scout scout.launch
   ```

## Debugging DualShock4

Make sure `ds4drv` is running, then run

```bash
jstest-gtk
```
