# scout

Self-Driving RC Car

## Installation

- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Install the following libraries
  ```bash
  sudo apt install python-is-python3 python3-rosdep
  ```
- Create a catkin workspace
  ```bash
  mkdir catkin_ws/src
  cd catkin_ws/src
  ln -s /path/to/carla-ai
  cd ..
  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src -r
  catkin_make
  ```
- Define the following environment variables
  ```bash
  export SCOUT_CATKIN_WS_PATH="/path/to/catkin_ws"
  ```
- Source the development script
  ```bash
  source /path/to/scout/scripts/devel.zsh
  ```

## Running

1. Run the Scout stack
   ```bash
   roslaunch scout scout.launch
   ```
