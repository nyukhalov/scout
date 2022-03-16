# Visualizing data in PlotJuggler

## Installation

Execute the below command to install the `plotjuggler` ros package on your host PC:

```shell
sudo apt install ros-noetic-plotjuggler-ros
```

## Usage

On your host PC execute the following commands:

```shell
cd <ROS_WORKSPACE>
export ROS_MASTER_URI=http://scout-jetson:11311
export ROS_IP=<HOST IP>
source /opt/ros/noetic/setup.zsh
source <PATH_TO_SCOUT>/scripts/devel.zsh
rosrun plotjuggler plotjuggler
```