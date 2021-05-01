# Mapping

In order to map the environment both Scout and Mapping node must be run.
The mapping node should be run on a developer's computer.

- Follow the [Nano usage guide](./nano-usage.md) to start Scout
- The below commands should be executed on the developer's computer
  - Set up ROS environment
    ```bash
    export ROS_IP=<HOST IP>
    export ROS_MASTER_URI=http://scout-jetson:11311
    rostopic list
    ```
  - Launch Cartographer and drive Scout around
    ```bash
    roslaunch scout mapping.launch
    ```
  - When the map is ready run the following command to save it
    ```bash
    <PATH_TO_SCOUT>/scripts/save-map.sh <PATH_TO_MAP>
    ```

## Running Cartographer on a rosbag

```bash
roslaunch scout mapping_bag.launch bag_filename:=/path/to/rosbag
```
