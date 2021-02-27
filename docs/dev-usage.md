# Running Scout on a developer's PC

- Run DualShock4 driver
  ```bash
  sudo ds4drv --hidraw
  ```
- Run the Scout stack
  ```bash
  roslaunch scout scout.launch
  ```

## Running Cartographer online

- Launch Cartographer and drive Scout around
  ```bash
  roslaunch scout mapping.launch
  ```
- When the map is ready run
  ```bash
  rosrun map_server map_saver -f filename
  ```

## Running Cartographer on a rosbag

```bash
roslaunch scout mapping_bag.launch bag_filename:=/path/to/rosbag
```
