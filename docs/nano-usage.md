# Running Scout on Jetson Nano

## Running Scout

The easiest way to run Scout is by running the `scout-ros-av` Docker image:

```bash
ssh roman@scout-jetson
docker-compose up -d scout
```

## Connecting to Jetson Nano's ROS from host

```
export ROS_MASTER_URI=http://scout-jetson:11311
rostopic list
```
