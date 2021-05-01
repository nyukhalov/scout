# Running Scout on Jetson Nano

## Set up

A DualShock joystick is used for teleoperation.
The joystick can be connected by running:

```bash
bluetoothctl connect <MAC_ADDR>
```

## Running Scout

The easiest way to run Scout is by running the `banzzaj/scout-ros-av` Docker image:

```bash
ssh roman@scout-jetson
docker-compose up -d scout
```

