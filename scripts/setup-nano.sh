#!/bin/bash

set -e

# install tools
apt update && apt install -y curl git python3-pip bluez-tools htop docker-compose joystick
pip3 install setuptools jetson-stats

# configure rootless docker
groupadd docker || true
usermod -aG docker $USER
newgrp docker

# setup YDLidar
git clone -b X4 --depth 1 https://github.com/YDLIDAR/ydlidar_ros.git
sh ydlidar_ros/startup/initenv.sh
rm -r ydlidar_ros

# update hostname
hostnamectl set-hostname scout-jetson
echo "127.0.0.1	scout-jetson" > /etc/hosts

# change the power profile to 5W
nvpmodel -m 1

# copying docker-compose file
curl -o ~/docker-compose.yml https://raw.githubusercontent.com/nyukhalov/scout/master/docker/compose/docker-compose.yml

