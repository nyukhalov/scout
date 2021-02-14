#!/bin/bash

set -e

# install tools
apt update && apt install -y curl git python3-pip bluez-tools htop docker-compose
pip3 install setuptools jetson-stats

# configure rootless docker
groupadd docker || true
usermod -aG docker $USER
newgrp docker

# install ds4drv
git clone --depth 1 https://github.com/chrippa/ds4drv.git
cd ds4drv
python3 setup.py sdist
pip3 install dist/ds4drv-0.5.1.tar.gz --system --upgrade
cp udev/50-ds4drv.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
cd ..
rm -r ds4drv

# make ds4drv a service
curl -o /etc/systemd/system/ds4drv.service https://raw.githubusercontent.com/nyukhalov/scout/master/systemd/ds4drv.service
systemctl enable ds4drv.service
systemctl start ds4drv.service

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

