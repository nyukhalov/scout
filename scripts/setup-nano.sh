#!/bin/bash

set -e

# install tools
apt update && apt install -y curl git python3-pip
pip3 install setuptools

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
