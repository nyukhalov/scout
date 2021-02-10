#!/bin/bash

apt install -y curl git docker python3 python3-pip
groupadd docker
gpassed -a $USER docker
newgrp docker

git clone https://github.com/chrippa/ds4drv.git
cd ds4drv
python3 setup.py sdist
pip3 install dist/ds4drv-0.5.1.tar.gz --system --upgrade
cp udev/50-ds4drv.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
cd ..
rm -r ds4drv

curl -o /etc/systemd/system/ds4drv.service https://raw.githubusercontent.com/nyukhalov/scout/master/systemd/ds4drv.service
systemctl enable ds4drv.service
systemctl start ds4drv.service
