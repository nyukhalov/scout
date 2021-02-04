# Setting up Jetson Nano

## Install docker

```bash
sudo apt install docker
sudo groupadd docker
sudo gpassed -a $USER docker
newgrp docker
```

## Install ds4drv and make it a systemd service

- Install ds4drv from the github repository.
  ```bash
  git clone https://github.com/chrippa/ds4drv.git
  cd ds4drv
  python3 setup.py sdist
  sudo pip3 install dist/ds4drv-0.5.1.tar.gz --system --upgrade
  sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ```
- Create and enable a systemd service
  ```bash
  sudo cp /path/to/scout/systemd/ds4srv.service /etc/systemd/system/ds4drv.service
  sudo systemctl enable ds4drv.service
  sudo systemctl start ds4drv.service
  ```

