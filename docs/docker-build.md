# Building a Docker image for Jetson Nano

## Prepare the host OS

- Install qemu for emulating aarch64
  ```bash
  sudo apt install qemu qemu-user-static
  ```
- Install podman for mounting volumes for building the image
  ```bash
  . /etc/os-release
  echo "deb https://download.opensuse.org/repositories/devel:/kubic:/libcontainers:/stable/xUbuntu_${VERSION_ID}/ /" | sudo tee /etc/apt/sources.list.d/devel:kubic:libcontainers:stable.list
  curl -L https://download.opensuse.org/repositories/devel:/kubic:/libcontainers:/stable/xUbuntu_${VERSION_ID}/Release.key | sudo apt-key add -
  sudo apt update
  sudo apt -y upgrade
  sudo apt -y install podman
  ```

## Building the images

```bash
# building and publishing the base image
podman build -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static -t banzzaj/scout-ros-base -f ./docker/nano/base/Dockerfile .
podman push banzzaj/scout-ros-base:latest

# building and publishing the av image
podman build -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static -t banzzaj/scout-ros-av -f ./docker/nano/av/Dockerfile .
podman push banzzaj/scout-ros-av
```

