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
./scripts/docker-build-base.sh

# building and publishing the av image
./scripts/docker-build-av.sh
```

