# Building a Docker image for Jetson Nano

## Prepare the host OS

- Install qemu for emulating aarch64
  ```bash
  sudo apt install qemu qemu-user-static binfmt-support
  ```
- Register aarch64 emulation
  ```bash
  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
  ```

## Building the images

```bash
# building and publishing the base image
./scripts/docker-build-base.sh

# building and publishing the av image
./scripts/docker-build-av.sh
```

