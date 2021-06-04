#!/bin/sh

set -e

IMG_NAME=banzzaj/scout-ros-base
DOCKERFILE_PATH=./docker/nano/base/Dockerfile

echo "Building $IMG_NAME image from $DOCKERFILE_PATH"
podman build -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static -t $IMG_NAME -f $DOCKERFILE_PATH .

echo "Publishing $IMG_NAME"
podman push $IMG_NAME

