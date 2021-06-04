#!/bin/sh

set -e

IMG_NAME=banzzaj/scout-ros-av
DOCKERFILE_PATH=./docker/nano/av/Dockerfile

echo "Building $IMG_NAME image from $DOCKERFILE_PATH"
docker build -t $IMG_NAME -f $DOCKERFILE_PATH .

echo "Publishing $IMG_NAME"
docker push $IMG_NAME

