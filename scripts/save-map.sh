#!/bin/bash

MAP_PATH=$1

echo "Saving map to '${MAP_PATH}'"
rosrun map_server map_saver -f ${MAP_PATH}
rosservice call /finish_trajectory 0
rosservice call /write_state ${MAP_PATH}.pbstream true
