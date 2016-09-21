#!/bin/bash

PORT=11311

if [ $# -eq 1 ]; then
	PORT=$1
fi

export ROS_MASTER_URI=http://localhost:$PORT

roslaunch traverse_formation_mm  mm_setup.launch port:=$PORT --screen
