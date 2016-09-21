#!/bin/bash

if [ $# -lt 1 ]; then
	echo "need the qcID as argument"
	exit 0
fi

roslaunch traverse_formation_mm  start_formation_slave_with_qcID.launch qcID:=$1 --screen

