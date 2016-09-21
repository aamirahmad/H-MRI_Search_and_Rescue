#!/bin/bash

BUILDBLEND_PATH=/home/participant/traverse_blend_vr/cpp/octomutils/build2
MAP_FILE=/tmp/mapfile.bt
OUTPUT_PATH=~/octomap_blend_files/


while true; do
	time=$(date +"%s")
	# getting the new map
	rosrun octomap_server octomap_saver $MAP_FILE
	
	# converting the map 
	$BUILDBLEND_PATH/buildBlend_exe $MAP_FILE -o $OUTPUT_PATH
	while [ "$(($time + 10))"  -gt "$(date +"%s")" ]; do
		sleep 1
	done
	# removing blend tmp files
	echo "removing blend tmp files"
	rm $OUTPUT_PATH/*blend[0-9]
done
