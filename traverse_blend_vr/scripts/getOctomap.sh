#!/bin/bash

COLORED_OCTOMAP=/tmp/colored_octomap.ot
BINARY_OCTOMAP=binary_octomap.bt
BLENDFILES_OUTPUTPATH=/tmp/blendfiles
if [! -d "$BLENDFILES_OUTPUTPATH" ]; then
	# Control will enter here if $DIRECTORY does not exists.
	mkdir -p $BLENDFILES_OUTPUTPATH
fi

while [ true ]
do
	# place the service call to notify rgbdslam to save a octomap
	rosservice call /rgbdslam/ros_ui_s save_octomap $COLORED_OCTOMAP

	# get the octomap onto this machine 
	# for this to work the ssh keys need to be setup properly
	scp -i ~/.ssh/cyb03_rsa dementor@cyb03:$COLORED_OCTOMAP $BINARY_OCTOMAP
	
	# convert the colored .ot to a .bt binary tree
	convert_octree $COLORED_OCTOMAP $BINARY_OCTOMAP
	
	# create the blendfiles
	./buildBlend_exe $BINARY_OCTOMAP -o $BLENDFILES_OUTPUTPATH
done;


