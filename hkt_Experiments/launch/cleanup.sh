#!/bin/bash

ssh participant@10.38.119.97 pkill -f octomap_server
ssh participant@10.38.119.97 pkill -f buildBlend_exe
ssh participant@10.38.119.97 pkill -f auto_convert.sh
# stopping all rosnodes
rosnode kill --all
# stopping the gazebo client aka gui
killall gzclient
# stopping the gazebo server
killall gzserver


# lets get a bit more drastic
pkill -f ros/indigo
pkill -f /home/dementor/catkin_ws



# remove tmp blend files on cyb06
ssh participant@10.38.119.97 rm /home/participant/octomap_blend_files/*
# remove the tmp files
ssh participant@10.38.119.97 rm /tmp/mapfile.bt
ssh participant@10.38.119.97 rm -r /tmp/blender_*
ssh participant@10.38.119.97 rm  /tmp/textures/*

# kill the blender vr
ssh participant@10.38.119.97 killall -s SIGKILL blender2.76
