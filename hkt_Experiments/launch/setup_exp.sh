#!/bin/bash

if [ $# -ne 3 ]; then
        echo "usage: $0 <number of robots> <mode> <map>"
        exit
fi

ROBOS=$1
MODE=$2
MAP=$3

ROBOT_IDS="["
HUMAN_INPUT="[1"
if [ $ROBOS -gt 2 ]; then
	Xs=( -8 -6.8 -7.25 -8.7 -9.15 )
	Ys=( -6.75 -7.6 -9 -9 -7.57 )
else
	Xs=( -8 -6.91 )
	Ys=( -6.75 -8.615 )
fi


for i in $(seq 0 $(($ROBOS-1))); do
	id=$(($i+1))
	roslaunch telekyb_gazebo spawn_firefly_withID.launch roboID:=$id X:=${Xs[$i]}  Y:=${Ys[$i]} &
	sleep 10	
	if [ "$MODE" != "full_autonomous" ]; then 
		roslaunch tk_formation_mm firefly_tkcore_gazebo.launch robotID:=$id &
	fi
	sleep 2
	rosrun hkt_experiments uav_state_tf_closer $id & 
	sleep 2
	rosrun topic_tools relay /firefly_$id/xtion/depth/points /firefly/xtion/depth/points &
	sleep 2
	# set the array with the robot ids
        if [ $i -eq 0 ]; then
       	        ROBOT_IDS=$ROBOT_IDS$id
        else
       	        ROBOT_IDS=$ROBOT_IDS","$id
        fi

	case $MODE in
	    full_control)
        	echo 'Starting full controll exp'
		roslaunch traverse_flyto_safe formation_slave_gazebo.launch robotID:=$id --screen &
        	if [ $i -gt 0 ]; then
                	HUMAN_INPUT=$HUMAN_INPUT",1"
	        fi
	        ;;
	    semi_auto_control)
        	echo 'Starting semi autonomous formation control'
		roslaunch traverse_formation_mm formation_slave_gazebo.launch robotID:=$id --screen &
        	if [ $i -gt 0 ]; then
                	HUMAN_INPUT=$HUMAN_INPUT",0"
	        fi
	        ;;
	    full_autonomous)
        	;;
	    *)
        	echo "Unknown Mode of operation"
	esac
done
sleep 10 
case $MODE in
	full_autonomous)
		roslaunch hkt_experiments ${ROBOS}_UAV_fullyAutonomousMotion.launch &
	;;
	semi_auto_control)
		roslaunch traverse_formation_mm formation_master.launch robotIDs:=$ROBOT_IDS"]"  humanInput:=$HUMAN_INPUT"]" &
	;;
	full_control)
		roslaunch traverse_flyto_safe formation_master.launch robotIDs:=$ROBOT_IDS"]"  humanInput:=$HUMAN_INPUT"]" &
	;;
	*)
        	echo "Unknown Mode of operation"
esac
# start octomap server
roslaunch hkt_experiments octomap_mapping.launch --screen 
# get the current time
date
