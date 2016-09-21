#!/bin/bash

ROBOS=$1
if [ $# -ne 1 ]; then
        echo "usage: $0 <number of robots>"
        exit
fi
ROBOT_IDS="["
HUMAN_INPUT="[1"

x=$((-1*$ROBOS))

for i in $(seq 1 $(($ROBOS))); do
	x=$(($x+2))
	y=-5
	id=$(($i+42))
	roslaunch telekyb_gazebo spawn_firefly_withID.launch roboID:=$id X:=$x Y:=$y &
	sleep 5 
	roslaunch tk_formation_mm firefly_tkcore_gazebo.launch robotID:=$id &
	sleep 5 
	roslaunch tk_formation_mm formation_slave_gazebo.launch robotID:=$id --screen &
	sleep 3
	rosrun hkt_experiments uav_state_tf_closer $id & 
	sleep 3
	rosrun topic_tools relay /firefly_$id/xtion/depth/points /firefly/xtion/depth/points &
	sleep 3

        if [ $i -eq 1 ]; then
                ROBOT_IDS=$ROBOT_IDS$id
        else
                ROBOT_IDS=$ROBOT_IDS","$id
        fi
        if [ $i -gt 1 ]; then
                HUMAN_INPUT=$HUMAN_INPUT",0"
        fi


done

roslaunch tk_formation_mm formation_master.launch robotIDs:=$ROBOT_IDS"]"  humanInput:=$HUMAN_INPUT"]" &

roslaunch tk_formation_mm joystick_TH.launch joyPubName:=joy_exp &
