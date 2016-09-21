#!/bin/bash

ROBOS=3

ROBOTS=( 1 2 3 )
MODES=( full_control semi_auto_control full_autonomous )
MAPS=( arena_HKT_1.world arena_HKT_2.world arena_HKT_3.world )

typeset -i max=$((${#ROBOTS[*]}*${#MODES[*]}*${#MAPS[*]})) r=0 m=0 k=0
for i in $(seq 0 $(($max-1))); do
	echo './setup_exp.sh' ${ROBOTS[$r]} ${MODES[$m]} ${MAPS[$k]}
	r=$r+1
	if [ $r -eq 3 ]; then
		r=0
		m=$m+1
	fi
	if [ $m -eq 3 ]; then
		m=0
		k=$k+1
	fi
	
done
