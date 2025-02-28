
roslaunch telekyb_gazebo spawn_firefly_withID.launch roboID:=1 X:=-8 Y:=-6.75 &

sleep 10s

roslaunch telekyb_gazebo spawn_firefly_withID.launch roboID:=2 X:=-6.91 Y:=-8.615 &

sleep 10s

rosrun hkt_experiments uav_state_tf_closer 1 &

sleep 2s

rosrun hkt_experiments uav_state_tf_closer 2 &


sleep 2s

roslaunch hkt_experiments 2_UAV_fullyAutonomousMotion.launch &

sleep 2s

rosrun topic_tools relay /firefly_1/xtion/depth/points /firefly/xtion/depth/points &

sleep 2s

rosrun topic_tools relay /firefly_2/xtion/depth/points /firefly/xtion/depth/points &


sleep 5s

# rosrun tf staticransform_publisher 0 0 0 0 0 0 1 firefly_1/xtion_depth_optical_frame firefly_1/firefly_1/xtion_depth_optical_frame 30 &
# 
# sleep 5s
# 
# rosrun tf staticransform_publisher 0 0 0 0 0 0 1 firefly_2/xtion_depth_optical_frame firefly_2/firefly_2/xtion_depth_optical_frame 30 &

roslaunch hkt_experiments octomap_mapping.launch