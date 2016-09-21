# HKT_Experiments
Experimental setups for the H-MRI search experiments. Package is prefixed HKT to remember that the first pilot of this experiment was done in a monastary in south germany called Heilegekreutztal. ( http://www.kloster-heiligkreuztal.de/meta/start.html )
# setting up 

## ROS side of things

mkdir -p ~/traverse/src && cd ~/traverse/src

copy these from the telekyb package that you should have downloaded if you read the readme in the upper level folder.
```
mav_comm
euroc_challenge3_simulator
euroc_comm
```
checkout hkt experiments with the start scripts and trajectories [No need if you have downloaded the public version already]
```
git clone https://github.molgen.mpg.de/TRaVERSE/HKT_Experiments.git
```
checkout the telekyb [No need now because you probably already did this if you read the readme in the upper level folder.]
```
svn co https://svn.tuebingen.mpg.de/humus-telekyb/hydro/trunk/ telekyb
```
the gazebo environment [No need if you have downloaded the public version already]
```
git clone https://github.molgen.mpg.de/TRaVERSE/traverse_gazebo.git
```
the full control mode [No need if you have downloaded the public version already]
```
git clone https://github.molgen.mpg.de/TRaVERSE/tk_flyto.git
```
checkout the semi auto mode [No need if you have downloaded the public version already]
```
git clone https://github.molgen.mpg.de/TRaVERSE/tk_formation_mm.git
```


install system dependencies
```
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping ros-indigo-octomap-msgs ros-indigo-octomap-ros ros-indigo-octomap-rviz-plugins ros-indigo-octomap-server
```

setting up the environment
```
export GAZEBO_MODEL_PATH=~/traverse/src/traverse_gazebo/models

export TELEKYB_EXTERNAL_LIBS=~/traverse/src/telekyb/external_libraries

export TELEKYB_CMAKE_SCRIPTS_DIR=~/traverse/src/telekyb/cmake_scripts
```

## VR side

checkout the VR environment
```
cd ~
git clone https://github.molgen.mpg.de/TRaVERSE/traverse_blend_vr.git
git checkout devel_v0.4_HKT_interface
```
get blender 2.76 from here 

http://download.blender.org/release/Blender2.76/

follow instruction here https://github.molgen.mpg.de/TRaVERSE/traverse_blend_vr/tree/devel_v0.4_HKT_interface/cpp/octomutils
 or in the traverse_blend_vr package present in the upper folder level.

```
export PYTHONPATH=$PYTHONPATH:~/traverse_blend_vr/python
export PYTHONPATH=$PYTHONPATH:~/traverse_blend_vr/blender/scripts
```
