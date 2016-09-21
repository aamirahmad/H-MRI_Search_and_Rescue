# traverse_blend_vr
VR environment based on blender game engine


# Setting up the VR

## installing dependencies
``` 
    sudo apt-get install python3.4-dev ros-indigo-roslib ros-indigo-rospy
```
installing rospkg by hand
```
    git clone git://github.com/ros/rospkg.git
    sudo pyhton3.4 setup.py install
```
installing pyinotify
```
    git clone https://github.com/seb-m/pyinotify.git
    cd cd pyinotify/
    sudo python3.4 setup.py install
```
```
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.4/dist-packages
```

## Build the blender python module based on this:

https://wiki.blender.org/index.php/Dev:Doc/Building_Blender/Linux/Ubuntu/CMake

checkout the tag of version 2.76
```
git checkout tags/2.76
```
DON'T DO THIS. 
```
git submodule foreach git checkout master
git submodule foreach git pull --rebase origin master
```
with these cmake settings
```
cmake .. \ 
    -DCMAKE_INSTALL_PREFIX=/opt/blender2.76 \
    -DWITH_INSTALL_PORTABLE=OFF \
    -DWITH_BUILDINFO=OFF \
    -DWITH_GAMEENGINE=ON \
    -DWITH_PLAYER=OFF \
    -DWITH_PYTHON_INSTALL=OFF \
    -DWITH_PYTHON_MODULE=ON \
    -DPYTHON_SITE_PACKAGES=/usr/local/lib/python3.4/dist-packages/
```
then 
```
    sudo checkinstall
```
to get a deb package which can be easily be reomved
