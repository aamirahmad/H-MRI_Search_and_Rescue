
cmake_minimum_required(VERSION 2.8)

project(misc_py)

find_package(catkin REQUIRED COMPONENTS	rospy)

catkin_package()

## Uncomment if the package has a setup.py
catkin_python_setup()

#install(PROGRAMS 
#	bin/viconPoseTransform_node.py
#	bin/getStaticTransfromQC_Cam.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
