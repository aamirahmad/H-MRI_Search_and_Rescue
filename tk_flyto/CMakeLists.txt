cmake_minimum_required(VERSION 2.8.3)
project(tk_filtflyto)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  telekyb_interface
  telekyb_base
  cmake_modules
  telekyb_msgs
  telekyb_srvs
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tk_eruff
#  CATKIN_DEPENDS geometry_msgs octomap roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	./include
	${Boost_INLCUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)


#add_dependencies(${PROJECT_NAME}
#        telekyb_msgs_generate_messages_cpp
# 	telekyb_srvs_generate_messages_cpp
#        mav_msgs_generate_messages
#)


link_directories(
        ${catkin_LIBRARY_DIRS}
        ${Boost_LIBRARY_DIRS}
)


## Declare a cpp executable
add_executable(tk_filtflyto_node src/tk_filtflyto.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(tk_eruff_node tk_eruff_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tk_filtflyto_node
 ${catkin_LIBRARIES}
 ${Boost_LIBRARIES}
)

