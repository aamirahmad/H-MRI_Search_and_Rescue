cmake_minimum_required(VERSION 2.8.3)
project(traverse_flyto_safe)

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
  tk_draft_msgs
  tk_behavior
  telekyb_joystick
  telekyb_defines
)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## System dependencies are found with CMake's conventions
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${PROJECT_SOURCE_DIR}/include/behaviors
	${PROJECT_SOURCE_DIR}/include/control
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${ODEINTV2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_behavior
        src/behaviors/AbstractGraphNode.cpp
        src/behaviors/Neighbor.cpp
        src/behaviors/Flyto_safe.cpp
	
)

add_library(${PROJECT_NAME}
        src/control/FormationControl.cpp
        src/control/FormationElement.cpp
        src/control/FormationSlave.cpp
        src/control/FormationSlaveOptions.cpp
)

add_dependencies(${PROJECT_NAME}
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_behavior
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)


add_executable(traverse_flyto_ctrl_node src/control/Main.cpp)

add_executable(traverse_flyto_safe_slave_node 	src/control/FormationSlaveNode.cpp)



target_link_libraries(traverse_flyto_ctrl_node
	${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)
target_link_libraries(traverse_flyto_safe_slave_node
	${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_behavior
	${PROJECT_NAME}
       ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base tk_draft_msgs telekyb_interface telekyb_joystick cmake_modules 
    INCLUDE_DIRS src
    LIBRARIES be_mk_tools
)