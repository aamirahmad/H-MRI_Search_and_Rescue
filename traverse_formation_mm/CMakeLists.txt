cmake_minimum_required(VERSION 2.4.6)
project(traverse_formation_mm)

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

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs telekyb_base tk_behavior tk_draft_msgs telekyb_interface telekyb_joystick cmake_modules telekyb_defines telekyb_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)


list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)


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
        src/behaviors/Formation.cpp
	
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

add_executable(traverse_formation_ctrl_node src/control/Main.cpp)

add_executable(traverse_formation_slave_node 	src/control/FormationSlaveNode.cpp)


target_link_libraries(traverse_formation_ctrl_node
	${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)
target_link_libraries(traverse_formation_slave_node
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
