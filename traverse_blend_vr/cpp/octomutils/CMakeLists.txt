project(octomutils)
cmake_minimum_required(VERSION 2.8)

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


find_package(octomap REQUIRED)

set(Python_ADDITIONAL_VERSIONS 3.4)
FIND_PACKAGE(PythonLibs 3.4 REQUIRED)


include_directories(
	${OCTOMAP_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    src/binaryoctree.cpp
    src/compareoctrees.cpp
    src/octomaptoblend.cpp
    )

add_executable(compareOctrees_exe src/main.cpp)
add_executable(buildBlend_exe src/buildBlend.cpp)
add_executable(updateOctree_exe src/updateOctree.cpp)
add_executable(showSubtrees_exe src/showSubtrees.cpp)
add_executable(mapUpdate_exe src/mapUpdate.cpp)


target_link_libraries(compareOctrees_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
)

target_link_libraries(buildBlend_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
      ${PYTHON_LIBRARIES}
)

target_link_libraries(updateOctree_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
)
target_link_libraries(showSubtrees_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
)
target_link_libraries(mapUpdate_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
      ${PYTHON_LIBRARIES}
)
