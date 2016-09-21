project(octomutils)
cmake_minimum_required(VERSION 2.8)

find_package(octomap REQUIRED)

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

add_executable(${PROJECT_NAME}_exe src/main.cpp)
add_executable(buildBlend_exe src/buildBlend.cpp)


target_link_libraries(${PROJECT_NAME}_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
)

target_link_libraries(buildBlend_exe
      ${PROJECT_NAME}
      ${OCTOMAP_LIBRARIES}
      ${PYTHON_LIBRARIES}
)

