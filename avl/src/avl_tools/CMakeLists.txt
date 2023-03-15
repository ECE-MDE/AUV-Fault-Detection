#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_tools ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_tools)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-psabi)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: no build type specified, choosing Release")
  set(CMAKE_BUILD_TYPE Release)
endif()

#===============================================================================
#                              FIND DEPENDENCIES
#===============================================================================

# Find other Catkin packages required for build
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    sensor_msgs
    pcl_ros
    avl_core
    avl_msgs
    avl_asio
    avl_devices
)

# Find the Eigen3 package required for build
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Specify include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

#===============================================================================
#                           MESSAGES/SERVICES/ACTIONS
#===============================================================================

#===============================================================================
#                              CONFIGURE PACKAGE
#===============================================================================

# Specify catkin-specific information to the build system
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
    CATKIN_DEPENDS
        roscpp
        std_msgs
        visualization_msgs
        sensor_msgs
        pcl_ros
        avl_core
        avl_msgs
        avl_asio
        avl_devices
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------

# Actuator Calibration Node
add_executable(actuator_calibration_node src/node/actuator_calibration_node.cpp)
add_dependencies(actuator_calibration_node ${EXPORTED_TARGETS})
target_link_libraries(actuator_calibration_node node yaml-cpp ${catkin_LIBRARIES})

# Tail Junction Calibration Node
add_executable(tail_junction_calibration_node src/node/tail_junction_calibration_node.cpp)
add_dependencies(tail_junction_calibration_node ${EXPORTED_TARGETS})
target_link_libraries(tail_junction_calibration_node node asio aaf ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================
