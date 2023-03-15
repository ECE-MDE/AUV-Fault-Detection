#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_system ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_system)

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
    roslib
    message_generation
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    avl_core
    avl_msgs
    avl_comms
    avl_devices
    avl_guidance
    avl_navigation
    avl_control
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
#                              CONFIGURE PACKAGE
#===============================================================================

# Specify catkin-specific information to the build system
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
    CATKIN_DEPENDS
        roscpp
        roslib
        message_runtime
        std_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        actionlib
        actionlib_msgs
        avl_core
        avl_msgs
        avl_comms
        avl_devices
        avl_guidance
        avl_navigation
        avl_control
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

# Heartbeat Node
add_executable(heartbeat_node src/node/heartbeat_node.cpp)
add_dependencies(heartbeat_node ${EXPORTED_TARGETS})
target_link_libraries(heartbeat_node node asio comms ${catkin_LIBRARIES})

# Logging Control Node
add_executable(logging_control_node src/node/logging_control_node.cpp)
add_dependencies(logging_control_node ${EXPORTED_TARGETS})
target_link_libraries(logging_control_node node comms stdc++fs ${catkin_LIBRARIES})

# Safety Node
add_executable(safety_node src/node/safety_node.cpp)
add_dependencies(safety_node ${EXPORTED_TARGETS})
target_link_libraries(safety_node node comms ${catkin_LIBRARIES})

# System Command Node
add_executable(system_command_node src/node/system_command_node.cpp)
add_dependencies(system_command_node ${EXPORTED_TARGETS})
target_link_libraries(system_command_node node comms ${catkin_LIBRARIES})

# Mission Manager Node
add_executable(mission_manager_node src/node/mission_manager_node.cpp)
add_dependencies(mission_manager_node ${EXPORTED_TARGETS})
target_link_libraries(mission_manager_node node comms ${catkin_LIBRARIES})

# Parameter Node
add_executable(parameter_node src/node/parameter_node.cpp)
add_dependencies(parameter_node ${EXPORTED_TARGETS})
target_link_libraries(parameter_node node comms asio ${catkin_LIBRARIES})

# Resource Logger Node
add_executable(resource_logger_node src/node/resource_logger_node.cpp)
add_dependencies(heartbeat_node ${EXPORTED_TARGETS})
target_link_libraries(resource_logger_node node ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================
