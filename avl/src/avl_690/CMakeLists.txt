#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_control ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 3.0.2)

# Package name
project(avl_690)

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
    avl_asio
    avl_control
    avl_core
    avl_guidance
    avl_navigation
    roscpp
    std_srvs
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
        avl_asio
        avl_control 
        avl_core 
        avl_guidance 
        avl_navigation 
        roscpp
        std_srvs
    DEPENDS
)

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------

# Primitive Guidance Node (With Canards)
add_executable(primitive_guidance_canards_node src/guidance/primitive_guidance_canards_node.cpp)
add_dependencies(primitive_guidance_canards_node control)
target_link_libraries(primitive_guidance_canards_node node asio ${catkin_LIBRARIES})

# PID Attitude Control Node (With Canards)
add_executable(pid_attitude_control_canards_node src/control/pid_attitude_control_canards_node.cpp)
add_dependencies(pid_attitude_control_canards_node control)
target_link_libraries(pid_attitude_control_canards_node node control ${catkin_LIBRARIES})
