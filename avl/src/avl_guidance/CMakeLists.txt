#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_guidance ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_guidance)

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
    message_generation
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    avl_core
    avl_msgs
    avl_control
    avl_navigation
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

## Declare the action files to be built
#add_action_files( DIRECTORY action FILES
#    Mission.action
#)
#
## Generate the language-specific message and service files
#generate_messages( DEPENDENCIES std_msgs geometry_msgs actionlib_msgs )

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
        message_runtime
        std_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        actionlib
        actionlib_msgs
        avl_core
        avl_msgs
        avl_control
        avl_navigation
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Library containing all node base class functionality
add_library(dubins src/algorithm/dubins.cpp)
add_dependencies(dubins ${EXPORTED_TARGETS})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------

# Primitive Guidance Node
add_executable(primitive_guidance_node src/node/primitive_guidance_node.cpp)
add_dependencies(primitive_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(primitive_guidance_node node asio ${catkin_LIBRARIES})

# Dive Guidance Node
add_executable(dive_guidance_node src/node/dive_guidance_node.cpp)
add_dependencies(dive_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(dive_guidance_node node asio ${catkin_LIBRARIES})

# Figure Eight Guidance Node
add_executable(figure_eight_guidance_node src/node/figure_eight_guidance_node.cpp)
add_dependencies(figure_eight_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(figure_eight_guidance_node node dubins ${catkin_LIBRARIES})

# Follow Guidance Node
add_executable(follow_guidance_node src/node/follow_guidance_node.cpp)
add_dependencies(follow_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(follow_guidance_node node comms ${catkin_LIBRARIES})

# Spiral Guidance Node
add_executable(spiral_guidance_node src/node/spiral_guidance_node.cpp)
add_dependencies(spiral_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(spiral_guidance_node node asio ${catkin_LIBRARIES})

# Waypoint Guidance Node
add_executable(waypoint_guidance_node src/node/waypoint_guidance_node.cpp)
add_dependencies(waypoint_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(waypoint_guidance_node node dubins asio ${catkin_LIBRARIES})

# Line Guidance Node
add_executable(line_guidance_node src/node/line_guidance_node.cpp)
add_dependencies(line_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(line_guidance_node node dubins asio ${catkin_LIBRARIES})

# Loiter Guidance Node
add_executable(loiter_guidance_node src/node/loiter_guidance_node.cpp)
add_dependencies(loiter_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(loiter_guidance_node node asio ${catkin_LIBRARIES})

# Command Guidance Node
add_executable(command_guidance_node src/node/command_guidance_node.cpp)
add_dependencies(command_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(command_guidance_node node asio ${catkin_LIBRARIES})

# Yaw Zigzag Guidance Node
add_executable(yaw_zigzag_guidance_node src/node/yaw_zigzag_guidance_node.cpp)
add_dependencies(yaw_zigzag_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(yaw_zigzag_guidance_node node asio ${catkin_LIBRARIES})

# Pitch Zigzag Guidance Node
add_executable(pitch_zigzag_guidance_node src/node/pitch_zigzag_guidance_node.cpp)
add_dependencies(pitch_zigzag_guidance_node ${EXPORTED_TARGETS})
target_link_libraries(pitch_zigzag_guidance_node node asio ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
