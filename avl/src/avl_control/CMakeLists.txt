#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_control ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_control)

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
    avl_core
    avl_msgs
    avl_devices
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
        avl_core
        avl_msgs
        avl_devices
        avl_navigation
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Control algorithm library
add_library(control src/algorithm/state_space_model.cpp
                    src/algorithm/pid.cpp)
add_dependencies(control ${EXPORTED_TARGETS})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------



# Height Control Node
add_executable(height_control_node src/node/height_control_node.cpp)
add_dependencies(height_control_node ${EXPORTED_TARGETS})
target_link_libraries(height_control_node node control ${catkin_LIBRARIES})

# H Infinity Attitude Control Node
add_executable(hinf_attitude_control_node src/node/hinf_attitude_control_node.cpp)
add_dependencies(hinf_attitude_control_node ${EXPORTED_TARGETS})
target_link_libraries(hinf_attitude_control_node node control ${catkin_LIBRARIES})

# Line Control Node
add_executable(line_control_node src/node/line_control_node.cpp)
add_dependencies(line_control_node ${EXPORTED_TARGETS})
target_link_libraries(line_control_node node control ${catkin_LIBRARIES})

# Orbit Control Node
add_executable(orbit_control_node src/node/orbit_control_node.cpp)
add_dependencies(orbit_control_node ${EXPORTED_TARGETS})
target_link_libraries(orbit_control_node node control ${catkin_LIBRARIES})

# PID Attitude Control Node
add_executable(pid_attitude_control_node src/node/pid_attitude_control_node.cpp)
add_dependencies(pid_attitude_control_node ${EXPORTED_TARGETS})
target_link_libraries(pid_attitude_control_node node control ${catkin_LIBRARIES})

# PID Depth Control Node
add_executable(pid_depth_control_node src/node/pid_depth_control_node.cpp)
add_dependencies(pid_depth_control_node ${EXPORTED_TARGETS})
target_link_libraries(pid_depth_control_node node control ${catkin_LIBRARIES})

# PID Speed Control Node
add_executable(pid_speed_control_node src/node/pid_speed_control_node.cpp)
add_dependencies(pid_speed_control_node ${EXPORTED_TARGETS})
target_link_libraries(pid_speed_control_node node control ${catkin_LIBRARIES})

# RPM Control Node
add_executable(rpm_control_node src/node/rpm_control_node.cpp)
add_dependencies(rpm_control_node ${EXPORTED_TARGETS})
target_link_libraries(rpm_control_node node control ${catkin_LIBRARIES})

# Speed Control Node
#add_executable(speed_control_node src/node/speed_control_node.cpp)
#add_dependencies(speed_control_node ${EXPORTED_TARGETS})
#target_link_libraries(speed_control_node node control ${catkin_LIBRARIES})

# SS Attitude Control Node
add_executable(ss_attitude_control_node src/node/ss_attitude_control_node.cpp)
add_dependencies(ss_attitude_control_node ${EXPORTED_TARGETS})
target_link_libraries(ss_attitude_control_node node control ${catkin_LIBRARIES})

# SS Depth Control Node
add_executable(ss_depth_control_node src/node/ss_depth_control_node.cpp)
add_dependencies(ss_depth_control_node ${EXPORTED_TARGETS})
target_link_libraries(ss_depth_control_node node control ${catkin_LIBRARIES})

# SS Line Control Node
add_executable(ss_line_control_node src/node/ss_line_control_node.cpp)
add_dependencies(ss_line_control_node ${EXPORTED_TARGETS})
target_link_libraries(ss_line_control_node node control ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
