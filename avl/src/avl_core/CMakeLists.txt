#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_core ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_core)

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
)

# Find the Eigen3 package required for build
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Specify include directories
include_directories(
    include
    include/avl_core
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
        include/avl_core
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES
    CATKIN_DEPENDS
        roscpp
        message_runtime
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Library containing AVL binary protocol
add_library(protocol src/protocol/abort_packet.cpp
                     src/protocol/action_packet.cpp
                     src/protocol/command_packet.cpp
                     src/protocol/device_packet.cpp
                     src/protocol/event_packet.cpp
                     src/protocol/field.cpp
                     src/protocol/heartbeat_packet.cpp
                     src/protocol/micro_heartbeat_packet.cpp
                     src/protocol/micro_passthrough_packet.cpp
                     src/protocol/packet_validator.cpp
                     src/protocol/packet.cpp
                     src/protocol/parameter_list.cpp
                     src/protocol/parameter.cpp
                     src/protocol/passthrough_packet.cpp
                     src/protocol/response_packet.cpp)
add_dependencies(protocol ${EXPORTED_TARGETS})

# Library containing util functions
add_library(util src/util/byte.cpp
                 src/util/file.cpp
                 src/util/geo.cpp
                 src/util/logic.cpp
                 src/util/math.cpp
                 src/util/matrix.cpp
                 src/util/misc.cpp
                 src/util/network.cpp
                 src/util/vector.cpp
                 src/util/ros.cpp
                 src/util/string.cpp
                 src/util/time.cpp)
add_dependencies(util ${EXPORTED_TARGETS})

# Library containing all node base class functionality
add_library(node src/node.cpp
                 src/logger.cpp)
add_dependencies(node util ${EXPORTED_TARGETS})
target_link_libraries(node util protocol ${catkin_LIBRARIES})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS protocol util node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
