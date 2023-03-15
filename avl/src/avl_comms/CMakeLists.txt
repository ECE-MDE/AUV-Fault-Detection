#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_comms ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_comms)

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
    avl_core
    avl_msgs
    avl_asio
)

# Specify include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
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
        avl_core
        avl_msgs
        avl_asio
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Library containing all comms functionality
add_library(comms src/packet_handler.cpp
                  src/command_handler.cpp)
add_dependencies(comms ${EXPORTED_TARGETS})
target_link_libraries(comms ${catkin_LIBRARIES})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------

# Acoustic Channel Node
add_executable(acoustic_channel_node src/node/acoustic_channel_node.cpp)
add_dependencies(acoustic_channel_node ${EXPORTED_TARGETS})
target_link_libraries(acoustic_channel_node node comms ${catkin_LIBRARIES})

# Comms Manager Node
add_executable(comms_manager_node src/node/comms_manager_node.cpp)
add_dependencies(comms_manager_node ${EXPORTED_TARGETS})
target_link_libraries(comms_manager_node node comms ${catkin_LIBRARIES})

# Ethernet Channel Node
add_executable(ethernet_channel_node src/node/ethernet_channel_node.cpp)
add_dependencies(ethernet_channel_node ${EXPORTED_TARGETS})
target_link_libraries(ethernet_channel_node node asio ${catkin_LIBRARIES})

# Iridium Channel Node
add_executable(iridium_channel_node src/node/iridium_channel_node.cpp)
add_dependencies(iridium_channel_node ${EXPORTED_TARGETS})
target_link_libraries(iridium_channel_node node ${catkin_LIBRARIES})

# Test Nodes -------------------------------------------------------------------

# BSD Passthrough Test Node
add_executable(bsd_passthrough_test_node src/test_node/bsd_passthrough_test_node.cpp)
add_dependencies(bsd_passthrough_test_node ${EXPORTED_TARGETS})
target_link_libraries(bsd_passthrough_test_node node asio ${catkin_LIBRARIES})

# BSD Tester Node
add_executable(bsd_tester_node src/test_node/bsd_tester_node.cpp)
add_dependencies(bsd_tester_node ${EXPORTED_TARGETS})
target_link_libraries(bsd_tester_node node asio ${catkin_LIBRARIES})

# FSD Tester Node
add_executable(fsd_tester_node src/test_node/fsd_tester_node.cpp)
add_dependencies(fsd_tester_node ${EXPORTED_TARGETS})
target_link_libraries(fsd_tester_node node asio ${catkin_LIBRARIES})

# Packet Handler Test Node
add_executable(packet_handler_test_node src/test_node/packet_handler_test_node.cpp)
add_dependencies(packet_handler_test_node ${EXPORTED_TARGETS})
target_link_libraries(packet_handler_test_node node asio comms ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS comms
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
