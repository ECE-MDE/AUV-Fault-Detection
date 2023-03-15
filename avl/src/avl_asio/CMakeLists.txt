#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_asio ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_asio)

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
    avl_core
)

# Specify include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
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
        avl_core
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Library containing all ASIO functionality
add_library(asio src/serial_port.cpp
                 src/tcp_socket.cpp
                 src/tcp_server.cpp
                 src/udp_socket.cpp)
add_dependencies(asio ${EXPORTED_TARGETS})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Test Nodes -------------------------------------------------------------------

# Packet Test Node
add_executable(packet_test_node src/test_node/packet_test_node.cpp)
add_dependencies(packet_test_node ${EXPORTED_TARGETS})
target_link_libraries(packet_test_node node asio ${catkin_LIBRARIES})

# Serial Port Test Node
add_executable(serial_port_test_node src/test_node/serial_port_test_node.cpp)
add_dependencies(serial_port_test_node ${EXPORTED_TARGETS})
target_link_libraries(serial_port_test_node node asio ${catkin_LIBRARIES})

# TCP Server Test Node
add_executable(tcp_server_test_node src/test_node/tcp_server_test_node.cpp)
add_dependencies(tcp_server_test_node ${EXPORTED_TARGETS})
target_link_libraries(tcp_server_test_node node asio ${catkin_LIBRARIES})

# TCP Socket Test Node
add_executable(tcp_socket_test_node src/test_node/tcp_socket_test_node.cpp)
add_dependencies(tcp_socket_test_node ${EXPORTED_TARGETS})
target_link_libraries(tcp_socket_test_node node asio ${catkin_LIBRARIES})

# UDP Socket Test Node
add_executable(udp_socket_test_node src/test_node/udp_socket_test_node.cpp)
add_dependencies(udp_socket_test_node ${EXPORTED_TARGETS})
target_link_libraries(udp_socket_test_node node asio ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS asio
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
