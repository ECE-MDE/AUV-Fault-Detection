#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_devices ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_devices)

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
    pcl_ros
    avl_core
    avl_msgs
    avl_asio
    avl_comms
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
        pcl_ros
        avl_core
        avl_msgs
        avl_asio
        avl_comms
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Device node library
add_library(device src/device_node.cpp)
add_dependencies(device ${EXPORTED_TARGETS})
target_link_libraries(device comms ${catkin_LIBRARIES})

# Maestro USB driver library
add_library(maestro src/driver/maestro_pwm.cpp)
add_dependencies(maestro ${EXPORTED_TARGETS})
target_link_libraries(maestro usb-1.0)

# Linux joystick driver library
add_library(joystick src/driver/joystick.cpp)
add_dependencies(joystick ${EXPORTED_TARGETS})

# MIP packet library
add_library(mip src/protocol/mip/mip_field.cpp
                src/protocol/mip/mip_packet.cpp
                src/protocol/mip/mip.cpp)
add_dependencies(mip ${EXPORTED_TARGETS})


# AAF packet library
add_library(aaf src/protocol/aaf/aaf_field.cpp
                src/protocol/aaf/aaf_packet.cpp
                src/protocol/aaf/aaf.cpp)
add_dependencies(aaf ${EXPORTED_TARGETS})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------

# Actuators Node
add_executable(actuators_node src/node/actuators_node.cpp)
add_dependencies(actuators_node ${EXPORTED_TARGETS})
target_link_libraries(actuators_node node yaml-cpp ${catkin_LIBRARIES})

# ASCL GPIO Node
add_executable(ascl_gpio_node src/node/ascl_gpio_node.cpp)
add_dependencies(ascl_gpio_node ${EXPORTED_TARGETS})
target_link_libraries(ascl_gpio_node node asio comms yaml-cpp ${catkin_LIBRARIES})

# ASCL Power Node
add_executable(ascl_power_node src/node/ascl_power_node.cpp)
add_dependencies(ascl_power_node ${EXPORTED_TARGETS})
target_link_libraries(ascl_power_node node asio ${catkin_LIBRARIES})

# Garmin GPS Node
add_executable(garmin_gps_node src/node/garmin_gps_node.cpp)
add_dependencies(garmin_gps_node ${EXPORTED_TARGETS})
target_link_libraries(garmin_gps_node node asio device ${catkin_LIBRARIES})

# Iridium Modem Node
add_executable(iridium_modem_node src/node/iridium_modem_node.cpp)
add_dependencies(iridium_modem_node ${EXPORTED_TARGETS})
target_link_libraries(iridium_modem_node node asio ${catkin_LIBRARIES})

# Joystick Node
add_executable(joystick_node src/node/joystick_node.cpp)
add_dependencies(joystick_node ${EXPORTED_TARGETS})
target_link_libraries(joystick_node node joystick ${catkin_LIBRARIES})

# KVH IMU Node
add_executable(kvh_imu_node src/node/kvh_imu_node.cpp)
add_dependencies(kvh_imu_node ${EXPORTED_TARGETS})
target_link_libraries(kvh_imu_node node asio ${catkin_LIBRARIES})

# LinkQuest_DVL Node
add_executable(linkquest_dvl_node src/node/linkquest_dvl_node.cpp)
add_dependencies(linkquest_dvl_node ${EXPORTED_TARGETS})
target_link_libraries(linkquest_dvl_node node asio ${catkin_LIBRARIES})

# Linx GPS Node
add_executable(linx_gps_node src/node/linx_gps_node.cpp)
add_dependencies(linx_gps_node ${EXPORTED_TARGETS})
target_link_libraries(linx_gps_node node asio ${catkin_LIBRARIES})

# Maestro PWM Controller Node
add_executable(maestro_pwm_node src/node/maestro_pwm_node.cpp)
add_dependencies(maestro_pwm_node ${EXPORTED_TARGETS})
target_link_libraries(maestro_pwm_node node maestro ${catkin_LIBRARIES})

# Marine Sonic Sonar Node
add_executable(marinesonic_sonar_node src/node/marinesonic_sonar_node.cpp)
add_dependencies(marinesonic_sonar_node ${EXPORTED_TARGETS})
target_link_libraries(marinesonic_sonar_node node asio ${catkin_LIBRARIES})

# Microstrain AHRS Node
add_executable(microstrain_ahrs_node src/node/microstrain_ahrs_node.cpp)
add_dependencies(microstrain_ahrs_node ${EXPORTED_TARGETS})
target_link_libraries(microstrain_ahrs_node node asio mip yaml-cpp comms ${catkin_LIBRARIES})

# NMEA GPS Node
add_executable(nmea_gps_node src/node/nmea_gps_node.cpp)
add_dependencies(nmea_gps_node ${EXPORTED_TARGETS})
target_link_libraries(nmea_gps_node node asio ${catkin_LIBRARIES})

# Pathfinder DVL Node
add_executable(pathfinder_dvl_node src/node/pathfinder_dvl_node.cpp)
add_dependencies(pathfinder_dvl_node ${EXPORTED_TARGETS})
target_link_libraries(pathfinder_dvl_node node asio device ${catkin_LIBRARIES})

# Ping DSP sonar Node
add_executable(pingdsp_sonar_node src/node/pingdsp_sonar_node.cpp)
add_dependencies(pingdsp_sonar_node ${EXPORTED_TARGETS})
target_link_libraries(pingdsp_sonar_node node asio comms ${catkin_LIBRARIES})

# SiRF GPS Node
add_executable(sirf_gps_node src/node/sirf_gps_node.cpp)
add_dependencies(sirf_gps_node ${EXPORTED_TARGETS})
target_link_libraries(sirf_gps_node node asio ${catkin_LIBRARIES})

# Ping DSP sonar Node
add_executable(temp_sonar_node src/node/temp_sonar_node.cpp)
add_dependencies(temp_sonar_node ${EXPORTED_TARGETS})
target_link_libraries(temp_sonar_node node asio comms ${catkin_LIBRARIES})

# Tail Junction Node
add_executable(tail_junction_node src/node/tail_junction_node.cpp)
add_dependencies(tail_junction_node ${EXPORTED_TARGETS})
target_link_libraries(tail_junction_node node asio aaf comms ${catkin_LIBRARIES})

# Trimble GPS Node
add_executable(trimble_gps_node src/node/trimble_gps_node.cpp)
add_dependencies(trimble_gps_node ${EXPORTED_TARGETS})
target_link_libraries(trimble_gps_node node asio ${catkin_LIBRARIES})

# 2G Actuators Node
add_executable(twog_actuator_node src/node/twog_actuator_node.cpp)
add_dependencies(twog_actuator_node ${EXPORTED_TARGETS})
target_link_libraries(twog_actuator_node node asio device ${catkin_LIBRARIES})

# Valeport IPS Pressure Sensor Node
#add_executable(valeport_ips_node src/node/valeport_ips_node.cpp)
#add_dependencies(valeport_ips_node ${EXPORTED_TARGETS})
#target_link_libraries(valeport_ips_node serial_device ${catkin_LIBRARIES})

# Valeport miniCT Node
add_executable(valeport_minict_node src/node/valeport_minict_node.cpp)
add_dependencies(valeport_minict_node ${EXPORTED_TARGETS})
target_link_libraries(valeport_minict_node node asio device ${catkin_LIBRARIES})

# Valeport miniSVS Node
add_executable(valeport_minisvs_node src/node/valeport_minisvs_node.cpp)
add_dependencies(valeport_minisvs_node ${EXPORTED_TARGETS})
target_link_libraries(valeport_minisvs_node node asio device ${catkin_LIBRARIES})

# WHOI Micromodem Node
add_executable(whoi_micromodem_node src/node/whoi_micromodem_node.cpp)
add_dependencies(whoi_micromodem_node ${EXPORTED_TARGETS})
target_link_libraries(whoi_micromodem_node node asio device ${catkin_LIBRARIES})

# Xeos GPS Node
add_executable(xeos_gps_node src/node/xeos_gps_node.cpp)
add_dependencies(xeos_gps_node ${EXPORTED_TARGETS})
target_link_libraries(xeos_gps_node node asio device ${catkin_LIBRARIES})

# Sonardyne DVL Node
add_executable(sprintnav_ins_node src/node/sprintnav_ins_node.cpp)
add_dependencies(sprintnav_ins_node ${EXPORTED_TARGETS})
target_link_libraries(sprintnav_ins_node node asio device ${catkin_LIBRARIES})

# Tecnadyne Thruster Node
add_executable(tecnadyne_thruster_node src/node/tecnadyne_thruster_node.cpp)
add_dependencies(tecnadyne_thruster_node ${EXPORTED_TARGETS})
target_link_libraries(tecnadyne_thruster_node node asio device ${catkin_LIBRARIES})

# Digi Radio Node
add_executable(digi_radio_node src/node/digi_radio_node.cpp)
add_dependencies(digi_radio_node ${EXPORTED_TARGETS})
target_link_libraries(digi_radio_node node asio device ${catkin_LIBRARIES})

# Topside Digi Radio Node
add_executable(topside_digi_radio_node src/node/topside_digi_radio_node.cpp)
add_dependencies(topside_digi_radio_node ${EXPORTED_TARGETS})
target_link_libraries(topside_digi_radio_node node asio device ${catkin_LIBRARIES})

# Xeos Iridium Strobe Node
add_executable(xeos_iridium_strobe_node src/node/xeos_iridium_strobe_node.cpp)
add_dependencies(xeos_iridium_strobe_node ${EXPORTED_TARGETS})
target_link_libraries(xeos_iridium_strobe_node node asio device ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS mip aaf
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
