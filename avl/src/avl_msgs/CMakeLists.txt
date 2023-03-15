#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_msgs ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_msgs)

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
)

# Specify include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
)

#===============================================================================
#                           MESSAGES/SERVICES/ACTIONS
#===============================================================================

# Declare the message files to be built
add_message_files( FILES
    MissionStatusMsg.msg
    NavigationMsg.msg
    RangeMsg.msg
    SafetyStatusMsg.msg
    VehicleStateMsg.msg
    ActuatorControlMsg.msg
    AhrsMsg.msg
    ByteArrayMsg.msg
    FinCalibrationMsg.msg
    FinsMsg.msg
    GpsMsg.msg
    GyrocompassMsg.msg
    XeosMsg.msg
    ImuMsg.msg
    IridiumSignalMsg.msg
    JoystickMsg.msg
    LinkquestDvlMsg.msg
    MagBearingMsg.msg
    MultibeamMsg.msg
    NavigationMsg.msg
    PwmMsg.msg
    TemperatureMsg.msg
    WhoiDataMsg.msg
    WhoiStatusMsg.msg
    WhoiTranspondersMsg.msg
    Float64SetpointMsg.msg
    LineSetpointMsg.msg
    OrbitSetpointMsg.msg
    PathfinderDvlMsg.msg
    CommsRxMsg.msg
    DataRxMsg.msg
    HeartbeatMsg.msg
)

# Declare the service files to be built
add_service_files( FILES
    MissionControlSrv.srv
    MissionModeSrv.srv
    SafetyAbortSrv.srv
    VehicleDynamicsSrv.srv
    ActuatorSrv.srv
    ByteArraySrv.srv
    MotorSrv.srv
    SailModeSrv.srv
    SetMagnetometerCalibrationSrv.srv
    StringSrv.srv
    StrobeSrv.srv
    CommsTxSrv.srv
    DataTxSrv.srv
    WhoiDownlinkSrv.srv
)

# Declare the action files to be built
add_action_files( DIRECTORY action FILES
    Mission.action
)

# Generate the language-specific message and service files
generate_messages( DEPENDENCIES std_msgs geometry_msgs actionlib_msgs )

#===============================================================================
#                              CONFIGURE PACKAGE
#===============================================================================

# Specify catkin-specific information to the build system
catkin_package(
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
    DEPENDS
    CFG_EXTRAS
)
