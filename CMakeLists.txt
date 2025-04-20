cmake_minimum_required(VERSION 2.8.3)
project(hesai_lidar)

# Compile as C++11, supported in ROS Kinetic and newer

# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package( Boost REQUIRED  COMPONENTS thread)
find_package(PCL REQUIRED)

if (ANDROID)
    find_package(yaml-cpp REQUIRED)
    add_definitions(-DHAVE_NEW_YAMLCPP)
    find_package(Pcap REQUIRED)
    message("pcap libs: ${PCAP_LIBRARIES}, pcap dirs: ${PCAP_INCLUDE_DIRS}")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
    find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
    find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})
    pkg_check_modules(PCAP REQUIRED libpcap)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    rosbag
    std_msgs
    sensor_msgs
    message_runtime
    image_transport
    pcl_conversions
    message_generation
    tf
)
add_message_files(
    FILES
    PandarPacket.msg
    PandarScan.msg
)
add_service_files(
    FILES
    StartStop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
    CATKIN_DEPENDS message_runtime
)

###PandarGeneralRaw
add_library(PandarGeneral
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/input.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral_internal.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pcap_reader.cpp
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral.cc
)
target_include_directories(PandarGeneral PRIVATE
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCAP_LIBRARIES}
)

###PandarGeneralSDK
add_library(PandarGeneralSDK SHARED
    src/HesaiLidar_General_SDK/src/hesai_lidar_client.cpp
    src/HesaiLidar_General_SDK/src/hesai_lidar_client_wrap.cpp
    src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc
    src/HesaiLidar_General_SDK/src/tcp_command_client.c
    src/HesaiLidar_General_SDK/src/util.c
)
target_include_directories(PandarGeneralSDK PRIVATE
    src/HesaiLidar_General_SDK/
    src/HesaiLidar_General_SDK/include/
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(PandarGeneralSDK 
    PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCAP_LIBRARIES}
)

###hesai node
add_executable(hesai_lidar_node
    src/main.cc
)

target_include_directories(hesai_lidar_node PRIVATE
    src/HesaiLidar_General_SDK/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(hesai_lidar_node 
    ${catkin_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    PandarGeneralSDK
)

add_library(cloud_nodelet src/cloud_nodelet.cc)

target_include_directories(cloud_nodelet PRIVATE
    src/HesaiLidar_General_SDK/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(cloud_nodelet
    ${catkin_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    PandarGeneralSDK
)

add_executable(recording_controller
    src/recording_controller.cpp
)

target_include_directories(recording_controller PRIVATE
    src/HesaiLidar_General_SDK/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(recording_controller
    PandarGeneralSDK)

install(TARGETS
    hesai_lidar_node
    PandarGeneral
    PandarGeneralSDK
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY src/HesaiLidar_General_SDK/include/
  src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".git" EXCLUDE
)
