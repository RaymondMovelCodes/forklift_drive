cmake_minimum_required(VERSION 2.8.3)
project(forklift_drive)

SET( CMAKE_BUILD_TYPE "Release" )

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    serial
    tf
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   modbus_client.msg
   modbus_server.msg
   
 )

 add_service_files(
  FILES
  chargingCmd.srv
  chargingStatus.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
  # Or other packages containing msgs
 )

 catkin_package(
  INCLUDE_DIRS include
  LIBRARIES modbus_client
  LIBRARIES modbus_server
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


 include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(

 #)

add_executable(forklift_drive
    src/forklift_drive_node.cpp
		src/forklift_drive.cpp
		)
target_link_libraries(forklift_drive  ${catkin_LIBRARIES} )

add_dependencies(forklift_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

