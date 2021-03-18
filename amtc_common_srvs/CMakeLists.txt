cmake_minimum_required(VERSION 3.0.2)
project(amtc_common_srvs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_service_files(
  DIRECTORY srv/Utils
  FILES
    PointCloud2.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)
catkin_package(
   CATKIN_DEPENDS message_generation roscpp rospy sensor_msgs std_msgs
)
