cmake_minimum_required(VERSION 3.0.2)
project(amtc_common_srvs)

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

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)
catkin_package(
   CATKIN_DEPENDS message_generation roscpp rospy sensor_msgs std_msgs
)
