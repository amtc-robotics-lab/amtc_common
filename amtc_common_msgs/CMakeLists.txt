cmake_minimum_required(VERSION 3.0.2)
project(amtc_common_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  PublisherInfo.msg
  SubscriberInfo.msg
  TimerInfo.msg
  ServiceClientInfo.msg
  ServiceServerInfo.msg
  CommunicationStatus.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs message_generation message_runtime sensor_msgs std_msgs
)




