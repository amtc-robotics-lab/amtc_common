cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_common_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(std_msgs  REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)


ament_export_dependencies(rosidl_default_runtime)

set(msg_files
"msg/PublisherInfo.msg"
"msg/SubscriberInfo.msg"
"msg/TimerInfo.msg"
"msg/ServiceClientInfo.msg"
"msg/ServiceServerInfo.msg"
"msg/CommunicationStatus.msg"
)


## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)





ament_package()