cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_common_srvs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(std_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(srv_files
"srv/PointCloud2.msg"
)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
  geomety_msgs   sensor_msgs   std_msgs
)



ament_package()