cmake_minimum_required(VERSION 2.8.3)
project(amtc_common)
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  amtc_common_msgs
  amtc_common_srvs
  diagnostic_msgs
  geometry_msgs
  roscpp
  rospy
  rostest
  std_msgs
  sensor_msgs
  tf
  urdf
  xacro
  actionlib
)

catkin_python_setup()
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES amtc_common
   CATKIN_DEPENDS amtc_common_msgs diagnostic_msgs geometry_msgs roscpp rospy rostest std_msgs sensor_msgs tf urdf xacro
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# amtc_common
add_library(amtc_common
  src/Communications.cpp
  src/Debug.cpp
  src/Enum.cpp
  src/NodeConfigure.cpp
)
add_dependencies(amtc_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} amtc_common_srvs_generate_messages_cpp)

target_link_libraries(amtc_common
 ${catkin_LIBRARIES}
)

# test_communications
add_executable(test_communication test/test_Communications.cpp)
target_link_libraries(test_communication
  ${catkin_LIBRARIES}
  amtc_common
)

# test_Communications_subscriber_with_caller_id
add_executable(test_Communications_subscriber_with_caller_id test/test_Communications_subscriber_with_caller_id.cpp)
target_link_libraries(test_Communications_subscriber_with_caller_id
        ${catkin_LIBRARIES}
        amtc_common
        )


# test_NodeConfigure
add_executable(test_NodeConfigure test/test_NodeConfigure.cpp)
target_link_libraries(test_NodeConfigure
  ${catkin_LIBRARIES}
  amtc_common
)
