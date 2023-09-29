cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_common)

set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)


# amtc_common
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)



target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  )

ament_export_targets(export_${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
  )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )


ament_package()