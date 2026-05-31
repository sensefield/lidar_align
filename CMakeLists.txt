cmake_minimum_required(VERSION 3.8)
project(lidar_align)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(Threads REQUIRED)

# NLOPT is frustratingly inconsistent in the name of its cmake file so we use our own.
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})
find_package(NLOPT REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/sensors.cpp
  src/aligner.cpp
  src/loader.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${NLOPT_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  rosbag2_cpp
  pcl_conversions
)
target_link_libraries(${PROJECT_NAME}
  ${NLOPT_LIBRARIES}
  ${PCL_LIBRARIES}
  Threads::Threads
)

add_executable(lidar_align_node
  src/lidar_align_node.cpp
)
target_link_libraries(lidar_align_node ${PROJECT_NAME})
ament_target_dependencies(lidar_align_node rclcpp)

install(
  TARGETS ${PROJECT_NAME} lidar_align_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch results config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  rosbag2_cpp
  pcl_conversions
)

ament_package()
