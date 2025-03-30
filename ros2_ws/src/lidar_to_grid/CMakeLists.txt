cmake_minimum_required(VERSION 3.8)
project(lidar_to_grid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_to_grid_node src/lidar_to_grid_node.cpp)
ament_target_dependencies(lidar_to_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
)
target_link_libraries(lidar_to_grid_node ${PCL_LIBRARIES})

install(TARGETS
  lidar_to_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 