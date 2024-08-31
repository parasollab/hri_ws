cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_map)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(pointcloud_subscriber src/pointcloud_subscriber.cpp)
add_executable(pointcloud_publisher src/pointcloud_publisher.cpp)

ament_target_dependencies(pointcloud_subscriber
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  octomap
  message_filters
  pcl_msgs
  octomap_msgs
)

ament_target_dependencies(pointcloud_publisher
  rclcpp
  sensor_msgs
)

target_link_libraries(pointcloud_subscriber
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud_subscriber
  pointcloud_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
