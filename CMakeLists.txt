cmake_minimum_required(VERSION 3.8)
project(negative_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Szintetikus terepgenerator (mock node a teszteleshez)
add_executable(mock_terrain_publisher src/mock_terrain_publisher.cpp)
ament_target_dependencies(mock_terrain_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(mock_terrain_publisher ${PCL_LIBRARIES})

# Negativ akadaly detektalo node
add_executable(detector_node src/detector_node.cpp)
ament_target_dependencies(detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  std_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(detector_node ${PCL_LIBRARIES})

install(TARGETS
  mock_terrain_publisher
  detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
