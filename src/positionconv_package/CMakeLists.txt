cmake_minimum_required(VERSION 3.8)
project(positionconv_package)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build the executable
add_executable(position_subscriber src/position_subscriber.cpp)

# Link dependencies
ament_target_dependencies(position_subscriber
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# Manually link yaml-cpp (fixes unresolved symbol issues)
target_link_libraries(position_subscriber yaml-cpp)

# Install the executable
install(TARGETS
  position_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch folder (optional)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()



