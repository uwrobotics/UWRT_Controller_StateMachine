cmake_minimum_required(VERSION 3.8)
project(my_publisher)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMsg.msg"
  DEPENDENCIES geometry_msgs
)

# Include generated headers
include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Add the executable for the publisher node
add_executable(my_publisher_node src/my_publisher_node.cpp)

# Link required dependencies
ament_target_dependencies(my_publisher_node rclcpp geometry_msgs)

# Install the node executable
install(TARGETS
  my_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime)

# Finalize the package
ament_package()