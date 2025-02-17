cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(uwrt_ros_msg REQUIRED)     

# Add include directory for your package's headers
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
include_directories(${INCLUDE_DIR})

# Gather header files (optional)
file(GLOB_RECURSE HEADERS "include/*.hpp")

# Add source file(s)
set(SOURCE_FILE "src/state_machine.cpp")

add_executable(state_machine_talker ${SOURCE_FILE} ${HEADERS})

# Link against the imported target for nlohmann_json
target_link_libraries(state_machine_talker PRIVATE nlohmann_json::nlohmann_json)

# Declare dependencies for ament (add other dependencies as needed)
ament_target_dependencies(state_machine_talker
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  uwrt_ros_msg
  nlohmann_json
)

# Install executable
install(TARGETS
  state_machine_talker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
