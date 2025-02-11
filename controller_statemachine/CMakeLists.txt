cmake_minimum_required(VERSION 3.8)
project(controller_statemachine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

find_package(rclcpp REQUIRED)
add_dependencies(tutorial_interfaces ${catkin_EXPORTED_TARGETS})

add_executable(publish_vel src/publish_vel.cpp)
ament_target_dependencies(publish_vel rclcpp)

install(TARGETS
  publish_vel
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(publish_vel "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
