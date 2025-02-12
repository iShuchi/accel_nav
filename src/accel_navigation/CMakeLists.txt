cmake_minimum_required(VERSION 3.8)
project(accel_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executable
add_executable(waypoint_navigation src/waypoint_navigation.cpp)
ament_target_dependencies(waypoint_navigation rclcpp rclcpp_action nav2_msgs visualization_msgs)

# Install executables
install(TARGETS waypoint_navigation
  DESTINATION lib/${PROJECT_NAME})

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
