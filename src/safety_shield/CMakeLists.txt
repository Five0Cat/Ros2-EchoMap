cmake_minimum_required(VERSION 3.8)
project(safety_shield)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(local_mapper REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_executable(safety_shield_node
  src/safety_shield.cpp
)
ament_target_dependencies(safety_shield_node
  rclcpp nav_msgs sensor_msgs geometry_msgs local_mapper visualization_msgs 
  tf2
  tf2_geometry_msgs
  # tf2_ros
)
# 5) Install targets and runtime resources
install(TARGETS safety_shield_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install launch files (if you have a launch/ folder)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

###pubisher###
add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs local_mapper)
install(TARGETS publisher_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
